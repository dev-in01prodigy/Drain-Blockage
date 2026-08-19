#include "arduinoFFT.h"

// ==================== 模式选择 ====================
// 主动模式：程序驱动电磁铁敲击，不检测 IO3 输入
// 被动模式：不驱动电磁铁，等待 IO3 上升沿触发
// 烧录前取消对应注释
// #define MODE_ACTIVE
#define MODE_PASSIVE

// ==================== 硬件引脚 ====================
#define CHANNEL          A0
#define EMAG_PIN         2
#define TRIGGER_PIN      3
#define PUMP_INTAKE_PIN  4    // 进水泵：水槽 -> 水管（IO4）：HIGH=开，LOW=关
#define PUMP_DRAIN_PIN   5    // 出水泵：水管 -> 水槽（IO5）：HIGH=开，LOW=关
#define WATER_HIGH_PIN   6    // 高水位检测（IO6）：浮空，LOW=有水，HIGH=无水
#define WATER_LOW_PIN    7    // 低水位检测（IO7）：浮空，LOW=有水，HIGH=无水

// ==================== 时序参数 ====================
#define KNOCK_MS         12
#define CYCLE_MS         2000
#define TRIGGER_TIMEOUT  50000   // 等待触发上升沿的超时时间(ms)

// ==================== FFT 参数 ====================
#define SAMPLES          256
const double samplingFrequency = 1000000.0 / (25000.0 / SAMPLES);  // 25ms, ≈10240 Hz

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);

// ==================== 测试元数据 ====================
enum PipeState { EMPTY, FULL };
const char* stateNames[] = { "Empty", "Full" };

int positionId = 1;
PipeState currentState = EMPTY;
int autoCompareCount = 50;        // 每批敲击次数
int remainingCount = 0;           // 本批剩余次数，0=待命
bool headerPrinted = false;
volatile bool triggerDetected = false;  // IO3 上升沿中断置位
volatile bool triggerBusy = false;                  // 防止捕获中重入
volatile uint32_t lastFallingUs = 0;              // 测试：记录最近一次IO3上升沿时间戳(us)
const uint32_t RISING_DEBOUNCE_US = 1000;         // ISR防抖：忽略1ms内的重复毛刺
bool passiveReady = false;                        // 被动AUTO：第1个IO3边沿到达后才开始响应（过滤抽/注水期误触发）
bool firstTriggerReceived = false;                  // 被动AUTO：静默等待第1个IO3边沿   // 中断标记：IO2 上升沿

// ==================== 泵状态 ====================
bool pumpIntakeRunning = false;   // 进水泵是否运行
bool pumpDrainRunning  = false;   // 出水泵是否运行

// ==================== AUTO 自动采集状态机 ====================
// 0=未执行  1=准备空管(检测水位)  2=抽水排空中  3=采集空管数据
// 4=注水等待(IO6 LOW)  5=采集满水数据
// ==================== 全局频段缓冲 ====================
// AUTO 状态机 + doKnockAndAnalyze 共用，避免每个 case 重复在栈上分配
double g_bands[8];

uint8_t  autoState = 0;
uint16_t autoCount = 0;
bool     autoHeaderPrinted = false;
uint32_t autoFillStartMs = 0;     // 开始注水时刻（ms）
uint32_t autoDrainStartMs = 0;    // 开始抽水时刻（ms）
uint8_t  autoRound = 0;    // AUTO 已完成轮次（0=未开始，1=第1轮完成，2=第2轮完成），第3轮后归零结束
// (rate limiting removed)
const uint32_t AUTO_FILL_TIMEOUT_MS = 60000;  // 注水超时 60s
const uint32_t AUTO_DRAIN_TIMEOUT_MS = 60000;  // 抽水超时 60s

// ==================== 水位消抖 ====================
// 每 10ms 检测一次，连续 5 次（约 50ms）才认定有效，过滤 EMI 瞬态尖峰
const unsigned long WATER_DEBOUNCE_MS = 10;
const uint8_t WATER_TRIGGER_COUNT = 5;

unsigned long lastWaterCheckMs = 0;
uint8_t io6_low_count = 0;
uint8_t io7_high_count = 0;

// ==================== 泵动作 EMI 静默 ====================
// 泵启动/停止瞬间产生强 EMI，水位检测线会误跳变，期间屏蔽停泵判断
const uint32_t PUMP_EMI_GUARD_MS = 200;
uint32_t lastPumpActionMs = 0;

// ==================== 采集 ====================
void captureSamples() {
  const unsigned long period_us = 1000000.0 / samplingFrequency;
  unsigned long next = micros();
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    next += period_us;
    while (micros() < next);
  }
}

// ==================== 时域统计 ====================
void computeTimeStats(double &rms, double &stddev) {
  double sum = 0, sumSq = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum   += vReal[i];
    sumSq += vReal[i] * vReal[i];
  }
  double mean = sum / SAMPLES;
  rms = sqrt(sumSq / SAMPLES);
  // 标准差 = sqrt( E(x^2) - E(x)^2 )
  stddev = sqrt(sumSq / SAMPLES - mean * mean);
}

// ==================== 频段能量 ====================
void getBandEnergies(double bandsOut[], int bandCount) {
  static const int bandDef[][2] = {
    {0,78},{78,117},{117,156},{156,234},
    {234,312},{312,500},{500,1000},{1000,2000}
  };
  double binFreq = samplingFrequency / SAMPLES;
  for (int b = 0; b < bandCount; b++) {
    int s = max(1, (int)(bandDef[b][0] / binFreq));
    int e = min(SAMPLES/2 - 1, (int)(bandDef[b][1] / binFreq));
    double energy = 0;
    for (int i = s; i <= e; i++) energy += vReal[i] * vReal[i];
    bandsOut[b] = energy;
  }
}

// ==================== CSV 表头（只打一次） ====================
void printCSVHeader() {
  Serial.println(F("Pos,State,Round,PeakHz,RMS,StdDev,B0,B1,B2,B3,B4,B5,B6,B7,#ENDCOL"));
  headerPrinted = true;
}

// ==================== 触发中断服务 ====================
void IRAM_ATTR onTriggerFalling() {
  uint32_t now = micros();
  if (now - lastFallingUs < RISING_DEBOUNCE_US) return;  // 1ms防抖，过滤毛刺
  lastFallingUs = now;
  triggerDetected = true;
}


// ==================== 等待 IO3 上升沿（含超时） ====================
bool waitForTrigger() {
  unsigned long start = millis();
  while (true) {
    if (triggerDetected) {
      triggerDetected = false;
      delay(KNOCK_MS);
      return true;
    }
    if (millis() - start >= TRIGGER_TIMEOUT) {
      triggerBusy = false;  // 超时也解锁
      return false;
    }
    yield();
  }
}

// ==================== 一次敲击 + 分析（纯 CSV 输出） ====================
void doKnockAndAnalyze() {
  unsigned long tCycle = millis();

#ifdef MODE_ACTIVE
  digitalWrite(EMAG_PIN, HIGH);
  digitalWrite(TRIGGER_PIN, LOW);
  delay(KNOCK_MS);
  digitalWrite(EMAG_PIN, LOW);
  digitalWrite(TRIGGER_PIN, HIGH);
#endif

  captureSamples();

  // ---- 表头 ----
  if (!headerPrinted) printCSVHeader();

  // ---- FFT ----
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  double peakFreq = FFT.majorPeak();

  // ---- 时域统计 ----
  double rms, stddev;
  computeTimeStats(rms, stddev);

  // ---- 频段能量 ----
  const int bandCount = 8;
  double bands[bandCount];
  getBandEnergies(bands, bandCount);

  // ---- 一行 CSV ----
  Serial.print(positionId);
  Serial.print(F(","));
  Serial.print(stateNames[currentState]);
  Serial.print(F(","));
  Serial.print(0);  // Round=0（批次命令用，非AUTO）
  Serial.print(F(","));
  Serial.print(peakFreq, 2);
  Serial.print(F(","));
  Serial.print(rms, 1);
  Serial.print(F(","));
  Serial.print(stddev, 1);
  for (int b = 0; b < bandCount; b++) {
    Serial.print(F(","));
    Serial.print((int)bands[b]);
  }
  Serial.println(F(",#END"));               // 帧结束标记

#ifdef MODE_ACTIVE
  unsigned long elapsed = millis() - tCycle;
  if (elapsed < CYCLE_MS) delay(CYCLE_MS - elapsed);
#endif
}

// ==================== 全频谱（单次 CSV） ====================
void doFullSpectrum() {
#ifdef MODE_ACTIVE
  digitalWrite(EMAG_PIN, HIGH);
  delay(KNOCK_MS);
  digitalWrite(EMAG_PIN, LOW);
#endif

  captureSamples();

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double peakFreq = FFT.majorPeak();
  double rms, stddev;
  computeTimeStats(rms, stddev);

  // CSV 输出
  Serial.print(positionId);
  Serial.print(F(","));
  Serial.print(stateNames[currentState]);
  Serial.print(F(",SPECTRUM,Peak="));
  Serial.print(peakFreq, 2);
  Serial.print(F(",RMS="));
  Serial.print(rms, 1);
  Serial.print(F(",Std="));
  Serial.print(stddev, 1);
  Serial.println(F(",#END"));

  // 全频谱数据
  Serial.println(F("FreqHz,Amp"));
  for (int i = 1; i < SAMPLES / 2; i++) {
    Serial.print(i * samplingFrequency / SAMPLES, 1);
    Serial.print(F(","));
    Serial.println((int)vReal[i]);
  }
  Serial.println(F("#SPECTRUM_END"));
}

// ==================== 传播时间测试 ====================
void doTimeTest() {
  Serial.println(F("#TIME_TEST_START"));
  analogRead(CHANNEL); delay(10); analogRead(CHANNEL);

  const int baseN = 50;
  int baseData[baseN];
  unsigned long next = micros();
  for (int i = 0; i < baseN; i++) {
    baseData[i] = analogRead(CHANNEL); next += 50; while (micros() < next);
  }

  long baseSum = 0; int baseMin = 4096, baseMax = 0;
  for (int i = 0; i < baseN; i++) {
    baseSum += baseData[i];
    if (baseData[i] < baseMin) baseMin = baseData[i];
    if (baseData[i] > baseMax) baseMax = baseData[i];
  }
  int baseline = (int)(baseSum / baseN);
  int noisePkPk = baseMax - baseMin;
  int detectThresh = baseline + noisePkPk + 20;

  Serial.print(F("#BASELINE,"));
  Serial.print(baseline); Serial.print(F(","));
  Serial.print(noisePkPk); Serial.print(F(","));
  Serial.println(detectThresh);

  const int scanTotal = 500;
  int scanData[scanTotal];
  unsigned long scanTime[scanTotal];
  unsigned long tDrive;

  next = micros();
  for (int i = 0; i < 20; i++) {
    scanData[i] = analogRead(CHANNEL); scanTime[i] = 0;
    next += 10; while (micros() < next);
  }
  tDrive = micros();
#ifdef MODE_ACTIVE
  digitalWrite(EMAG_PIN, HIGH);
#endif
  for (int i = 20; i < scanTotal; i++) {
    scanData[i] = analogRead(CHANNEL); scanTime[i] = micros();
    next += 10; while (micros() < next);
  }
#ifdef MODE_ACTIVE
  digitalWrite(EMAG_PIN, LOW);
#endif

  unsigned long realEnd = scanTime[scanTotal - 1];
  if (realEnd == 0) realEnd = micros();
  Serial.print(F("#SCAN,"));
  Serial.print(realEnd - tDrive);
  Serial.print(F(" us, "));
  Serial.print(scanTotal - 20);
  Serial.println(F(" points"));

  Serial.println(F("t_us,ADC"));
  for (int i = 0; i < scanTotal; i++) {
    if (scanTime[i] > 0) Serial.print((int32_t)(scanTime[i] - tDrive));
    else Serial.print(F("pre"));
    Serial.print(F(",")); Serial.println(scanData[i]);
  }

  int arrivalIdx = -1;
  for (int i = 25; i < scanTotal - 2; i++) {
    if (scanData[i] > detectThresh && scanData[i+1] > detectThresh && scanData[i+2] > detectThresh) {
      arrivalIdx = i; break;
    }
  }
  Serial.print(F("#ARRIVAL,"));
  if (arrivalIdx >= 0) {
    Serial.print((int32_t)(scanTime[arrivalIdx] - tDrive));
    Serial.println(F(" us"));
  } else {
    Serial.println(F("none"));
  }
  Serial.println(F("#TIME_TEST_END"));
}

// ==================== 泵控制 ====================
// 进水：水槽 -> 水管  (IO4)
// 出水：水管 -> 水槽  (IO5)
void pumpStartIntake() {
  digitalWrite(PUMP_INTAKE_PIN, HIGH);
  pumpIntakeRunning = true;
  lastPumpActionMs = millis();
  Serial.println(F("#PUMP,INTAKE,ON"));
}

void pumpStopIntake() {
  digitalWrite(PUMP_INTAKE_PIN, LOW);
  pumpIntakeRunning = false;
  lastPumpActionMs = millis();
  Serial.println(F("#PUMP,INTAKE,OFF"));
}

void pumpStartDrain() {
  digitalWrite(PUMP_DRAIN_PIN, HIGH);
  pumpDrainRunning = true;
  lastPumpActionMs = millis();
  Serial.println(F("#PUMP,DRAIN,ON"));
}

void pumpStopDrain() {
  digitalWrite(PUMP_DRAIN_PIN, LOW);
  pumpDrainRunning = false;
  lastPumpActionMs = millis();
  Serial.println(F("#PUMP,DRAIN,OFF"));
}

void pumpStopAll() {
  pumpStopIntake();
  pumpStopDrain();
}

// ==================== 水位检测辅助 ====================
// 有水：IO6 LOW 或 IO7 LOW
// 空管：IO6 HIGH 且 IO7 HIGH
bool hasWater() {
  return (digitalRead(WATER_HIGH_PIN) == LOW || digitalRead(WATER_LOW_PIN) == LOW);
}

// ==================== AUTO 自动采集 ====================
void autoBegin() {
  triggerDetected = false;
  triggerBusy = false;
  passiveReady = false;  // 清命令发送前的残留触发/中断
  autoState = 1;
  autoCount = 0;
  autoRound = 0;
  autoHeaderPrinted = false;
  remainingCount = 0;    // AUTO 独立控制，不走普通批次

  Serial.println(F("#AUTO_START"));
#ifdef MODE_ACTIVE
  Serial.println(F("#AUTO,MODE,ACTIVE  [IO2驱动电磁铁敲击，IO3同步输出]"));
#else
  Serial.println(F("#AUTO,MODE,PASSIVE  [仅等外部IO3触发，无泵无水位]"));
#endif
  Serial.print(F("#AUTO_ROUND,0"));
  Serial.println();

#ifdef MODE_PASSIVE
  // ---- 被动模式：不控制泵，不检测水位，只等 IO3 触发 ----
  // 流程：等50笔(Round0) → 等50笔(Round1) → #AUTO_END
  currentState = EMPTY;
  autoState = 10;   // 状态10=被动等第1轮触发
  autoCount = 0;
  headerPrinted = false;
  Serial.println(F("#AUTO,PASSIVE_START  [被动模式，等IO3触发采集50笔空管数据]"));
  Serial.print(F("#PHASE,Empty,"));
  Serial.println(autoCompareCount);
  passiveReady = false; firstTriggerReceived = false;  // 重置触发状态
#else
  // ---- 主动模式：水管理 + 敲击采集 ----
  if (digitalRead(WATER_HIGH_PIN) == HIGH && digitalRead(WATER_LOW_PIN) == HIGH) {
    currentState = EMPTY;
    autoState = 3;
    autoCount = 0;
    headerPrinted = false;
    Serial.println(F("#AUTO,EMPTY_CONFIRMED  [IO6 HIGH && IO7 HIGH，管内已空]"));
    Serial.print(F("#PHASE,Empty,"));
    Serial.println(autoCompareCount);
  } else {
    Serial.println(F("#AUTO,DRAIN_START  [水位未空，开始抽水排空]"));
    pumpStartDrain();
    autoDrainStartMs = millis();
    io6_low_count = 0;
    io7_high_count = 0;
    autoState = 2;
  }
#endif
}

// ==================== 初始化 ====================
void setup() {
  pinMode(EMAG_PIN, OUTPUT);
  digitalWrite(EMAG_PIN, LOW);

  pinMode(PUMP_INTAKE_PIN, OUTPUT);
  pinMode(PUMP_DRAIN_PIN, OUTPUT);
  digitalWrite(PUMP_INTAKE_PIN, LOW);
  digitalWrite(PUMP_DRAIN_PIN, LOW);

  // 水位检测：浮空输入，LOW=有水，HIGH=无水
  pinMode(WATER_HIGH_PIN, INPUT);  // IO6
  pinMode(WATER_LOW_PIN,  INPUT);  // IO7

#ifdef MODE_PASSIVE
  pinMode(TRIGGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTriggerFalling, FALLING);
#else
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
#endif

  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("#SYSTEM,PipeAnalyzer v12.0"));
  Serial.println(F("#INFO,Format: CSV"));
  Serial.println(F("#INFO,State codes: 1=Empty  2=Full"));
  Serial.println(F("#INFO,Commands:"));
  Serial.println(F("#CMD,0=Stop  1=Empty  2=Full"));
  Serial.println(F("#CMD,3=TimeTest  4=Spectrum  5 n=SetPos"));
  Serial.println(F("#CMD,A=PumpIntake ON  B=PumpIntake OFF"));
  Serial.println(F("#CMD,C=PumpDrain ON   D=PumpDrain OFF"));
  Serial.println(F("#CMD,E=Pump ALL OFF"));
  Serial.println(F("#CMD,6=AUTO[排空->空管采集->注水->满水采集]"));
  Serial.print(F("#INFO,Current Pos="));
  Serial.print(positionId);
  Serial.print(F("  Count="));
  Serial.println(autoCompareCount);

  // 水位初始状态
  Serial.print(F("#WATER_SENSE,High="));
  Serial.print(digitalRead(WATER_HIGH_PIN) ? F("无水") : F("有水"));
  Serial.print(F("  Low="));
  Serial.println(digitalRead(WATER_LOW_PIN) ? F("无水") : F("有水"));

  // ---- ADC 自检：连续读取 10 次 A0 ----
  Serial.println(F("#ADC_TEST,10 reads on A0"));
  for (int i = 0; i < 10; i++) {
    Serial.print(analogRead(CHANNEL));
    if (i < 9) Serial.print(F(","));
    delay(100);
  }
  Serial.println();
}

// ==================== 主循环 ====================
void loop() {
  // ---- 水位消抖：每 10ms 检测一次，连续 2 次 LOW 才停泵 ----
  if (millis() - lastWaterCheckMs >= WATER_DEBOUNCE_MS) {
    lastWaterCheckMs = millis();

    if (digitalRead(WATER_HIGH_PIN) == LOW) {
      if (io6_low_count < WATER_TRIGGER_COUNT) io6_low_count++;
    } else {
      if (io6_low_count > 0) io6_low_count--;
    }

    if (digitalRead(WATER_LOW_PIN) == HIGH) {
      if (io7_high_count < WATER_TRIGGER_COUNT) io7_high_count++;
    } else {
      if (io7_high_count > 0) io7_high_count--;
    }

    // IO6 连续 5 次 LOW 且过 EMI 静默期 -> 停进水泵；IO7 连续 5 次 HIGH 且过静默期 -> 停出水泵
    if (io6_low_count >= WATER_TRIGGER_COUNT && pumpIntakeRunning && (millis() - lastPumpActionMs > PUMP_EMI_GUARD_MS)) {
      pumpStopIntake();
      Serial.println(F("#!IO6_LOW,INTAKE_PUMP_STOP  [高水位有水，停进水泵]"));
    }
    if (io7_high_count >= WATER_TRIGGER_COUNT && pumpDrainRunning && (millis() - lastPumpActionMs > PUMP_EMI_GUARD_MS)) {
      pumpStopDrain();
      Serial.println(F("#!IO7_HIGH,DRAIN_PUMP_STOP  [低水位无水，停出水泵]"));
    }
  }

  // ---- AUTO 状态机 ----
  if (autoState > 0) {
    switch (autoState) {

      // ---- 状态2：抽水排空中（消抖：IO6 HIGH 且 IO7 HIGH 持续 ~50ms 才认定空管）----
      case 2: {
        // 消抖：水位信号需连续 5 次（约 50ms）稳定才认定
        // IO6: io6_low_count==0 表示持续 HIGH（无水）
        // IO7: io7_high_count>=5 表示持续 HIGH（无水）
        bool io6StableHigh = (io6_low_count == 0);
        bool io7StableHigh = (io7_high_count >= WATER_TRIGGER_COUNT);

        if (io6StableHigh && io7StableHigh) {
          pumpStopDrain();
          currentState = EMPTY;
          autoState = 3;
          autoCount = 0;
          headerPrinted = false;
          Serial.println(F("#AUTO,DRAIN_DONE  [IO6+IO7 均 HIGH，管内已空]"));
          Serial.println(F("#AUTO,EMPTY_CONFIRMED"));
          Serial.print(F("#PHASE,Empty,"));
          Serial.println(autoCompareCount);
        } else if (millis() - autoDrainStartMs > AUTO_DRAIN_TIMEOUT_MS) {
          pumpStopDrain();
          Serial.println(F("#AUTO,ERR,DRAIN_TIMEOUT  [60s内未排空，请检查出水泵/管路]"));
          autoState = 0;
        }
        break;
      }

      // ---- 状态3：采集空管数据 50 笔 ----
      case 3: {
        if (autoCount < autoCompareCount) {
          unsigned long tCycle = millis();
          bool triggered = true;  // 标记是否成功触发（被动模式超时则 false）

#ifdef MODE_ACTIVE
          digitalWrite(EMAG_PIN, HIGH);
          digitalWrite(TRIGGER_PIN, HIGH);
          delay(KNOCK_MS);
          digitalWrite(EMAG_PIN, LOW);
          digitalWrite(TRIGGER_PIN, LOW);
#else
          if (!waitForTrigger()) {
            Serial.println(F("#AUTO,PASSIVE_TIMEOUT  [IO3 触发超时，跳过本笔]"));
            triggered = false;
          }
#endif

          if (triggered) {
            captureSamples();

            if (!headerPrinted) printCSVHeader();

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
            double peakFreq = FFT.majorPeak();
            double rms, stddev;
            computeTimeStats(rms, stddev);
            getBandEnergies(g_bands, 8);

            Serial.print(positionId);
            Serial.print(F(",Empty,"));
            Serial.print(autoRound);  // 轮次标识（0=第一轮, 1=第二轮）
            Serial.print(F(","));
            Serial.print(peakFreq, 2);
            Serial.print(F(","));
            Serial.print(rms, 1);
            Serial.print(F(","));
            Serial.print(stddev, 1);
            for (int b = 0; b < 8; b++) {
              Serial.print(F(","));
              Serial.print((int)g_bands[b]);
            }
            Serial.println(F(",#END"));

#ifdef MODE_ACTIVE
            unsigned long elapsed = millis() - tCycle;
            if (elapsed < CYCLE_MS) delay(CYCLE_MS - elapsed);
#endif
          }
          autoCount++;  // 超时也计数，防止死循环
        } else {
          digitalWrite(EMAG_PIN, LOW);
          Serial.println(F("#AUTO,EMPTY_DONE"));
          Serial.print(F("#AUTO_ROUND,"));
          Serial.print(autoRound);
          Serial.println();

          if (autoRound >= 2) {
            // 两轮已完成，结束 AUTO
            pumpStopAll();
            Serial.println(F("#AUTO,FULL_DONE"));
            Serial.println(F("#AUTO_END"));
            autoState = 0;
            autoRound = 0;
          } else {
            // 第 1 轮完成 → 抽水排空，准备第 2 轮
            Serial.println(F("#AUTO,FILL_START  [进水泵启动，注水直到 IO6 LOW]"));
            pumpStartIntake();
            autoFillStartMs = millis();
            autoState = 4;
          }
        }
        break;
      }

      // ---- 状态4：等待注水达标（消抖：IO6 LOW 持续 ~50ms 才认定有水）----
      case 4: {
        // 消抖：IO6 LOW 需连续 5 次（约 50ms）稳定
        bool io6StableLow = (io6_low_count >= WATER_TRIGGER_COUNT);

        if (io6StableLow) {
          pumpStopIntake();
          Serial.println(F("#AUTO,FILL_DONE  [IO6 持续 LOW，有水，停进水泵]"));
          delay(500);
          currentState = FULL;
          autoState = 5;
          autoCount = 0;
          headerPrinted = false;
          Serial.print(F("#PHASE,Full,"));
          Serial.println(autoCompareCount);
          Serial.println(F("#AUTO,FULL_CONFIRMED"));
        } else if (millis() - autoFillStartMs > AUTO_FILL_TIMEOUT_MS) {
          pumpStopIntake();
          Serial.println(F("#AUTO,ERR,FILL_TIMEOUT  [60s内IO6未稳定 LOW，请检查管路]"));
          autoState = 0;
        }
        break;
      }

      // ---- 状态5：采集满水数据 50 笔 ----
      case 5: {
        if (autoCount < autoCompareCount) {
          unsigned long tCycle = millis();
          bool triggered = true;

#ifdef MODE_ACTIVE
          digitalWrite(EMAG_PIN, HIGH);
          digitalWrite(TRIGGER_PIN, HIGH);
          delay(KNOCK_MS);
          digitalWrite(EMAG_PIN, LOW);
          digitalWrite(TRIGGER_PIN, LOW);
#else
          if (!waitForTrigger()) {
            Serial.println(F("#AUTO,PASSIVE_TIMEOUT  [IO3 触发超时，跳过本笔]"));
            triggered = false;
          }
#endif

          if (triggered) {
            captureSamples();

            if (!headerPrinted) printCSVHeader();

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
            double peakFreq = FFT.majorPeak();
            double rms, stddev;
            computeTimeStats(rms, stddev);
            getBandEnergies(g_bands, 8);

            Serial.print(positionId);
            Serial.print(F(",Full,"));
            Serial.print(autoRound);  // 轮次标识（0=第一轮, 1=第二轮）
            Serial.print(F(","));
            Serial.print(peakFreq, 2);
            Serial.print(F(","));
            Serial.print(rms, 1);
            Serial.print(F(","));
            Serial.print(stddev, 1);
            for (int b = 0; b < 8; b++) {
              Serial.print(F(","));
              Serial.print((int)g_bands[b]);
            }
            Serial.println(F(",#END"));

#ifdef MODE_ACTIVE
            unsigned long elapsed = millis() - tCycle;
            if (elapsed < CYCLE_MS) delay(CYCLE_MS - elapsed);
#endif
          }
          autoCount++;  // 超时也计数，防止死循环
        } else {
          // 满水采集完成，本轮结束
          digitalWrite(EMAG_PIN, LOW);
#ifndef MODE_PASSIVE
          pumpStopAll();
#endif
          autoRound++;
          Serial.print(F("#AUTO,ROUND_DONE  [第 "));
          Serial.print(autoRound);
          Serial.println(F(" 轮完成]"));
          Serial.print(F("#AUTO_ROUND,"));
          Serial.print(autoRound);
          Serial.println();

          if (autoRound >= 2) {
            // 两轮全部完成，退出 AUTO
            Serial.println(F("#AUTO_END"));
            autoState = 0;
            autoRound = 0;
          } else {
            // 第 2 轮：抽水排空，重新开始
            Serial.println(F("#AUTO,DRAIN_START  [第2轮，开始抽水排空]"));
            pumpStartDrain();
            autoDrainStartMs = millis();
            io6_low_count = 0;
            io7_high_count = 0;
            autoState = 2;
          }
        }
        break;
      }

      // ========== 被动 AUTO 状态（MODE_PASSIVE 专用）==========
      // 状态10：等第1轮50笔IO3触发（空管）
      // 状态11：等第2轮50笔IO3触发（满管）
      // 状态12：结束
#ifdef MODE_PASSIVE
      case 10:
      case 11: {
        if (autoCount < autoCompareCount) {
          // ---- 被动AUTO：先静默等待第1个IO3边沿（主动设备开始敲击），过滤抽/注水期任何误触发 ----
          if (!firstTriggerReceived) {
            if (triggerDetected && !triggerBusy) {
              triggerDetected = false;
              firstTriggerReceived = true;  // 第1个有效敲击，后续正常计数
              passiveReady = true;
            }
          } else if (triggerDetected && !triggerBusy && passiveReady) {
            triggerDetected = false;
            captureSamples();
            triggerBusy = false;  // 捕获完成后解锁，允许下一次触发

            if (!headerPrinted) printCSVHeader();

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
            double peakFreq = FFT.majorPeak();
            double rms, stddev;
            computeTimeStats(rms, stddev);
            getBandEnergies(g_bands, 8);

            Serial.print(positionId);
            Serial.print(F(","));
            Serial.print(stateNames[currentState]);
            Serial.print(F(","));
            Serial.print(autoRound);  // 0=第1轮, 1=第2轮
            Serial.print(F(","));
            Serial.print(peakFreq, 2);
            Serial.print(F(","));
            Serial.print(rms, 1);
            Serial.print(F(","));
            Serial.print(stddev, 1);
            for (int b = 0; b < 8; b++) {
              Serial.print(F(","));
              Serial.print((int)g_bands[b]);
            }
            Serial.println(F(",#END"));

            autoCount++;
          }
          // else: 未触发或忙，静默忽略
        } else {
          // 50笔完成
          if (autoState == 10) {
            Serial.println(F("#AUTO,PASSIVE_EMPTY_DONE"));
            Serial.print(F("#AUTO_ROUND,"));
            Serial.print(autoRound);
            Serial.println();
            // 下一轮：满管，状态11
            currentState = FULL;
            autoState = 11;
            autoCount = 0;
            autoRound = 1;  // 第2轮（Round=1）
            headerPrinted = false;
            firstTriggerReceived = false;  // 重置，静默等待第1个IO3边沿
            passiveReady = false;
            Serial.print(F("#PHASE,Full,"));
            Serial.println(autoCompareCount);
          } else {
            // 状态11：第2轮完成，结束
            Serial.println(F("#AUTO,PASSIVE_FULL_DONE"));
            Serial.print(F("#AUTO_ROUND,"));
            Serial.print(autoRound);
            Serial.println();
            Serial.println(F("#AUTO_END"));
            autoState = 0;
            autoRound = 0;
          }
        }
        break;
      }
#endif
    }
  }

  // ---- 命令处理：所有命令（含 AUTO 中的 0）均独立于 autoState 检查 ----
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // ---- 命令 0：最优先，任意状态下均可停止 AUTO / 批次 / 敲击 ----
    if (cmd == '0') {
#ifndef MODE_PASSIVE
      pumpStopAll();
#endif
      digitalWrite(EMAG_PIN, LOW);
      if (autoState > 0) {
        autoState = 0;
        autoRound = 0;
        Serial.println(F("#AUTO,STOPPED  [已停止 AUTO]"));
      } else {
        remainingCount = 0;
        Serial.println(F("#STOP"));
      }
    }

    // ---- AUTO 执行中只响应 0，其他命令跳过 ----
    if (autoState > 0) return;

    switch (cmd) {
      case '1':
        triggerDetected = false;
        currentState = EMPTY;
        remainingCount = autoCompareCount;
        headerPrinted = false;
        Serial.print(F("#BATCH,Empty,"));
        Serial.println(autoCompareCount);

        break;
      case '2':
        triggerDetected = false;
        currentState = FULL;
        remainingCount = autoCompareCount;
        headerPrinted = false;
        Serial.print(F("#BATCH,Full,"));
        Serial.println(autoCompareCount);
        break;
      case '3':
        doTimeTest();
        break;
      case '4':
        doFullSpectrum();
        break;
      case '5': {
        delay(50);
        if (Serial.available() > 0) {
          char p = Serial.read();
          int np = p - '0';
          if (np >= 0 && np <= 9) positionId = np;
        }
        Serial.print(F("#POS,"));
        Serial.println(positionId);

        break;
      }
      // ---- 水泵控制命令 ----
      case 'A':
        pumpStartIntake();
        break;
      case 'B':
        pumpStopIntake();
        break;
      case 'C':
        pumpStartDrain();
        break;
      case 'D':
        pumpStopDrain();
        break;
      case 'E':
        pumpStopAll();
        break;
      case '6':
        autoBegin();
        break;
    }
  }

  // ---- 普通批次敲击（仅在非 AUTO 状态下执行）----
  if (autoState == 0 && remainingCount > 0) {
#ifdef MODE_PASSIVE
    if (waitForTrigger()) {
      doKnockAndAnalyze();
      remainingCount--;
      if (remainingCount == 0) {
        Serial.println(F("#DONE,请换位置或注水后发送指令 1/2"));
      }
    } else {
      remainingCount = 0;
      Serial.println(F("#TIMEOUT,触发超时，批次已停止"));
    }
#else
    doKnockAndAnalyze();
    remainingCount--;
    if (remainingCount == 0) {
      digitalWrite(EMAG_PIN, LOW);
      Serial.println(F("#DONE,请换位置或注水后发送指令 1/2"));
    }
#endif
  }

  // ---- 被动模式测试：检测到IO3上升沿则打印调试信息 ----
  if (lastFallingUs > 0) {
    uint32_t t = micros() - lastFallingUs;
    Serial.print(F("#IO3_FALLING,us="));
    Serial.print(t);
    Serial.println();
    lastFallingUs = 0;
  }

  // 给 ESP32 loopTask 看门狗喂狗，防止 Core 1 WDT 超时重启
  yield();
}
