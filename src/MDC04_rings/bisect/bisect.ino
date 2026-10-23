/*
  MDC04 v31 - CH1 Monitor + OFAT 参数扫描测试模式
  - 正常模式：CH1 每秒轮询（铜片干/湿）
  - 测试模式（串口 '6' 启动）：单因素轮换扫描 Cr / 重复性 / Co
    每个参数在 EMPTY 和 FULL 两段各扫多个值，每值采 50 笔
    用进/出水泵 + 水位传感器自动判断满/空（仿 V2）

  引脚：
    DQ      GPIO2  (单总线)
    进水泵   GPIO4  (HIGH=开, 水槽->水管)
    出水泵   GPIO5  (HIGH=开, 水管->水槽)
    高水位   GPIO6  (LOW=有水)
    低水位   GPIO7  (LOW=有水)
*/

#define DQ_PIN          2
#define PUMP_INTAKE_PIN 4
#define PUMP_DRAIN_PIN  5
#define WATER_HIGH_PIN  6
#define WATER_LOW_PIN   7

// --- 默认参数 ------------------------------------------------
#define COS_CH1_DEFAULT 0x09   // → CH1 (b6=0x78)
#define CFB_DEFAULT      0x23
#define REP_DEFAULT      0x01   // 重复性候选(MED)，字节待实测

// --- 扫描候选值 ----------------------------------------------
uint8_t crVals[]  = {0x10, 0x18, 0x23, 0x30, 0x40};   // Cr (Cfb) 5值
uint8_t repVals[] = {0x00, 0x01, 0x02};                // 重复性 (Config) 3值
uint8_t coVals[]  = {0x00, 0x01, 0x02, 0x04, 0x09};   // Co (Cos) 5值，会动通道
const int CR_N  = 5;
const int REP_N = 3;
const int CO_N  = 5;
const char* paramNames[] = {"Cr", "Rep", "Co"};

// --- 采样/超时 ------------------------------------------------
const int      SAMPLES_PER_BATCH = 50;
const uint32_t FILL_TIMEOUT_MS   = 60000;
const uint32_t DRAIN_TIMEOUT_MS  = 90000;
const uint32_t SAMPLE_GAP_MS     = 20;   // 每笔间隔

// --- 正常范围 -------------------------------------------------
const uint16_t RAW_MIN_VALID = 600;

// --- 单总线时序 ----------------------------------------------
inline void gpio_low()  { digitalWrite(DQ_PIN, LOW); }
inline void gpio_high() { digitalWrite(DQ_PIN, HIGH); }
inline int  gpio_read() { return digitalRead(DQ_PIN); }

void gpio_init() {
  pinMode(DQ_PIN, OUTPUT_OPEN_DRAIN);
  gpio_high();
}

bool ow_reset() {
  cli();
  gpio_low(); delayMicroseconds(500);
  gpio_high();
  int found = -1;
  for (int t = 0; t < 300; t++) {
    if (gpio_read() == 0) { found = t; break; }
    delayMicroseconds(1);
  }
  sei();
  delayMicroseconds(found < 0 ? 0 : 410 - found);
  return found >= 0;
}

void ow_write_byte(uint8_t byte) {
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    gpio_low();
    if (byte & 0x01) { delayMicroseconds(6);  gpio_high(); delayMicroseconds(64); }
    else              { delayMicroseconds(65); gpio_high(); delayMicroseconds(5);  }
    byte >>= 1;
  }
  sei();
}

uint8_t ow_read_byte() {
  uint8_t byte = 0;
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    gpio_low(); gpio_high();
    delayMicroseconds(13);
    if (gpio_read()) byte |= 0x80;
    delayMicroseconds(50);
    byte >>= 1;
  }
  sei();
  return byte;
}

bool ow_poll_done() {
  uint32_t deadline = millis() + 500;
  while (millis() < deadline) {
    cli();
    gpio_low(); delayMicroseconds(1); gpio_high(); delayMicroseconds(9);
    uint8_t bit = gpio_read();
    sei();
    if (bit == 0) return true;
    delayMicroseconds(10);
  }
  return false;
}

// --- 写参数（Cos/Cfb/Config 各一次 WriteScratchpad）----------
void write_scratch_byte(uint8_t addr, uint8_t val) {
  if (!ow_reset()) return;
  ow_write_byte(0xCC);
  ow_write_byte(0x4E);
  ow_write_byte(addr);
  ow_write_byte(val);
  delayMicroseconds(25);
  gpio_high();
  delayMicroseconds(10);
}

void apply_params(uint8_t cos, uint8_t cfb, uint8_t config) {
  write_scratch_byte(0x1D, cos);
  delay(2);
  write_scratch_byte(0x22, cfb);
  delay(2);
  write_scratch_byte(0x06, config);
}

// --- 触发转换 + 读 Scratchpad（带转换时间计时）----------------
uint32_t g_convMs = 0;
bool convert_and_read(uint16_t& c_raw, uint8_t& b6) {
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);
  ow_write_byte(0x66);          // ConvertC
  uint32_t t0 = millis();
  if (!ow_poll_done()) return false;
  g_convMs = millis() - t0;
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);
  ow_write_byte(0xBE);          // ReadScratchpad
  uint8_t d[9];
  for (int i = 0; i < 9; i++) d[i] = ow_read_byte();
  gpio_low(); delayMicroseconds(2); gpio_high(); delayMicroseconds(500);
  c_raw = ((uint16_t)d[3] << 8) | d[2];
  b6 = d[6];
  if (c_raw < RAW_MIN_VALID) return false;
  return true;
}

bool measure_with_params(uint8_t cos, uint8_t cfb, uint8_t config,
                         uint16_t& c_raw, uint8_t& b6, double& c_pf) {
  apply_params(cos, cfb, config);
  delay(3);
  for (int retry = 0; retry < 3; retry++) {
    if (convert_and_read(c_raw, b6)) {
      c_pf = 15.5 + c_raw / 32768.0 * 31.0;
      return true;
    }
    delay(20);
  }
  return false;
}

// --- 泵 / 水位 ------------------------------------------------
void pumpIntakeOn()  { digitalWrite(PUMP_INTAKE_PIN, HIGH); }
void pumpIntakeOff() { digitalWrite(PUMP_INTAKE_PIN, LOW); }
void pumpDrainOn()   { digitalWrite(PUMP_DRAIN_PIN,  HIGH); }
void pumpDrainOff()  { digitalWrite(PUMP_DRAIN_PIN,  LOW); }
void pumpAllOff()    { pumpIntakeOff(); pumpDrainOff(); }

const unsigned long WATER_DEBOUNCE_MS = 10;
const uint8_t WATER_TRIGGER_COUNT = 5;
unsigned long lastWaterCheckMs = 0;
uint8_t io6_low_count = 0;   // IO6==LOW(有水) 计数
uint8_t io7_high_count = 0;  // IO7==HIGH(无水) 计数

void waterDebounce() {
  if (millis() - lastWaterCheckMs < WATER_DEBOUNCE_MS) return;
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
}
bool waterEmptyStable() { return (io6_low_count == 0 && io7_high_count >= WATER_TRIGGER_COUNT); }
bool waterFullStable()  { return (io6_low_count >= WATER_TRIGGER_COUNT && io7_high_count == 0); }

// --- 测试模式状态机 ------------------------------------------
enum TestState {
  TEST_IDLE = 0, TEST_ENSURE_EMPTY, TEST_COLLECT,
  TEST_FILL, TEST_DRAIN_RESTORE, TEST_DONE
};

struct BatchResult {
  uint8_t stage;     // 0=EMPTY,1=FULL
  uint8_t param;     // 0=Cr,1=Rep,2=Co
  uint8_t valIdx;
  double  mean;
  double  stddev;
  uint8_t b6;
};

bool    testActive = false;
uint8_t testState  = TEST_IDLE;
uint8_t testStage  = 0;
uint8_t testParam  = 0;
uint8_t testValIdx = 0;
uint8_t testSampleIdx = 0;
uint32_t testPhaseStartMs = 0;
double  testSum = 0, testSumSq = 0;
uint8_t testCurCos = COS_CH1_DEFAULT, testCurCfb = CFB_DEFAULT, testCurConfig = REP_DEFAULT;
uint8_t testCurB6 = 0;
BatchResult results[26];
int resultCount = 0;
int positionId = 1;

uint8_t getParamVal(uint8_t param, uint8_t valIdx) {
  if (param == 0) return crVals[valIdx];
  if (param == 1) return repVals[valIdx];
  return coVals[valIdx];
}
void getParamValues(uint8_t param, uint8_t& cos, uint8_t& cfb, uint8_t& config, uint8_t valIdx) {
  cos = COS_CH1_DEFAULT; cfb = CFB_DEFAULT; config = REP_DEFAULT;
  if (param == 0)      cfb   = crVals[valIdx];
  else if (param == 1) config = repVals[valIdx];
  else                 cos   = coVals[valIdx];
}

struct Foo { int a; };
void bar(Foo* p) {}

void setup() {}
void loop() {}
