/*
  MDC04 v31 - CH1 Monitor + OFAT 参数扫描测试模式
  - 正常模式:CH1 每秒轮询(铜片干/湿)
  - 测试模式(串口 '6' 启动):扫描 Co(Cos) 参数
    已固定 Cr=0x18、Rep=0x1(最优候选),不再扫描这两个参数
    Co 在 EMPTY 和 FULL 两段各扫 5 个值,每值采 50 笔
    用进/出水泵 + 水位传感器自动判断满/空(仿 V2)

  引脚:
    DQ      GPIO2  (单总线)
    进水泵   GPIO4  (HIGH=开, 水槽->水管)
    出水泵   GPIO5  (HIGH=开, 水管->水槽)
    高水位   GPIO6  (LOW=有水)
    低水位   GPIO7  (LOW=有水)
*/

#include "LoRaWan_APP.h"   // Heltec LoRaWAN 库(移植自 LoRaWan.ino)

#define DQ_PIN          2
#define PUMP_INTAKE_PIN 4
#define PUMP_DRAIN_PIN  5
#define WATER_HIGH_PIN  6
#define WATER_LOW_PIN   7

// --- 默认参数 ------------------------------------------------
#define COS_CH1_DEFAULT 0x09   // → CH1 (b6=0x78)
#define CFB_DEFAULT      0x18   // Cr 已固定 0x18(OFAT 最优候选)
#define REP_DEFAULT      0x01   // 重复性固定 MED

// --- 扫描候选值 ----------------------------------------------
// 已固定: Cr=0x18, Rep=0x1 -- 后续不再扫描这两个参数,只扫 Co(Cos)
uint8_t coVals[]  = {0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x09};   // Co (Cos) 扫描值
const int CO_N  = 8;
const char* paramNames[] = {"Co"};

// --- 采样/超时 ------------------------------------------------
const int      SAMPLES_PER_BATCH = 1000;
const uint32_t FILL_TIMEOUT_MS   = 60000;
const uint32_t DRAIN_TIMEOUT_MS  = 90000;
const uint32_t SAMPLE_GAP_MS     = 200;  // 每笔间隔

// --- 连续监测模式 ----------------------------------------------
const uint8_t  COS_CONT       = 0x01;  // 持续监测固定 Co=0x01（用户指定）

// === 通道检测开关（编译期宏，可单独关闭 CH1 / CH4 检测）========
// 1=启用检测, 0=禁用检测。禁用后对应通道不参加 ConvertC、不打印数据行。
// 典型组合：
//   都开 (1,1)：同时监测 CH1 + CH4（默认）
//   仅 CH1 (1,0)：只监测通道1
//   仅 CH4 (0,1)：只监测通道4
//   都关 (0,0)：无任何通道被转换（启动时打印 #WARN，数据全 ERR）
#define ENABLE_CH1_DETECT  0   // 通道1（电极 C1P/C1N）检测开关
#define ENABLE_CH4_DETECT  1   // 通道X（电极 CXP/CXN）检测开关；实际通道由 CHX_USE_CH1 决定

// === 通道X 实际通道选择（1=读 CH1，0=读 CH4）============
// 仅切换 CHX_USE_CH1 即可在 CH1/CH4 间切换；当前只接了 CH1 电极，用 CH1 代替 CH4 检测。
// 注意：硬件电极必须接对应的 CXP/CXN，否则读到空通道（raw≈0 被 RAW_MIN_VALID 过滤）。
//   CHX_USE_CH1=1：读 CH1（ReadScratchpad 0xBE 取 Cap1 字节对 + CH_sel bit0=0x01）
//   CHX_USE_CH1=0：读 CH4（ReadC2-4 0xDC 取 Cap4 字节对 + CH_sel bit3=0x08，原行为）
#define CHX_USE_CH1        1   // 1=把通道X当作 CH1 读取（用 CH1 代替 CH4）；0=CH4(原行为)

// Ch_sel(0x1C) 低4位各自独立选择转换的通道：
//   bit0=CH1  bit1=CH2  bit2=CH3  bit3=CH4
//   bit6(0x40)=保留位（默认置位，低功耗模式），须保持
// 由上面的开关宏动态组合，不再写死全部四通道。
#define CHSEL_BASE   0x40   // 保留位，必须保持
#define CHSEL_CONT  (CHSEL_BASE \
                     | (ENABLE_CH1_DETECT ? 0x01 : 0x00) \
                     | (ENABLE_CH4_DETECT ? (CHX_USE_CH1 ? 0x01 : 0x08) : 0x00))
const uint32_t CONT_SAMPLE_MS = 200;   // 实时连续采样周期(ms)
const uint8_t  CONT_AVG_N     = 1;     // 每周期平均笔数（对齐 '6' 的批量均值，抑制单笔双模态跳变）

// --- 正常范围 -------------------------------------------------
const uint16_t RAW_MIN_VALID = 100;  // 仅剔除近零/短路读数;当前电极干态 raw≈600,低于此才视为异常

// --- 应用监测模式（基于测试17 跨液面基线, 每10ms采样, ΔC连续>2pF报警）---
// 测试17 结果: CH1 干态基线≈16.77pF, 满管(L2)≈20.66pF, ΔC≈3.9pF; 取 2.0pF 作为高水位告警阈值。
const uint32_t APP_SAMPLE_MS       = 20;    // 每 10ms 采样一次
const double   APP_OVER_DELTA_PF   = 2.0;   // ΔC 超过 2pF 视为触发（对应水位明显升高）
const uint8_t  APP_OVER_COUNT_TRIG = 10;    // 连续 10 次（≈100ms）超阈值才确认, 抗抖动
const uint32_t APP_DEBUG_MS        = 2000;  // 触发后每 2s 输出一次调试信息

// === CH4 流态检测（基于测试21：静水≈20.73pF / 动水≈20.17pF，流水时电容更低；Δ≈0.56pF，SNR≈7）===
// 判别依据：流水时电极周围介质被带走，等效电容比静水低；用"当前值相对静水基线的偏移"判定。
// 基线只在静止期缓慢跟踪，流水期冻结，避免把流水低值拉进基线。
#define EVT_CH1_ALERT        1   // CH1 高水位触发
#define EVT_CH1_RECOVER      0   // CH1 恢复正常
#define EVT_CH4_FLOW_START   3   // CH4 检测到流水（开始）
#define EVT_CH4_FLOW_STOP    4   // CH4 流水停止（恢复静止）

const double   FLOW_DELTA_PF        = 0.24;  // 低于静水基线超过 0.28pF 判为流水（Δ=0.56，取中点，留足余量）
const double   FLOW_RECOVER_PF      = 0.12;  // 回到距基线 0.12pF 内判为静止（滞回防抖）
const uint8_t  FLOW_OVER_COUNT_TRIG = 5;     // 连续 5 次(~100ms@20ms)确认流水
const uint8_t  FLOW_RECOVER_TRIG    = 5;     // 连续 5 次确认静止
const double   FLOW_BASELINE_ALPHA  = 0.02;  // 静水基线慢跟踪系数（静止期更新）

bool     flowActive       = false;  // 当前是否判定为流水
double   baseline_ch4     = 0.0;    // 静水基线（首次有效读数播种，静止期缓慢跟踪）
uint32_t flowOverCount    = 0;      // 连续判定“偏低(流水)”计数
uint32_t flowRecoverCount = 0;      // 连续判定“回到基线(静止)”计数
uint32_t flowStartMs      = 0;      // 最近一次进入流水态的时间
uint32_t lastFlowDbgMs    = 0;

uint8_t  appOverCount = 0;
bool     appAlert     = false;
uint32_t lastAppMs    = 0;
uint32_t lastAppDbgMs = 0;
double   baseline_ch1 = 0.0;   // 干态基线(启动校准得到); 应用监测以此算 ΔC

// === LoRaWAN (移植自 LoRaWan.ino, Heltec LoRaWan_APP.h) ========
// OTAA 参数(示例值, 部署时替换为你的节点密钥)
uint8_t devEui[] = {0xBB,0xF0,0x64,0xF5,0x64,0x88,0x41,0x17};
uint8_t appEui[] = {0xDD,0x9E,0xDA,0x52,0xC6,0x4D,0x4B,0xE3};
uint8_t appKey[] = {0x20,0x1D,0xC8,0xF6,0xF2,0x4D,0xDD,0x88,0x52,0x8C,0x8D,0x9F,0xD6,0xC9,0x08,0x74};

/* ABP para (本工程用 OTAA, 以下仅为兼容 Heltec 库符号保留, 不参与编译逻辑) */
uint8_t nwkSKey[] = { 0x15,0xb1,0xd0,0xef,0xa4,0x63,0xdf,0xbe,0x3d,0x11,0x18,0x1e,0x1e,0xc7,0xda,0x85 };
uint8_t appSKey[] = { 0xd7,0x2c,0x78,0x75,0x8c,0xdc,0xca,0xbf,0x55,0xee,0x4a,0x77,0x8d,0x16,0xef,0x67 };
uint32_t devAddr  = (uint32_t)0x007e6ae1;

uint16_t userChannelsMask[6] = {0x00FF,0x0000,0x0000,0x0000,0x00FF,0x00FF};
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t  loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 1000;     // 事件驱动: 实际发送由 uploadEvent() 触发
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
bool ready_flag = false;
int g_lorawanRssi = 0;

// 事件上传缓冲(时间+事件+持续时长); 仅在状态跳变时上传一次
uint8_t  g_event = 0;           // 1=触发(高水位), 0=恢复正常
uint32_t g_eventTime = 0;       // 触发时的 uptime(ms); 0=尚未发生过事件
uint32_t g_eventDurationS = 0;  // 本次事件持续时长(秒); 触发时=0, 恢复时=alertStartMs至今除以1000
uint32_t alertStartMs = 0;      // 最近一次 ALERT 开始时间(ms), 用于计算恢复时长

// 恢复阈值(滞回): 连续10次 ΔC < 1.9pF 视为恢复
const double APP_RECOVER_DELTA_PF = 1.9;

uint8_t appRecoverCount = 0;

void downLinkAckHandleResp(McpsIndication_t* mcpsIndication) {
  ready_flag = true;
  g_lorawanRssi = mcpsIndication->Rssi;
}

// 上传负载: 时间(4字节, uptime ms, 小端) + 事件(1字节: 1=触发, 0=恢复)
// 上传负载: 9字节 = 时间4字节(uptime ms) + 事件1字节(1=触发,0=恢复) + 持续时长4字节(ms)
static void prepareTxFrame(uint8_t port) {
  appDataSize = 9;
  uint32_t t = g_eventTime;
  uint32_t d = g_eventDurationS;
  appData[0] = (t >> 24) & 0xFF;
  appData[1] = (t >> 16) & 0xFF;
  appData[2] = (t >> 8) & 0xFF;
  appData[3] = t & 0xFF;
  appData[4] = g_event;
  appData[5] = (d >> 24) & 0xFF;
  appData[6] = (d >> 16) & 0xFF;
  appData[7] = (d >> 8) & 0xFF;
  appData[8] = d & 0xFF;
}

// 是否已加入网络: Heltec 库在 INIT/JOIN 后会离开这两个状态
bool loraJoined() {
  return (deviceState != DEVICE_STATE_INIT) && (deviceState != DEVICE_STATE_JOIN);
}

// 触发一次事件上传(仅在入网后); 状态跳变时各调用一次
// dur 为本次事件的持续时长(ms); 触发时传 0, 恢复时传 alertStartMs 至今
void uploadEvent(uint8_t ev, uint32_t dur) {
  if (!loraJoined()) return;
  g_event = ev;
  g_eventTime = millis();
  g_eventDurationS = dur / 1000;  // 毫秒转秒(整数)
  deviceState = DEVICE_STATE_CYCLE;   // 经 CYCLE->定时器->SEND 机制真正发送(见 loraTask)
}

// LoRa 状态机(移植自 LoRaWan.ino loop 的 switch)
void loraTask() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      break;
    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;
    case DEVICE_STATE_SEND:
      if (g_eventTime == 0) { deviceState = DEVICE_STATE_SLEEP; break; } // 尚无事件, 跳过首包
      prepareTxFrame(appPort);
      LoRaWAN.send();
      g_eventTime = 0;   // 已发送, 防止 SEND 被再次触发时重发旧事件
      deviceState = DEVICE_STATE_SLEEP;
      break;
    case DEVICE_STATE_CYCLE:
      // 事件驱动: 启动 TxNextPacketTimer（dutyCycle ms 后触发 OnTxNextPacketTimerEvent,
      // 该回调置 nextTx=true 并把 deviceState 置回 SEND），随后回 SLEEP 等待回调。
      // 不循环重发: SEND 发送完即回 SLEEP, 直到下一次 uploadEvent() 再进入 CYCLE。
      LoRaWAN.cycle(appTxDutyCycle);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      break;
    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}

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

// --- 写参数(Cos/Cfb/Config 各一次 WriteScratchpad)----------
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

// --- 通道选择 / 通道4读取 -------------------------------------
// 写 Ch_sel 寄存器(单总线指令 WriteCh_sel = 0xAA,主机给 1 字节)
void write_ch_sel(uint8_t v) {
  if (!ow_reset()) return;
  ow_write_byte(0xCC);   // Skip ROM
  ow_write_byte(0xAA);   // WriteCh_sel
  ow_write_byte(v);
  gpio_high();
  delayMicroseconds(10);
}

// 读 Ch_sel 寄存器(ReadCh_sel = 0x8A,芯片回 1 字节)
uint8_t read_ch_sel() {
  if (!ow_reset()) return 0xFF;
  ow_write_byte(0xCC);
  ow_write_byte(0x8A);   // ReadCh_sel
  return ow_read_byte();
}

// 通道X 结果读取：CHX_USE_CH1=1 读 CH1（ReadScratchpad 0xBE，电极 C1P/C1N），=0 读 CH4（ReadC2-4 0xDC，原行为）
bool read_ch4_raw(uint16_t& c4) {
#if CHX_USE_CH1
  // 用 CH1 代替 CH4：读 ReadScratchpad 的 Cap1（需 CHSEL 已选 CH1 且已触发 ConvertC）
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);   // Skip ROM
  ow_write_byte(0xBE);   // ReadScratchpad
  uint8_t d[9];
  for (int i = 0; i < 9; i++) d[i] = ow_read_byte();
  gpio_low(); delayMicroseconds(2); gpio_high(); delayMicroseconds(500);
  c4 = ((uint16_t)d[3] << 8) | d[2];   // Cap1_msb<<8 | Cap1_lsb  (CHX_USE_CH1=1)
  return true;
#else
  // 原行为：读 CH2/3/4 结果寄存器(单总线指令 ReadC2-4 = 0xDC)
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);   // Skip ROM
  ow_write_byte(0xDC);   // ReadC2-4
  uint8_t d[7];
  for (int i = 0; i < 7; i++) d[i] = ow_read_byte();
  gpio_low(); delayMicroseconds(2); gpio_high(); delayMicroseconds(500);
  c4 = ((uint16_t)d[5] << 8) | d[4];   // Cap4_msb<<8 | Cap4_lsb  (CHX_USE_CH1=0)
  return true;
#endif
}

// --- 触发转换 + 读 Scratchpad(带转换时间计时)----------------
uint32_t g_convMs = 0;   // 转换耗时(micros, 提高分辨率以观察双模态与转换时间的相关性)
bool convert_and_read(uint16_t& c_raw, uint8_t& b6) {
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);
  ow_write_byte(0x66);          // ConvertC
  uint32_t t0 = micros();
  if (!ow_poll_done()) return false;
  g_convMs = micros() - t0;
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

// 仅触发一次转换（不读取，不依赖任一通道数据有效性），
// 供连续监测模式按宏开关分别读取各通道。
bool convert_only() {
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);
  ow_write_byte(0x66);          // ConvertC
  uint32_t t0 = micros();
  if (!ow_poll_done()) return false;
  g_convMs = micros() - t0;
  return true;
}

// 读通道1结果（ReadScratchpad 返回的 Cap1）
bool read_ch1_raw(uint16_t& c1, uint8_t& b6) {
  if (!ow_reset()) return false;
  ow_write_byte(0xCC);
  ow_write_byte(0xBE);          // ReadScratchpad
  uint8_t d[9];
  for (int i = 0; i < 9; i++) d[i] = ow_read_byte();
  gpio_low(); delayMicroseconds(2); gpio_high(); delayMicroseconds(500);
  c1 = ((uint16_t)d[3] << 8) | d[2];
  b6 = d[6];
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

extern bool testActive;  // 定义见下方全局区,仅供 waterDebounce 引用
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

  // --- 自动保护:仅手动模式生效;测试模式由状态机独占泵控制,避免双模块打架导致抖动 ---
  if (!testActive) {
    if (io6_low_count >= WATER_TRIGGER_COUNT && digitalRead(PUMP_INTAKE_PIN) == HIGH) {
      pumpIntakeOff();
      Serial.println(F("#!IO6_LOW,INTAKE_PUMP_STOP  [高水位有水,自动停进水泵]"));
    }
    if (io7_high_count >= WATER_TRIGGER_COUNT && digitalRead(PUMP_DRAIN_PIN) == HIGH) {
      pumpDrainOff();
      Serial.println(F("#!IO7_HIGH,DRAIN_PUMP_STOP  [低水位无水,自动停出水泵]"));
    }
  }
}
bool waterEmptyStable() { return (io6_low_count == 0 && io7_high_count >= WATER_TRIGGER_COUNT); }

// IO6 高水位持续无水(WATER_TRIGGER_COUNT 次),
// 用于排水阶段:当低水位从未被触发过时,仅靠 IO6 也足以判定管已空。
bool waterEmptyByHighSensor() { return (io6_low_count == 0); }

// 满管:水淹没到高水位传感器,IO6+IO7 都为 LOW(LOW=有水)
bool waterFullStable()  { return (io6_low_count >= WATER_TRIGGER_COUNT && io7_high_count == 0); }

// --- 测试模式状态机 ------------------------------------------
enum TestState {
  TEST_IDLE = 0, TEST_ENSURE_EMPTY,
  TEST_STABILIZE,   // 停泵后等 2s 稳定再采集
  TEST_COLLECT,
  TEST_FILL,
  TEST_DRAIN_RESTORE, TEST_DONE
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
uint32_t testStabilizeStartMs = 0;   // 停泵稳定开始时刻
double  testSum = 0, testSumSq = 0;
uint8_t testCurCos = COS_CH1_DEFAULT, testCurCfb = CFB_DEFAULT, testCurConfig = REP_DEFAULT;
uint8_t testCurB6 = 0;
BatchResult results[26];
int resultCount = 0;
int positionId = 1;

// --- 连续监测状态 ----------------------------------------------
bool     contActive       = false;
uint32_t lastContMs       = 0;
bool     lastPumpActive   = false;

uint8_t getParamVal(uint8_t /*param*/, uint8_t valIdx) {
  return coVals[valIdx];
}
void getParamValues(uint8_t /*param*/, uint8_t& cos, uint8_t& cfb, uint8_t& config, uint8_t valIdx) {
  cos = coVals[valIdx];   // 仅 Co 扫描,cfb/config 固定为默认值(Cr=0x18, Rep=0x1)
  cfb = CFB_DEFAULT;
  config = REP_DEFAULT;
}
int findResult(uint8_t stage, uint8_t param, uint8_t valIdx) {
  for (int i = 0; i < resultCount; i++)
    if (results[i].stage == stage && results[i].param == param && results[i].valIdx == valIdx)
      return i;
  return -1;
}

void testStart() {
  testActive = true;
  resultCount = 0;
  testState = TEST_ENSURE_EMPTY;
  testPhaseStartMs = millis();
  pumpAllOff();
  Serial.println(F("#TEST_START  OFAT: Co(Cos) x EMPTY/FULL, 50 samples/val  (Cr=0x18 Rep=0x1 fixed)"));
  Serial.println(F("#CMD 0=stop  A/B/C/D/E=pump 5n=pos"));
}
void testStop() {
  testActive = false;
  pumpAllOff();
  apply_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT);
  Serial.println(F("#TEST_STOPPED"));
}

void testCollectSample() {
  if (testSampleIdx == 0) {
    getParamValues(testParam, testCurCos, testCurCfb, testCurConfig, testValIdx);
    testSum = 0; testSumSq = 0;
  }
  uint16_t c_raw; uint8_t b6; double c_pf;
  bool ok = measure_with_params(testCurCos, testCurCfb, testCurConfig, c_raw, b6, c_pf);
  if (ok) {
    testSum += c_pf; testSumSq += c_pf * c_pf;
    testCurB6 = b6;
  } else {
    c_pf = 0;
  }
  Serial.print(positionId);            Serial.print(',');
  Serial.print(testStage == 0 ? F("EMPTY") : F("FULL")); Serial.print(',');
  Serial.print(paramNames[testParam]); Serial.print(',');
  Serial.print(F("0x")); Serial.print(getParamVal(testParam, testValIdx), HEX); Serial.print(',');
  Serial.print(c_raw);                 Serial.print(',');
  Serial.print(c_pf, 4);              Serial.print(F("pF,"));
  Serial.print(F("b6=0x")); Serial.print(b6, HEX); Serial.print(',');
  Serial.print(g_convMs);             Serial.println(F("ms,#S"));
  testSampleIdx++;
  if (testSampleIdx >= SAMPLES_PER_BATCH) testFinishBatch();
}

void testFinishBatch() {
  double mean = testSum / SAMPLES_PER_BATCH;
  double stddev = sqrt(testSumSq / SAMPLES_PER_BATCH - mean * mean);
  results[resultCount].stage  = testStage;
  results[resultCount].param  = testParam;
  results[resultCount].valIdx = testValIdx;
  results[resultCount].mean   = mean;
  results[resultCount].stddev = stddev;
  results[resultCount].b6     = testCurB6;
  resultCount++;
  Serial.print(F("SUMMARY,"));
  Serial.print(testStage == 0 ? F("EMPTY") : F("FULL")); Serial.print(',');
  Serial.print(paramNames[testParam]); Serial.print(F(",0x"));
  Serial.print(getParamVal(testParam, testValIdx), HEX); Serial.print(',');
  Serial.print(mean, 4); Serial.print(F("pF,std=")); Serial.print(stddev, 4);
  Serial.print(F(",b6=0x")); Serial.println(testCurB6, HEX);
  // advance
  testValIdx++;
  if (testValIdx < CO_N) {
    testSampleIdx = 0;
  } else {
    if (testStage == 0) {
      testState = TEST_FILL; testPhaseStartMs = millis(); pumpIntakeOn();
      Serial.println(F("#TEST,FILL_START"));
    } else {
      testState = TEST_DRAIN_RESTORE; testPhaseStartMs = millis(); pumpDrainOn();
      Serial.println(F("#TEST,DRAIN_START"));
    }
  }
}

void printTestSummary() {
  Serial.println(F("RESULT,Param,Val,EMPTY_mean,EMPTY_std,FULL_mean,FULL_std,Delta,SNR,b6_E/b6_F"));
  for (int v = 0; v < CO_N; v++) {
    int ei = findResult(0, 0, v);
    int fi = findResult(1, 0, v);
    if (ei < 0 || fi < 0) continue;
    double delta = results[fi].mean - results[ei].mean;
    double snr = (results[ei].stddev + results[fi].stddev) > 0 ? fabs(delta) / (results[ei].stddev + results[fi].stddev) : 0;
    Serial.print(F("RESULT,"));
    Serial.print(paramNames[0]); Serial.print(F(",0x"));
    Serial.print(getParamVal(0, v), HEX); Serial.print(',');
      Serial.print(results[ei].mean, 4); Serial.print(',');
      Serial.print(results[ei].stddev, 4);  Serial.print(',');
      Serial.print(results[fi].mean, 4); Serial.print(',');
      Serial.print(results[fi].stddev, 4);  Serial.print(',');
      Serial.print(delta, 4);  Serial.print(',');
      Serial.print(snr, 3);    Serial.print(',');
      Serial.print(F("0x")); Serial.print(results[ei].b6, HEX); Serial.print('/');
      Serial.println(results[fi].b6, HEX);
  }
  Serial.println(F("#TEST_SUMMARY_END"));
}

void dbgWaterState(const char* tag) {
  static uint32_t lastDb = 0;
  if (millis() - lastDb < 500) return;
  lastDb = millis();
  Serial.print(F("#DBG,")); Serial.print(tag);
  Serial.print(F(" IN6=")); Serial.print(digitalRead(WATER_HIGH_PIN));
  Serial.print(F(" IN7=")); Serial.print(digitalRead(WATER_LOW_PIN));
  Serial.print(F(" io6_low=")); Serial.print(io6_low_count);
  Serial.print(F(" io7_high=")); Serial.print(io7_high_count);
  Serial.print(F(" pumpDrain=")); Serial.println(digitalRead(PUMP_DRAIN_PIN));
}

void runTest() {
  switch (testState) {
    case TEST_ENSURE_EMPTY:
      dbgWaterState("ENSURE_EMPTY");
      // 条件独立顺序检查,保证每种情况都有对应动作
      if (!waterEmptyStable()) {
        // 未达到空管条件,持续排水
        pumpDrainOn();
      }
      // 必须 IO6+IO7 都高(水位低于低水位)才算空管,开始采集
      if (waterEmptyStable()) {
        // 电极露出,停泵等稳定
        pumpDrainOff();
        testStage = 0; testParam = 0; testValIdx = 0; testSampleIdx = 0;
        testState = TEST_STABILIZE;
        testStabilizeStartMs = millis();
        Serial.println(F("#TEST,EMPTY_STAGE_START"));
      }
      if (millis() - testPhaseStartMs > DRAIN_TIMEOUT_MS) {
        pumpDrainOff();
        Serial.println(F("#TEST,ERR,EMPTY_TIMEOUT")); testStop();
      }
      break;
    case TEST_STABILIZE:
      if (millis() - testStabilizeStartMs >= 2000) {
        testState = TEST_COLLECT;
      }
      break;
    case TEST_COLLECT:
      testCollectSample();
      delay(SAMPLE_GAP_MS);
      break;
    case TEST_FILL:
      // 未到高水位时,确保进水泵开着
      if (!waterFullStable() && millis() - testPhaseStartMs <= FILL_TIMEOUT_MS) {
        pumpIntakeOn();
      }
      // 水满(IO6+IO7 都为 LOW)时停泵等稳定
      if (waterFullStable()) {
        pumpIntakeOff();
        testStage = 1; testParam = 0; testValIdx = 0; testSampleIdx = 0;
        testState = TEST_STABILIZE;
        testStabilizeStartMs = millis();
        Serial.println(F("#TEST,FULL_STAGE_START"));
      } else if (millis() - testPhaseStartMs > FILL_TIMEOUT_MS) {
        pumpIntakeOff();
        Serial.println(F("#TEST,ERR,FILL_TIMEOUT")); testStop();
      }
      break;
    case TEST_DRAIN_RESTORE:
      dbgWaterState("DRAIN_RESTORE");
      // 未干态时,持续排水
      if (!waterEmptyStable() && !waterEmptyByHighSensor()) {
        pumpDrainOn();
      }
      if (waterEmptyStable() || waterEmptyByHighSensor()) {
        pumpDrainOff();
        testState = TEST_DONE;
        Serial.println(F("#TEST,DONE"));
        printTestSummary();
        testActive = false;
        apply_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT);
      } else if (millis() - testPhaseStartMs > DRAIN_TIMEOUT_MS) {
        pumpDrainOff();
        Serial.println(F("#TEST,ERR,RESTORE_TIMEOUT")); testStop();
      }
      break;
    case TEST_DONE:
    case TEST_IDLE:
      testActive = false;
      break;
  }
}

// --- CH4 流态检测: 流水时电容比静水基线低, 用相对偏移判定; 流水线期冻结基线 ---
void ch4FlowDetect(double c4_pf, uint32_t now) {
  if (baseline_ch4 <= 0.0) {            // 首次有效读数播种基线
    baseline_ch4 = c4_pf;
    return;
  }
  double dev = c4_pf - baseline_ch4;     // 负=比静水低(偏流水方向)

  if (!flowActive) {
    // 静止期: 缓慢跟踪静水基线; 判定是否进入流水
    baseline_ch4 = baseline_ch4 * (1 - FLOW_BASELINE_ALPHA) + c4_pf * FLOW_BASELINE_ALPHA;
    if (dev < -FLOW_DELTA_PF) flowOverCount++; else flowOverCount = 0;
    if (flowOverCount >= FLOW_OVER_COUNT_TRIG) {
      flowActive = true;
      flowStartMs = now;
      uploadEvent(EVT_CH4_FLOW_START, 0);
      Serial.print(F("#C_EVT,3,")); Serial.print(now); Serial.print(',');
      Serial.println(g_eventDurationS);
      Serial.print(F("*** CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
      Serial.print(F(" FLOW START: dC=")); Serial.print(dev, 4);
      Serial.print(F("pF -> LoRa upload event=3 ***"));
      lastFlowDbgMs = now;
    }
  } else {
    // 流水期: 冻结基线; 判定是否回到静止
    if (dev > -FLOW_RECOVER_PF) flowRecoverCount++; else flowRecoverCount = 0;
    if (flowRecoverCount >= FLOW_RECOVER_TRIG) {
      flowActive = false;
      uint32_t dur = now - flowStartMs;
      uploadEvent(EVT_CH4_FLOW_STOP, dur);
      Serial.print(F("#C_EVT,4,")); Serial.print(now); Serial.print(',');
      Serial.println(g_eventDurationS);
      Serial.print(F("*** CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
      Serial.print(F(" FLOW STOP: lasted "));
      Serial.print(dur / 1000); Serial.print(F("s dC=")); Serial.print(dev, 4);
      Serial.println(F("pF -> LoRa upload event=4 ***"));
      lastFlowDbgMs = now;
    }
  }

  // 调试信息: 流水期间每 2s 输出一次
  if (flowActive && (now - lastFlowDbgMs >= APP_DEBUG_MS)) {
    lastFlowDbgMs = now;
    Serial.print(F("[CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
    Serial.print(F(" DBG] dC=")); Serial.print(dev >= 0 ? "+" : "");
    Serial.print(dev, 4); Serial.print(F("pF  C")); Serial.print(CHX_USE_CH1 ? '1' : '4'); Serial.print(F("=")); Serial.print(c4_pf, 4);
    Serial.print(F("pF  baseline=")); Serial.print(baseline_ch4, 4);
    Serial.print(F("pF  flowOver=")); Serial.print((int)flowOverCount);
    Serial.print(F("  flowRecover=")); Serial.println((int)flowRecoverCount);
  }
}

// --- 应用监测模式（默认运行: CH1 水位 + CH4 流态, 双阈值滞回, LoRa 事件上报）---
void appMonitor() {
  // ---- CH1 测量（水位检测，仅在启用时）----
  uint16_t c_raw = 0; uint8_t b6 = 0; double c_pf = 0;
  bool ok = false;
#if ENABLE_CH1_DETECT
  ok = measure_with_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT, c_raw, b6, c_pf);
#endif

  // ---- CH4 测量（流态检测，仅在启用时）----
  // CH1 已测量时会触发一次 ConvertC，可直接读 C4；CH1 禁用时单独触发一次转换
  double c4_pf = 0; bool ok4 = false; uint16_t c4_raw = 0;
#if ENABLE_CH4_DETECT
#if !ENABLE_CH1_DETECT
  apply_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT);
  delay(3);
  bool convOk = convert_only();
  if (convOk && read_ch4_raw(c4_raw) && c4_raw >= RAW_MIN_VALID) {
    c4_pf = 15.5 + c4_raw / 32768.0 * 31.0; ok4 = true;
  }
#else
  if (read_ch4_raw(c4_raw) && c4_raw >= RAW_MIN_VALID) {
    c4_pf = 15.5 + c4_raw / 32768.0 * 31.0; ok4 = true;
  }
#endif
#endif

  if (!ENABLE_CH1_DETECT && !ENABLE_CH4_DETECT) return;  // 两通道都关, 无事可做

  uint32_t now = millis();

  // ===== CH1 水位检测 =====
#if ENABLE_CH1_DETECT
  if (!ok) {
    appOverCount = 0; appRecoverCount = 0;   // 通信失败: 清零, 不误报/不上传
  } else {
    double dC = c_pf - baseline_ch1;
    // 双阈值滞回: >2.0pF 累积触发计数, <1.9pF 累积恢复计数, 中间区保持(防边界抖动)
    if (dC > APP_OVER_DELTA_PF) {
      appOverCount++; appRecoverCount = 0;
    } else if (dC < APP_RECOVER_DELTA_PF) {
      appRecoverCount++; appOverCount = 0;
    }
    // 1.9~2.0pF 之间: 两个计数均不变

    // 状态跳变(各只上传一次): NORMAL->ALERT 上传事件1, ALERT->NORMAL 上传事件0
    if (!appAlert && appOverCount >= APP_OVER_COUNT_TRIG) {
      appAlert = true;
      alertStartMs = now;
      uploadEvent(EVT_CH1_ALERT, 0);   // 触发: 持续时长未知, 填 0
      Serial.print(F("#C_EVT,1,")); Serial.print(now); Serial.print(',');
      Serial.println(g_eventDurationS);   // 触发调试行(含 #C, 供绘图识别)
      Serial.print(F("*** APP ALERT: dC>+")); Serial.print(APP_OVER_DELTA_PF, 1);
      Serial.print(F("pF x")); Serial.print((int)APP_OVER_COUNT_TRIG);
      Serial.println(F(" -> LoRa upload event=1 ***"));
      lastAppDbgMs = now;
    } else if (appAlert && appRecoverCount >= APP_OVER_COUNT_TRIG) {
      appAlert = false;
      uint32_t dur = now - alertStartMs;   // 计算本次 ALERT 持续时长
      uploadEvent(EVT_CH1_RECOVER, dur);   // 恢复: 上传持续时长
      Serial.print(F("#C_EVT,0,")); Serial.print(now); Serial.print(',');
      Serial.println(g_eventDurationS);   // 恢复调试行(含 #C, 含持续时长秒)
      Serial.print(F("*** APP NORMAL: dC<1.9pF x10, lasted "));
      Serial.print(dur / 1000); Serial.print(F("s -> LoRa upload event=0, duration="));
      Serial.print(dur / 1000); Serial.println(F("s ***"));
      lastAppDbgMs = now;
    }

    // 调试信息: 触发期间每 2s 输出一次
    if (appAlert && (now - lastAppDbgMs >= APP_DEBUG_MS)) {
      lastAppDbgMs = now;
      Serial.print(F("[APP DBG] dC=")); Serial.print(dC >= 0 ? "+" : "");
      Serial.print(dC, 4);  Serial.print(F("pF  C="));  Serial.print(c_pf, 4);
      Serial.print(F("pF  baseline=")); Serial.print(baseline_ch1, 4);
      Serial.print(F("pF  raw=")); Serial.print(c_raw);
      Serial.print(F("  overCount=")); Serial.print((int)appOverCount);
      Serial.print(F("  recoverCount=")); Serial.println((int)appRecoverCount);
    }
  }
#endif

  // ===== CH4 流态检测 =====
#if ENABLE_CH4_DETECT
  if (!ok4) {
    flowOverCount = 0; flowRecoverCount = 0;  // 通信失败: 清零, 不误报/不上传
  } else {
    ch4FlowDetect(c4_pf, now);
  }
#endif
}

// --- 正常监控模式（保留, 未被默认调用）----------------------
bool silentMode = true;    // true=仅打印事件,不打印每轮 [CH1] 轮询
uint32_t good_ch1 = 0;
uint32_t total_iter = 0;
const double WET_DELTA_THRESHOLD = 1.0;
const double STABLE_MAX_DELTA   = 0.15;
int     stable_count_ch1 = 0;
bool    wet_alert_shown = false;

void startup_calibration() {
#if !ENABLE_CH1_DETECT
  Serial.println(F("--- Startup calibration skipped (CH1 disabled) ---"));
  return;
#endif
  Serial.println(F("--- Startup calibration (dry sensors) ---"));
  double sum = 0; int ok = 0;
  for (int i = 0; i < 10 && ok < 5; i++) {
    uint16_t c_raw; uint8_t b6; double c_pf;
    if (measure_with_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT, c_raw, b6, c_pf)) {
      sum += c_pf; ok++;
      Serial.print(F("[Cal] CH1: ")); Serial.print(c_pf, 4); Serial.println(F("pF"));
    } else {
      Serial.print(F("[Cal] CH1: -- raw=")); Serial.println(c_raw);
    }
    delay(1000);
  }
  if (ok > 0) baseline_ch1 = sum / ok;
  Serial.print(F("Baseline CH1=")); Serial.print(baseline_ch1, 4); Serial.println(F("pF"));
  stable_count_ch1 = ok;
}

void normalMonitor() {
#if !ENABLE_CH1_DETECT
  return;  // 通道1检测已禁用，跳过正常轮询
#endif
  uint32_t ts = millis();
  uint16_t c_raw; uint8_t b6; double c_pf;
  bool ok = measure_with_params(COS_CH1_DEFAULT, CFB_DEFAULT, REP_DEFAULT, c_raw, b6, c_pf);
  double delta = ok ? (c_pf - baseline_ch1) : 999.0;
  bool is_wet = ok && (delta > WET_DELTA_THRESHOLD);
  if (is_wet && !wet_alert_shown) {
    Serial.print(F("*** WET DETECTED *** dC=")); Serial.print(delta, 4); Serial.println(F("pF"));
    wet_alert_shown = true;
  } else if (!is_wet && wet_alert_shown) {
    Serial.println(F("*** DRY RESTORED ***"));
    wet_alert_shown = false;
  }
  if (!silentMode) {
    Serial.print(F("[CH1] raw=")); Serial.print(c_raw);
    Serial.print(F(" C=")); Serial.print(c_pf, 4); Serial.print(F("pF"));
    if (ok && baseline_ch1 > 0) {
      Serial.print(F(" dC=")); Serial.print(delta >= 0 ? "+" : ""); Serial.print(delta, 4); Serial.println(F("pF"));
    } else Serial.println(F(" [BAD]"));
  }
  if (ok) {
    if (delta > -STABLE_MAX_DELTA && delta < STABLE_MAX_DELTA) {
      stable_count_ch1++;
      if (stable_count_ch1 >= 10) { baseline_ch1 = baseline_ch1 * 0.95 + c_pf * 0.05; stable_count_ch1 = 10; }
    } else stable_count_ch1 = 0;
    good_ch1++;
  }
  total_iter++;
  if (!silentMode && total_iter % 30 == 0) {
    Serial.print(F("--- stats: CH1 ")); Serial.print(good_ch1);
    Serial.print(F("/")); Serial.print(total_iter);
    Serial.print(F(" baseline=")); Serial.print(baseline_ch1, 4); Serial.println(F("pF ---"));
  }
  uint32_t elapsed = millis() - ts;
  // 延迟移到 loop() 级别,让 waterDebounce() 能以 10ms 间隔持续检测
}

// --- 连续监测模式 --------------------------------------------
bool pumpActive() {
  return (digitalRead(PUMP_INTAKE_PIN) == HIGH || digitalRead(PUMP_DRAIN_PIN) == HIGH);
}

void contStart() {
  contActive = true;
  lastContMs = 0;
  lastPumpActive = false;
  apply_params(COS_CONT, CFB_DEFAULT, REP_DEFAULT);
  write_ch_sel(CHSEL_CONT);              // 按宏开关开启 CH1 / 通道X（可能同时或单独）
  uint8_t chsel_rb = read_ch_sel();      // 回读确认写入是否生效
  Serial.print(F("#CONT_START Co=0x")); Serial.print(COS_CONT, HEX);
  Serial.print(F(", realtime  ("));
#if ENABLE_CH1_DETECT
  Serial.print(F("CH1"));
#if ENABLE_CH4_DETECT
  Serial.print(F(" + CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
#endif
#else
#if ENABLE_CH4_DETECT
  Serial.print(F("CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
#else
  Serial.print(F("NONE"));
#endif
#endif
  Serial.println(F(")"));
  Serial.print(F("#CHSEL write=0x")); Serial.print(CHSEL_CONT, HEX);
  Serial.print(F(" read=0x")); Serial.println(chsel_rb, HEX);
#if (ENABLE_CH1_DETECT == 0 && ENABLE_CH4_DETECT == 0)
  Serial.println(F("#WARN: 两个通道检测均已禁用，ConvertC 不转换任何通道，数据将全部 ERR"));
#endif
  Serial.println(F("#CONT_FMT #C,<uptime_ms>,<raw1>,<pF1>,<b6>,<conv_us>,<#C_END>  #C4,<uptime_ms>,<raw4>,<pF4>,<#C4_END>  (single-sample, no averaging)"));
  // 快速标定基线（仅当 CH1 启用时）
#if ENABLE_CH1_DETECT
  double sum = 0; int ok = 0;
  for (int i = 0; i < 5; i++) {
    uint16_t r; uint8_t b6; double pf;
    if (measure_with_params(COS_CONT, CFB_DEFAULT, REP_DEFAULT, r, b6, pf)) { sum += pf; ok++; }
    delay(200);
  }
  if (ok > 0) baseline_ch1 = sum / ok;
  Serial.print(F("#CONT_BASELINE=")); Serial.print(baseline_ch1, 4); Serial.println(F("pF"));
#else
  Serial.println(F("#CONT_BASELINE= N/A (CH1 disabled)"));
#endif
}

void contStop() {
  contActive = false;
  pumpAllOff();
  Serial.println(F("#CONT_STOPPED"));
}

void runContinuous() {
  if (millis() - lastContMs < CONT_SAMPLE_MS) return;
  lastContMs = millis();

  // 泵状态跟踪（仅打标，与 '6' 的 #TEST 事件同性质）
  bool pa = pumpActive();
  if (pa != lastPumpActive) {
    lastPumpActive = pa;
    Serial.print(F("#PUMP,")); Serial.print(millis()); Serial.print(',');
    Serial.println(pa ? F("ON") : F("OFF"));
  }

  uint32_t ms = millis();

  // 触发一次转换（转换哪些通道由 CHSEL_CONT 决定），不依赖任一通道数据有效性
  if (!convert_only()) {
    // 通信失败：各启用通道均打印 ERR
#if ENABLE_CH1_DETECT
    Serial.print(F("#C,"));  Serial.print(ms); Serial.print(',');
    Serial.println(F("0,0.0000pF,b6=0x-,ERR,#C_END"));
#endif
#if ENABLE_CH4_DETECT
    Serial.print(F("#C4,")); Serial.print(ms); Serial.print(',');
    Serial.println(F("0,0.0000pF,ERR,#C4_END"));
#endif
    return;
  }

  // --- 通道1 ---
#if ENABLE_CH1_DETECT
  {
    uint16_t c_raw; uint8_t b6;
    if (read_ch1_raw(c_raw, b6) && c_raw >= RAW_MIN_VALID) {
      double c_pf = 15.5 + c_raw / 32768.0 * 31.0;
      Serial.print(F("#C,"));
      Serial.print(ms); Serial.print(',');
      Serial.print(c_raw);                 Serial.print(',');
      Serial.print(c_pf, 4);               Serial.print(F("pF,"));
      Serial.print(F("b6=0x"));            Serial.print(b6, HEX); Serial.print(',');
      Serial.print(g_convMs);              Serial.println(F("us,#C_END"));
    } else {
      Serial.print(F("#C,")); Serial.print(ms); Serial.print(',');
      Serial.println(F("0,0.0000pF,b6=0x-,ERR,#C_END"));
    }
  }
#endif

  // --- 通道4 ---
#if ENABLE_CH4_DETECT
  {
    uint16_t c4_raw;
    if (read_ch4_raw(c4_raw)) {
      double c4_pf = 15.5 + c4_raw / 32768.0 * 31.0;
      Serial.print(F("#C4,"));
      Serial.print(ms); Serial.print(',');
      Serial.print(c4_raw);                Serial.print(',');
      Serial.print(c4_pf, 4);              Serial.println(F("pF,#C4_END"));
    } else {
      Serial.print(F("#C4,")); Serial.print(ms); Serial.print(',');
      Serial.println(F("0,0.0000pF,ERR,#C4_END"));
    }
  }
#endif
}

// --- 命令 ----------------------------------------------------
void handleSerial() {
  if (Serial.available() == 0) return;
  char cmd = Serial.read();
  if (cmd == '6') { if (!testActive && !contActive) testStart(); }
  else if (cmd == '7') { if (!testActive && !contActive) contStart(); }
  else if (cmd == '0') { if (testActive) testStop(); else if (contActive) contStop(); }
  else if (cmd == 'A') pumpIntakeOn();
  else if (cmd == 'B') pumpIntakeOff();
  else if (cmd == 'C') pumpDrainOn();
  else if (cmd == 'D') pumpDrainOff();
  else if (cmd == 'E') pumpAllOff();
  else if (cmd == 'S') {
    silentMode = !silentMode;
    Serial.print(F("[MODE] silent=")); Serial.println(silentMode ? F("ON (events only)") : F("OFF (full poll)"));
  }
  else if (cmd == '5') {
    delay(50);
    if (Serial.available()) {
      char p = Serial.read(); int np = p - '0';
      if (np >= 0 && np <= 9) positionId = np;
    }
  }
}

// --- 初始化 --------------------------------------------------
void setup() {
  pinMode(PUMP_INTAKE_PIN, OUTPUT); digitalWrite(PUMP_INTAKE_PIN, LOW);
  pinMode(PUMP_DRAIN_PIN,  OUTPUT); digitalWrite(PUMP_DRAIN_PIN,  LOW);
  pinMode(WATER_HIGH_PIN,  INPUT);
  pinMode(WATER_LOW_PIN,   INPUT);

  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);   // 初始化 Heltec 板载 LoRa 等外设
  delay(300);
  Serial.println(F("\n=== MDC04 v31 - CH1 Monitor + OFAT Test ==="));
  Serial.println(F("Pins: DQ=2  PumpIn=4  PumpDrain=5  WHigh=6  WLow=7"));
  Serial.println(F("App: CH1 10ms poll, dC>+2pF x10 -> 2s debug | Test: '6' OFAT sweep | Cont: '7' continuous monitor"));
  Serial.print(F("Channel switches: CH1=")); Serial.print(ENABLE_CH1_DETECT);
  Serial.print(F(" CHX=CH")); Serial.print(CHX_USE_CH1 ? '1' : '4');
  Serial.print(F("(")); Serial.print(ENABLE_CH4_DETECT); Serial.println(F(")"));
  gpio_init();
  delay(2000);
  startup_calibration();
  write_ch_sel(CHSEL_CONT);   // 按宏开关开启 CH1/CH4 转换（appMonitor 模式也需使能 CH4）
  uint8_t chsel_rb = read_ch_sel();
  Serial.print(F("#CHSEL write=0x")); Serial.print(CHSEL_CONT, HEX);
  Serial.print(F(" read=0x")); Serial.println(chsel_rb, HEX);
  Serial.println(F("--- Start polling (send 6 for test) ---\n"));
}

void loop() {
  // 水位检测:非阻塞,loop 每次都进来(约 Hz 级)
  // 注意:contActive 期间 runContinuous() 会阻塞数百毫秒,
  //       此 waterDebounce() 调用频率会自然降至每周期 1 次,已足够。
  waterDebounce();

  if (testActive) {
    runTest();
  } else if (contActive) {
    runContinuous();
  } else {
    // 应用监测: 每 10ms 采样一次（基于测试17 跨液面基线, ΔC 连续>2pF 报警）
    if (millis() - lastAppMs >= APP_SAMPLE_MS) {
      lastAppMs = millis();
      appMonitor();
    }
  }

  handleSerial();
  loraTask();      // 处理 LoRa 入网/发送(事件驱动)
  yield();
}
