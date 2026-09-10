/*
SO-101 Leader Arm Emulator — PRODUCTION FIRMWARE
Arduino Nano / Uno / Pro Mini (ATmega328P)
6x AS5600 via TCA9548A I2C multiplexer
FeeTech STS3215 Protocol v1 over USB-UART @ 1Mbps
EEPROM calibration (zero offsets + invert flags)
CALIBRATION: Hold button on A0 during power-on for 2 sec
 */
#include <Wire.h>
#include <EEPROM.h>
// ═══════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define BAUDRATE_FEE      1000000UL
#define BROADCAST_ID      0xFE
#define NUM_JOINTS        6
// I2C
#define TCA9548A_ADDR     0x70
#define AS5600_ADDR       0x36
// Calibration button — УДАЛЕНО: A0 физически не подключена (голый вход),
// наводка на неё при auto-reset порта вызывала ложный запуск 2-сек
// блокирующего ожидания в setup(), из-за чего MakerMods Lab не получал
// ответ по Serial 1-3 сек и падал. Калибровка теперь только программно,
// см. функцию calibrateAll() (можно дёрнуть её отдельной командой при желании).
#define LED_PIN           LED_BUILTIN
// Servo IDs (must match LeRobot config)
const uint8_t SERVO_IDS[NUM_JOINTS] = {1, 2, 3, 4, 5, 6};
// Default invert direction (set true if angle decreases on forward motion)
const bool DEFAULT_INVERT[NUM_JOINTS] = {false, false, false, false, false, false};
// EEPROM layout
#define EEPROM_MAGIC      0x4242
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_DATA  2
typedef struct {
  int16_t  zero_offset[NUM_JOINTS];
  uint8_t  invert_mask;  // bit0=joint0, bit1=joint1, ...
} calibration_t;
// ═══════════════════════════════════════════════════════════════
// FEE-TECH PROTOCOL CONSTANTS (CORRECTED)
// ═══════════════════════════════════════════════════════════════
#define INST_PING         0x01
#define INST_READ         0x02
#define INST_WRITE        0x03
#define INST_REG_WRITE    0x04
#define INST_ACTION       0x05
#define INST_RESET        0x06
#define INST_SYNC_READ    0x82
#define INST_SYNC_WRITE   0x83
#define REG_MODEL_NUMBER        0x03
#define REG_ID                  0x05
#define REG_BAUD_RATE           0x06
#define REG_MIN_ANGLE_LIMIT     0x09
#define REG_MAX_ANGLE_LIMIT     0x0B
#define REG_TORQUE_ENABLE       0x28
#define REG_GOAL_POSITION       0x2A
#define REG_GOAL_SPEED          0x2E
#define REG_PRESENT_POSITION    0x38
#define REG_PRESENT_SPEED       0x3A
#define REG_PRESENT_LOAD        0x3C
#define REG_PRESENT_VOLTAGE     0x3E
#define REG_PRESENT_TEMPERATURE 0x3F
#define REG_MOVING              0x42
// Кастомный регистр (не существует в реальном протоколе STS3215, поэтому
// безопасно переиспользовать): запись любого байта сюда обнуляет текущее
// положение этого сустава как новый ноль калибровки (взамен убранной
// кнопки на A0). Калибровать нужно по одному суставу, стоя на КРАЮ его
// физического хода — тогда граница шкалы 4095/0 уйдёт туда, куда сустав
// не доезжает, и скачков при движении не будет.
#define REG_CALIBRATE_ZERO      0x50
#define MODEL_NUMBER_STS3215    777
// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════
uint16_t cached_positions[NUM_JOINTS] = {2048, 2048, 2048, 2048, 2048, 2048};
uint8_t  reg_torque_enable[NUM_JOINTS] = {0};
uint16_t reg_goal_position[NUM_JOINTS] = {2048, 2048, 2048, 2048, 2048, 2048};
uint16_t reg_goal_speed[NUM_JOINTS] = {0};
uint8_t  reg_moving[NUM_JOINTS] = {0};
uint16_t reg_min_angle[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
uint16_t reg_max_angle[NUM_JOINTS] = {4095, 4095, 4095, 4095, 4095, 4095};
calibration_t cal;
enum ParserState {
    WAIT_FF1, WAIT_FF2, WAIT_ID, WAIT_LEN, READ_DATA, WAIT_CHKSUM
};
ParserState parser_state = WAIT_FF1;
uint8_t pkt_id = 0;
uint8_t pkt_length = 0;
uint8_t pkt_data[32];
uint8_t pkt_data_idx = 0;
uint8_t calc_checksum = 0;
unsigned long last_sensor_update = 0;
uint8_t current_sensor = 0;
unsigned long last_parser_byte_ms = 0;
#define PARSER_TIMEOUT_MS 15
// ═══════════════════════════════════════════════════════════════
// I2C & AS5600
// ═══════════════════════════════════════════════════════════════
bool selectI2CChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    if (Wire.endTransmission() != 0) return false;
    // Читаем обратно регистр мультиплексора: если из-за наводки/вибрации
    // селект не прошёл, следующее чтение AS5600 молча вернёт данные с
    // ЧУЖОГО канала (без ошибки Wire) — именно это давало "прыжок на
    // другую ось" при движении соседнего сустава. Если readback не
    // совпал — просто пропускаем этот замер.
    Wire.requestFrom(TCA9548A_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return false;
    uint8_t actual = Wire.read();
    return actual == (1 << channel);
}
uint16_t readAS5600_Raw(uint8_t channel) {
    if (!selectI2CChannel(channel)) return 0xFFFF;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return 0xFFFF;
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        return (((uint16_t)(hi & 0x0F)) << 8) | lo;
    }
    return 0xFFFF;
}
uint16_t applyCalibration(uint8_t idx, uint16_t raw) {
    if (raw == 0xFFFF) return cached_positions[idx];
    int32_t pos = (int32_t)raw + cal.zero_offset[idx];
    while (pos < 0) pos += 4096;
    while (pos > 4095) pos -= 4096;
    if (cal.invert_mask & (1 << idx)) pos = 4095 - pos;
    return (uint16_t)pos;
}
// ═══════════════════════════════════════════════════════════════
// EEPROM CALIBRATION
// ═══════════════════════════════════════════════════════════════
void loadCalibration() {
    uint16_t magic;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);
    if (magic == EEPROM_MAGIC) {
        EEPROM.get(EEPROM_ADDR_DATA, cal);
    } else {
        // Первая загрузка / пустой EEPROM: смещения = 0, инверсия берётся
        // из DEFAULT_INVERT[] (раньше этот массив объявлялся, но нигде не
        // применялся, из-за чего инверсия направления не работала вообще)
        cal.invert_mask = 0;
        for (uint8_t i = 0; i < NUM_JOINTS; i++) {
            cal.zero_offset[i] = 0;
            if (DEFAULT_INVERT[i]) cal.invert_mask |= (1 << i);
        }
    }
}
void saveCalibration() {
    uint16_t magic = EEPROM_MAGIC;
    EEPROM.put(EEPROM_ADDR_MAGIC, magic);
    EEPROM.put(EEPROM_ADDR_DATA, cal);
}
void calibrateOne(uint8_t idx) {
    // Свежее чтение мимо фильтра сглаживания — калибровка должна быть точной.
    // ВАЖНО: калибровать нужно, когда сустав стоит в СЕРЕДИНЕ хода (как и
    // просит калибровка LeRobot/bambot), а не на краю. Середина мапится на
    // 2048 (центр шкалы 0-4095), а не на 0 — так у сустава остаётся запас
    // ~2048 отсчётов в обе стороны до границы 4095/0, и обычный ход сустава
    // никогда её не пересекает (что и вызывало "Motor discontinuity detected").
    uint16_t raw = readAS5600_Raw(idx);
    if (raw != 0xFFFF) {
        cal.zero_offset[idx] = 2048 - ((int16_t)raw); // середина хода -> 2048
        saveCalibration();
        cached_positions[idx] = applyCalibration(idx, raw); // сразу применяем, не ждём опроса
    }
}
void calibrateAll() {
    // Read current positions and store as zero offsets
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        uint16_t raw = readAS5600_Raw(i);
        if (raw != 0xFFFF) {
            cal.zero_offset[i] = -((int16_t)raw); // so calibrated = 0
        }
        delay(5);
    }
    saveCalibration();
}
// ═══════════════════════════════════════════════════════════════
// FEE-TECH CHECKSUM & TX
// ═══════════════════════════════════════════════════════════════
uint8_t calcChecksum(uint8_t id, uint8_t len, uint8_t inst, const uint8_t* p, uint8_t n) {
    uint16_t sum = id + len + inst;
    for (uint8_t i = 0; i < n; i++) sum += p[i];
    return (~(sum & 0xFF)) & 0xFF;
}
void sendStatus(uint8_t id, uint8_t err) {
    uint8_t c = calcChecksum(id, 2, err, NULL, 0);
    Serial.write(0xFF); Serial.write(0xFF);
    Serial.write(id); Serial.write(2);
    Serial.write(err); Serial.write(c);
}
void sendData(uint8_t id, uint8_t err, const uint8_t* d, uint8_t n) {
    uint8_t len = n + 2;
    uint8_t c = calcChecksum(id, len, err, d, n);
    Serial.write(0xFF); Serial.write(0xFF);
    Serial.write(id); Serial.write(len);
    Serial.write(err);
    for (uint8_t i = 0; i < n; i++) Serial.write(d[i]);
    Serial.write(c);
}
void sendPing(uint8_t id) { sendStatus(id, 0x00); }
void sendModel(uint8_t id) {
    uint8_t d[2] = {MODEL_NUMBER_STS3215 & 0xFF, (MODEL_NUMBER_STS3215 >> 8) & 0xFF};
    sendData(id, 0x00, d, 2);
}
void sendPosition(uint8_t id, uint16_t pos) {
    uint8_t d[2] = {pos & 0xFF, (pos >> 8) & 0xFF};
    sendData(id, 0x00, d, 2);
}
void sendTorque(uint8_t id, uint8_t idx) {
    uint8_t d[1] = {reg_torque_enable[idx]};
    sendData(id, 0x00, d, 1);
}
void sendWord(uint8_t id, uint16_t val) {
    uint8_t d[2] = {val & 0xFF, (val >> 8) & 0xFF};
    sendData(id, 0x00, d, 2);
}
void sendByte(uint8_t id, uint8_t val) {
    uint8_t d[1] = {val};
    sendData(id, 0x00, d, 1);
}
// ═══════════════════════════════════════════════════════════════
// PACKET HANDLERS
// ═══════════════════════════════════════════════════════════════
int8_t findIndex(uint8_t id) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) if (SERVO_IDS[i] == id) return i;
    return -1;
}
void handleRead(uint8_t id, const uint8_t* p, uint8_t n) {
    if (n < 2) return;
    int8_t idx = findIndex(id);
    if (idx < 0) return;
    uint8_t addr = p[0];
    uint8_t len  = p[1];
if (addr == REG_MODEL_NUMBER && len == 2) { sendModel(id); return; }
if (addr == REG_TORQUE_ENABLE && len == 1) { sendTorque(id, idx); return; }
if (addr == REG_PRESENT_POSITION && len == 2) { sendPosition(id, cached_positions[idx]); return; }
if (addr == REG_PRESENT_SPEED && len == 2) { sendWord(id, 0); return; }
if (addr == REG_PRESENT_LOAD && len == 2) { sendWord(id, 0); return; }
if (addr == REG_PRESENT_VOLTAGE && len == 1) { sendByte(id, 120); return; }
if (addr == REG_PRESENT_TEMPERATURE && len == 1) { sendByte(id, 30); return; }
if (addr == REG_MOVING && len == 1) { sendByte(id, reg_moving[idx]); return; }
if (addr == REG_GOAL_POSITION && len == 2) { sendWord(id, reg_goal_position[idx]); return; }
if (addr == REG_GOAL_SPEED && len == 2) { sendWord(id, reg_goal_speed[idx]); return; }
if (addr == REG_MIN_ANGLE_LIMIT && len == 2) { sendWord(id, reg_min_angle[idx]); return; }
if (addr == REG_MAX_ANGLE_LIMIT && len == 2) { sendWord(id, reg_max_angle[idx]); return; }
if (addr == REG_BAUD_RATE && len == 1) { sendByte(id, 0); return; }
if (addr == REG_ID && len == 1) { sendByte(id, id); return; }
// Регистр не реализован (Lock, Acceleration, Torque_Limit и т.п.) —
// отдаём нули нужной длины вместо статус-ошибки без данных, чтобы не
// ломать разбор пакета на стороне scservo_sdk/LeRobot
if (len == 1) { sendByte(id, 0); return; }
if (len == 2) { sendWord(id, 0); return; }
sendStatus(id, 0x02);
}
void handleWrite(uint8_t id, const uint8_t* p, uint8_t n) {
    if (n < 2) return;
    int8_t idx = findIndex(id);
    if (idx < 0) return;
    uint8_t addr = p[0];
if (addr == REG_TORQUE_ENABLE && n >= 2) { reg_torque_enable[idx] = p[1]; sendStatus(id, 0x00); return; }
if (addr == REG_GOAL_POSITION && n >= 3) { reg_goal_position[idx] = p[1] | (p[2] << 8); sendStatus(id, 0x00); return; }
if (addr == REG_GOAL_SPEED && n >= 3) { reg_goal_speed[idx] = p[1] | (p[2] << 8); sendStatus(id, 0x00); return; }
if (addr == REG_MIN_ANGLE_LIMIT && n >= 3) { reg_min_angle[idx] = p[1] | (p[2] << 8); sendStatus(id, 0x00); return; }
if (addr == REG_MAX_ANGLE_LIMIT && n >= 3) { reg_max_angle[idx] = p[1] | (p[2] << 8); sendStatus(id, 0x00); return; }
if (addr == REG_CALIBRATE_ZERO) { calibrateOne(idx); sendStatus(id, 0x00); return; }
sendStatus(id, 0x00);
}
void handleSyncRead(const uint8_t* p, uint8_t n) {
    if (n < 3) return;
    uint8_t addr = p[0];
    uint8_t len  = p[1];
    uint8_t ids  = n - 2;
    for (uint8_t i = 0; i < ids; i++) {
        uint8_t tid = p[2 + i];
        int8_t idx = findIndex(tid);
        if (idx < 0) continue;
        if (addr == REG_PRESENT_POSITION && len == 2) sendPosition(tid, cached_positions[idx]);
        else if (addr == REG_TORQUE_ENABLE && len == 1) sendTorque(tid, idx);
        else if (addr == REG_MODEL_NUMBER && len == 2) sendModel(tid);
        else if (addr == REG_MIN_ANGLE_LIMIT && len == 2) sendWord(tid, reg_min_angle[idx]);
        else if (addr == REG_MAX_ANGLE_LIMIT && len == 2) sendWord(tid, reg_max_angle[idx]);
        else if (addr == REG_PRESENT_SPEED && len == 2) sendWord(tid, 0);
        else if (addr == REG_PRESENT_VOLTAGE && len == 1) sendByte(tid, 120);
        else if (addr == REG_PRESENT_TEMPERATURE && len == 1) sendByte(tid, 30);
        else if (addr == REG_MOVING && len == 1) sendByte(tid, reg_moving[idx]);
        else sendStatus(tid, 0x02);
        delayMicroseconds(50);
    }
}
void handleSyncWrite(const uint8_t* p, uint8_t n) {
    if (n < 2) return;
    uint8_t addr = p[0];
    uint8_t dlen = p[1];
    uint8_t esize = dlen + 1;
    uint8_t entries = (n - 2) / esize;
    for (uint8_t i = 0; i < entries; i++) {
        uint8_t off = 2 + i * esize;
        uint8_t tid = p[off];
        int8_t idx = findIndex(tid);
        if (idx < 0) continue;
        if (addr == REG_GOAL_POSITION && dlen == 2) reg_goal_position[idx] = p[off+1] | (p[off+2] << 8);
        else if (addr == REG_TORQUE_ENABLE && dlen == 1) reg_torque_enable[idx] = p[off+1];
        else if (addr == REG_GOAL_SPEED && dlen == 2) reg_goal_speed[idx] = p[off+1] | (p[off+2] << 8);
        else if (addr == REG_MIN_ANGLE_LIMIT && dlen == 2) reg_min_angle[idx] = p[off+1] | (p[off+2] << 8);
        else if (addr == REG_MAX_ANGLE_LIMIT && dlen == 2) reg_max_angle[idx] = p[off+1] | (p[off+2] << 8);
    }
}
void handleReset(uint8_t id) {
    int8_t idx = findIndex(id);
    if (idx < 0) return;
    reg_torque_enable[idx] = 0;
    reg_goal_position[idx] = 2048;
    reg_goal_speed[idx] = 0;
    reg_moving[idx] = 0;
    reg_min_angle[idx] = 0;
    reg_max_angle[idx] = 4095;
    sendStatus(id, 0x00);
}
// ═══════════════════════════════════════════════════════════════
// MAIN PARSER
// ═══════════════════════════════════════════════════════════════
void processPacket() {
    uint8_t cmd = pkt_data[0];
    uint8_t pcnt = pkt_length - 2; // ← FIXED: was -3
if (pkt_id == BROADCAST_ID) {
    switch (cmd) {
        case INST_SYNC_READ: handleSyncRead(pkt_data + 1, pcnt); return;
        case INST_SYNC_WRITE: handleSyncWrite(pkt_data + 1, pcnt); return;
        case INST_ACTION: sendStatus(BROADCAST_ID, 0x00); return;
        default: return;
    }
}
switch (cmd) {
    case INST_PING: if (findIndex(pkt_id) >= 0) sendPing(pkt_id); break;
    case INST_READ: handleRead(pkt_id, pkt_data + 1, pcnt); break;
    case INST_WRITE: handleWrite(pkt_id, pkt_data + 1, pcnt); break;
    case INST_REG_WRITE: handleWrite(pkt_id, pkt_data + 1, pcnt); break;
    case INST_ACTION: sendStatus(pkt_id, 0x00); break;
    case INST_RESET: handleReset(pkt_id); break;
    default: if (findIndex(pkt_id) >= 0) sendStatus(pkt_id, 0x02); break;
}
}
// ═══════════════════════════════════════════════════════════════
// SETUP & LOOP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Wire.begin();
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__)
    TWBR = 12; // 400 kHz
#else
    Wire.setClock(400000);
#endif
// Защита от зависания I2C-шины (TCA9548A/AS5600 не подключены, нет
// подтяжек, шина "залипла"): без этого Wire.endTransmission() может
// блокироваться НАВСЕГДА, и плата никогда не доходит до loop(), то есть
// не отвечает по serial вообще ни на один ID
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__) || defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(3000, true); // 3 мс таймаут, автосброс шины
#endif
pinMode(LED_PIN, OUTPUT);
digitalWrite(LED_PIN, LOW);
Serial.begin(BAUDRATE_FEE);
loadCalibration();
// Initial sensor read
for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t raw = readAS5600_Raw(i);
    if (raw != 0xFFFF) cached_positions[i] = applyCalibration(i, raw);
    delay(5);
}
digitalWrite(LED_PIN, HIGH); // Ready
}
void loop() {
    // Async sensor polling
    unsigned long us = micros();
    if (us - last_sensor_update >= 1000) {
        last_sensor_update = us;
        uint16_t raw = readAS5600_Raw(current_sensor);
        if (raw != 0xFFFF) {
            uint16_t newpos = applyCalibration(current_sensor, raw);
            // Экспоненциальное сглаживание (alpha=1/4) против собственного
            // шума AS5600 в ~1 младший бит (0.088°), который иначе виден
            // как постоянное подёргивание модели в 3D при неподвижной руке.
            // Учитываем переход через границу 0/4095 (шкала циклическая).
            int16_t diff = (int16_t)newpos - (int16_t)cached_positions[current_sensor];
            if (diff > 2048) diff -= 4096;
            if (diff < -2048) diff += 4096;
            int32_t filtered = (int32_t)cached_positions[current_sensor] + diff / 4;
            if (filtered < 0) filtered += 4096;
            if (filtered > 4095) filtered -= 4096;
            cached_positions[current_sensor] = (uint16_t)filtered;
        }
        current_sensor++;
        if (current_sensor >= NUM_JOINTS) current_sensor = 0;
    }
// Сторож парсера: если застряли в середине пакета дольше PARSER_TIMEOUT_MS
// (потерянный/повреждённый байт на линии), сбрасываем состояние, иначе
// парсер может зависнуть до следующего случайного совпадения по чек-сумме
if (parser_state != WAIT_FF1 && (millis() - last_parser_byte_ms > PARSER_TIMEOUT_MS)) {
    parser_state = WAIT_FF1;
    calc_checksum = 0;
}
// FeeTech packet parser
while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    last_parser_byte_ms = millis();
    switch (parser_state) {
        case WAIT_FF1:
            if (b == 0xFF) parser_state = WAIT_FF2;
            break;
        case WAIT_FF2:
            parser_state = (b == 0xFF) ? WAIT_ID : WAIT_FF1;
            break;
        case WAIT_ID:
            pkt_id = b; calc_checksum = b; parser_state = WAIT_LEN;
            break;
        case WAIT_LEN:
            pkt_length = b; calc_checksum += b; pkt_data_idx = 0;
            if (pkt_length >= 2 && pkt_length <= (sizeof(pkt_data) + 1)) parser_state = READ_DATA; // ← FIXED: +1 not +2
            else parser_state = WAIT_FF1;
            break;
        case READ_DATA:
            pkt_data[pkt_data_idx++] = b; calc_checksum += b;
            if (pkt_data_idx >= pkt_length - 1) parser_state = WAIT_CHKSUM;
            break;
        case WAIT_CHKSUM:
            if (b == ((~calc_checksum) & 0xFF)) processPacket();
            parser_state = WAIT_FF1; calc_checksum = 0;
            break;
    }
}
} 
