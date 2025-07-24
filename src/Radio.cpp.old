#include "Radio.hpp"

#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2A
#define REG_RSSI_WIDEBAND 0x2C
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERT_IQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42

#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05

#define IRQ_TX_DONE 0x08
#define IRQ_RX_DONE 0x40
#define IRQ_CRC_ERROR 0x10

RadioInterface::RadioInterface(uint8_t cs) : _cs(cs), _ldro(false), _limit_rate(false), _interference_detected(false), _avoid_interference(true), _difs_ms(10), _difs_wait_start(0), _cw_wait_start(0), _cw_wait_target(0), _cw_wait_passed(0), _csma_cw(-1), _cw_band(1), _cw_min(0), _cw_max(8), _noise_floor_sampled(false), _noise_floor_sample(0), _noise_floor(-292), _led_id_filter(0), _preamble_detected_at(0) {
  memset(_packet, 0, sizeof(_packet));
  memset(_noise_floor_buffer, 0, sizeof(_noise_floor_buffer));
}

bool RadioInterface::begin() {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  SPI.begin(LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI, LORA_SS);
  
  uint8_t version = SPI.transfer(REG_VERSION);
  if (version != 0x12) {
    return false;
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
  writeRegister(REG_LNA, 0x20 | 0x03); // Max gain, boost on
  writeRegister(REG_MODEM_CONFIG_3, _ldro ? 0x08 : 0x00);
  writeRegister(REG_DETECTION_OPTIMIZE, 0xC5);
  writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
  writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0: RX/TX done
  writeRegister(REG_DIO_MAPPING_2, 0x00);

  setFrequency(CFG_FREQUENCY);
  setBandwidth(CFG_BANDWIDTH);
  setSpreadingFactor(CFG_SF);
  setCodingRate(CFG_CR);
  setPreambleLength(CFG_PREAMBLE_LENGTH);
  setSyncWord(CFG_SYNC_WORD);
  setOutputPower(CFG_TX_POWER);

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  return true;
}

bool RadioInterface::transmit(uint8_t* data, size_t len) {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, len);

  for (size_t i = 0; i < len; i++) {
    writeRegister(REG_FIFO, data[i]);
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  
  while (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
    delay(1);
  }
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE);
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  return true;
}

int16_t RadioInterface::receive(uint8_t* data, size_t len) {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
  
  if (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE) {
    if (readRegister(REG_IRQ_FLAGS) & IRQ_CRC_ERROR) {
      writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE | IRQ_CRC_ERROR);
      return -1;
    }

    uint8_t rxBytes = readRegister(REG_RX_NB_BYTES);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    for (uint8_t i = 0; i < rxBytes && i < len; i++) {
      data[i] = readRegister(REG_FIFO);
    }
    writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);
    return rxBytes;
  }
  return 0;
}

void RadioInterface::setFrequency(float freq) {
  uint64_t frf = ((uint64_t)freq * 1000000) / 32000000;
  writeRegister(REG_FRF_MSB, (frf >> 16) & 0xFF);
  writeRegister(REG_FRF_MID, (frf >> 8) & 0xFF);
  writeRegister(REG_FRF_LSB, frf & 0xFF);
}

void RadioInterface::setBandwidth(float bw) {
  uint8_t bwValue;
  if (bw <= 7.8) bwValue = 0;
  else if (bw <= 10.4) bwValue = 1;
  else if (bw <= 15.6) bwValue = 2;
  else if (bw <= 20.8) bwValue = 3;
  else if (bw <= 31.25) bwValue = 4;
  else if (bw <= 41.7) bwValue = 5;
  else if (bw <= 62.5) bwValue = 6;
  else if (bw <= 125.0) bwValue = 7;
  else if (bw <= 250.0) bwValue = 8;
  else bwValue = 9;
  uint8_t reg = readRegister(REG_MODEM_CONFIG_1) & 0x0F;
  writeRegister(REG_MODEM_CONFIG_1, reg | (bwValue << 4));
}

void RadioInterface::setSpreadingFactor(uint8_t sf) {
  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;
  uint8_t reg = readRegister(REG_MODEM_CONFIG_2) & 0x0F;
  writeRegister(REG_MODEM_CONFIG_2, reg | (sf << 4));
}

void RadioInterface::setCodingRate(uint8_t cr) {
  if (cr < 5) cr = 5;
  if (cr > 8) cr = 8;
  uint8_t reg = readRegister(REG_MODEM_CONFIG_1) & 0xF1;
  writeRegister(REG_MODEM_CONFIG_1, reg | ((cr - 4) << 1));
}

void RadioInterface::setPreambleLength(uint16_t len) {
  writeRegister(REG_PREAMBLE_MSB, (len >> 8) & 0xFF);
  writeRegister(REG_PREAMBLE_LSB, len & 0xFF);
}

void RadioInterface::setSyncWord(uint8_t sw) {
  writeRegister(REG_SYNC_WORD, sw);
}

void RadioInterface::setOutputPower(int8_t power) {
  if (power > 17) power = 17;
  if (power < 2) power = 2;
  writeRegister(REG_PA_CONFIG, 0x80 | (power - 2));
}

uint8_t RadioInterface::readRegister(uint8_t addr) {
  digitalWrite(_cs, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t value = SPI.transfer(0);
  digitalWrite(_cs, HIGH);
  return value;
}

void RadioInterface::writeRegister(uint8_t addr, uint8_t value) {
  digitalWrite(_cs, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(_cs, HIGH);
}