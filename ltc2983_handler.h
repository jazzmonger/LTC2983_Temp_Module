#pragma once
#include "esphome.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <cmath>
#include <cstring>

static const char *const TAG = "LTC2983";

// Independent 2-wire PT1000s (any one open leaves the others valid):
//   CH1 = RSENSE return (→GND)
//   CH2 = 2 kΩ RSENSE (shared excitation hub / COM)
//   Odd channels CH3/5/7/9/11 hard-tied to CH2 (not assigned as sensors)
//   Even sense channels — each RTD wired between that CH and the prior odd hub:
//     RTD1: CH4 ↔ CH3(=COM)   read CH4
//     RTD2: CH6 ↔ CH5(=COM)   read CH6
//     RTD3: CH8 ↔ CH7(=COM)   read CH8
//     RTD4: CH10 ↔ CH9(=COM)  read CH10
//     RTD5: CH12 ↔ CH11(=COM) read CH12
//   K-type TCs single-ended (TC− → COM): TC+ on CH13–CH17
//   CJC: 2N3906 diode on CH20 — emitter→CH20, base+collector→COM
//
// Conversion: multi-channel mask (0x00F4–0x00F7) + start 0x80 — one wait for all.

#define LTC2983_COMMAND_STATUS_REG  0x0000
#define LTC2983_GLOBAL_CONFIG_REG   0x00F0
#define LTC2983_MULT_CH_MASK_BASE   0x00F4
#define LTC2983_MUX_CONFIG_REG      0x00FF
#define LTC2983_CH_ASSIGN_BASE      0x0200
#define LTC2983_DATA_BASE           0x0010

#define STATUS_DONE_MASK    0x40
#define STATUS_START_MASK   0x80
#define CONV_START_MULTI    0x80  // B7=1, B4:0=00000 → multi-channel

// Shared SPI2 with MCP3208 (ESPHome spi_bus in packages/io_gpio.yaml). CS is GPIO bit-bang.
#define LTC2983_CS_PIN      GPIO_NUM_2
#define LTC2983_SPI_HOST    SPI2_HOST
#define LTC2983_SPI_CLK     GPIO_NUM_9
#define LTC2983_SPI_MOSI    GPIO_NUM_5
#define LTC2983_SPI_MISO    GPIO_NUM_4
#define LTC2983_SPI_HZ      4000000

#define CH_RSENSE   2
#define CH_RTD_1    4
#define CH_RTD_2    6
#define CH_RTD_3    8
#define CH_RTD_4   10
#define CH_RTD_5   12
#define CH_TC_1    13
#define CH_TC_2    14
#define CH_TC_3    15
#define CH_TC_4    16
#define CH_TC_5    17
#define CH_DIODE_P 20

#define NUM_RTD  5
#define NUM_TC   5

#define CFG_RSENSE ((uint32_t)0x1D << 27) | 0x001F4000UL
#define CFG_RTD_PT1000 ((uint32_t)0x0F << 27) | ((uint32_t)CH_RSENSE << 22) \
                       | (0x1UL << 18) | (0x5UL << 14)
// Independent pairs: minimal extra settle (base 1 ms + 2×100 µs)
#define MUX_SETTLE_LSB  2
#define CFG_DIODE_CJC ((uint32_t)0x1C << 27) | (7UL << 24) | (3UL << 22)
#define CFG_TC_K_SE ((uint32_t)0x02 << 27) | ((uint32_t)CH_DIODE_P << 22) \
                     | (1UL << 21) | (0x1UL << 20)

static constexpr uint8_t RTD_CHANNELS[NUM_RTD] = {
    CH_RTD_1, CH_RTD_2, CH_RTD_3, CH_RTD_4, CH_RTD_5,
};
static constexpr uint8_t TC_CHANNELS[NUM_TC] = {
    CH_TC_1, CH_TC_2, CH_TC_3, CH_TC_4, CH_TC_5,
};
static constexpr float TC_TEMP_MIN_C = -200.0f;
static constexpr float TC_TEMP_MAX_C = 1370.0f;

enum FaultCode : uint8_t {
  FAULT_OK = 0,
  FAULT_OPEN,
  FAULT_SHORT_VDD,
  FAULT_OVERRANGE,
  FAULT_UNDERRANGE,
  FAULT_HARD,
  FAULT_TIMEOUT,
  FAULT_INVALID,
};

static const char *const FAULT_TEXT[] = {
    "OK",
    "Open Circuit",
    "Short to VDD",
    "Overrange",
    "Underrange",
    "Hard Failure",
    "General Fault",
    "Invalid",
};

struct LtcSensors {
  esphome::template_::TemplateSensor *rtd[NUM_RTD];
  esphome::template_::TemplateSensor *tc[NUM_TC];
  esphome::text_sensor::TextSensor *fault_rtd[NUM_RTD];
  esphome::text_sensor::TextSensor *fault_tc[NUM_TC];
};

static spi_device_handle_t ltc2983_spi_device = nullptr;
static bool ltc2983_initialized = false;
static bool ltc2983_spi_bus_ready = false;
static bool ltc2983_absent = false;
static uint32_t ltc2983_next_init_ms = 0;
static bool ltc_conv_pending = false;
static uint32_t ltc_conv_start_ms = 0;
static uint8_t conversion_mask_[4] = {};

static void ltc2983_cs_low()  { gpio_set_level(LTC2983_CS_PIN, 0); }
static void ltc2983_cs_high() { gpio_set_level(LTC2983_CS_PIN, 1); }

static bool spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) {
  spi_transaction_t t = {};
  t.length    = len * 8;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  return spi_device_polling_transmit(ltc2983_spi_device, &t) == ESP_OK;
}

static void write_byte(uint16_t reg, uint8_t data) {
  uint8_t tx[4] = {0x02, (uint8_t)(reg >> 8), (uint8_t)reg, data};
  ltc2983_cs_low();
  spi_xfer(tx, nullptr, 4);
  ltc2983_cs_high();
}

static void write_dword(uint16_t reg, uint32_t data) {
  uint8_t tx[7] = {
      0x02, (uint8_t)(reg >> 8), (uint8_t)reg,
      (uint8_t)(data >> 24), (uint8_t)(data >> 16),
      (uint8_t)(data >> 8), (uint8_t)data,
  };
  ltc2983_cs_low();
  spi_xfer(tx, nullptr, 7);
  ltc2983_cs_high();
}

static uint8_t read_byte(uint16_t reg) {
  uint8_t tx[4] = {0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00};
  uint8_t rx[4] = {};
  ltc2983_cs_low();
  spi_xfer(tx, rx, 4);
  ltc2983_cs_high();
  return rx[3];
}

static uint32_t read_dword(uint16_t reg) {
  uint8_t tx[7] = {0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0, 0, 0, 0};
  uint8_t rx[7] = {};
  ltc2983_cs_low();
  spi_xfer(tx, rx, 7);
  ltc2983_cs_high();
  return ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
         ((uint32_t)rx[5] << 8) | (uint32_t)rx[6];
}

static inline uint16_t ch_assign_addr(uint8_t ch) {
  return LTC2983_CH_ASSIGN_BASE + (uint16_t)(ch - 1) * 4;
}

static inline uint16_t ch_data_addr(uint8_t ch) {
  return LTC2983_DATA_BASE + (uint16_t)(ch - 1) * 4;
}

// Datasheet Table 65: 0x0F4 reserved; 0x0F5 B7..B4 = CH20..17;
// 0x0F6 B7..B0 = CH16..9; 0x0F7 B7..B0 = CH8..1
static void mask_set_channel(uint8_t mask[4], uint8_t ch) {
  if (ch < 1 || ch > 20) return;
  if (ch <= 8)
    mask[3] |= (uint8_t)(1u << (ch - 1));
  else if (ch <= 16)
    mask[2] |= (uint8_t)(1u << (ch - 9));
  else
    mask[1] |= (uint8_t)(1u << (ch - 13));
}

static void build_conversion_mask() {
  memset(conversion_mask_, 0, 4);
  for (uint8_t ch : RTD_CHANNELS) mask_set_channel(conversion_mask_, ch);
  for (uint8_t ch : TC_CHANNELS) mask_set_channel(conversion_mask_, ch);
  // CH20 diode is not in the mask — each TC conversion measures CJC via its
  // assignment-word pointer (saves one full ADC cycle ≈ 170 ms).
}

static bool initialize_ltc2983() {
  if (ltc2983_initialized) return true;
  // UART duplex dies if we block ~3 s every 2 s when the chip is missing — back off.
  if (ltc2983_absent && (int32_t) (millis() - ltc2983_next_init_ms) < 0)
    return false;

  gpio_config_t cs_cfg = {};
  cs_cfg.pin_bit_mask  = (1ULL << LTC2983_CS_PIN);
  cs_cfg.mode          = GPIO_MODE_OUTPUT;
  cs_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
  cs_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  cs_cfg.intr_type     = GPIO_INTR_DISABLE;
  gpio_config(&cs_cfg);
  ltc2983_cs_high();

  if (!ltc2983_spi_bus_ready) {
    spi_bus_config_t bus = {};
    bus.mosi_io_num     = LTC2983_SPI_MOSI;
    bus.miso_io_num     = LTC2983_SPI_MISO;
    bus.sclk_io_num     = LTC2983_SPI_CLK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = 32;
    esp_err_t err = spi_bus_initialize(LTC2983_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
      ltc2983_absent = true;
      ltc2983_next_init_ms = millis() + 30000;
      return false;
    }
    ltc2983_spi_bus_ready = true;
  }

  if (ltc2983_spi_device == nullptr) {
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = LTC2983_SPI_HZ;  // 4 MHz — same as temp-module demo
    dev.mode           = 0;
    dev.spics_io_num   = -1;
    dev.queue_size     = 1;
    esp_err_t err = spi_bus_add_device(LTC2983_SPI_HOST, &dev, &ltc2983_spi_device);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
      ltc2983_absent = true;
      ltc2983_next_init_ms = millis() + 30000;
      return false;
    }
  }

  // Status 0x80 = start-up; 0x40 = ready. Dead bus = 0x00/0xFF → fail fast for UART.
  uint32_t tries = 0;
  uint8_t st = 0;
  while (true) {
    st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) break;
    if (st == 0x00 || st == 0xFF) {
      if (++tries > 20) {
        ESP_LOGW(TAG, "Chip not ready — status=0x%02x (no SPI; retry in 30s)", st);
        ltc2983_absent = true;
        ltc2983_next_init_ms = millis() + 30000;
        return false;
      }
    } else if (++tries > 300) {  // ~3 s like temp-module
      ESP_LOGW(TAG, "Chip not ready — status=0x%02x (retry in 30s)", st);
      ltc2983_absent = true;
      ltc2983_next_init_ms = millis() + 30000;
      return false;
    }
    delay(10);
  }

  for (uint8_t ch = 1; ch <= 20; ++ch)
    write_dword(ch_assign_addr(ch), 0);

  write_dword(ch_assign_addr(CH_RSENSE), CFG_RSENSE);
  for (uint8_t ch : RTD_CHANNELS)
    write_dword(ch_assign_addr(ch), CFG_RTD_PT1000);
  for (uint8_t ch : TC_CHANNELS)
    write_dword(ch_assign_addr(ch), CFG_TC_K_SE);
  write_dword(ch_assign_addr(CH_DIODE_P), CFG_DIODE_CJC);
  write_byte(LTC2983_MUX_CONFIG_REG, MUX_SETTLE_LSB);
  write_byte(LTC2983_GLOBAL_CONFIG_REG, 0x00);

  build_conversion_mask();
  for (uint8_t i = 0; i < 4; ++i)
    write_byte(LTC2983_MULT_CH_MASK_BASE + i, conversion_mask_[i]);

  ltc2983_initialized = true;
  ltc2983_absent = false;
  ESP_LOGI(TAG, "ready — RTD %d/%d/%d/%d/%d TC %d–%d CJC %d mask=%02X%02X%02X%02X SPI=%dHz",
           CH_RTD_1, CH_RTD_2, CH_RTD_3, CH_RTD_4, CH_RTD_5,
           CH_TC_1, CH_TC_5, CH_DIODE_P,
           conversion_mask_[0], conversion_mask_[1], conversion_mask_[2], conversion_mask_[3],
           LTC2983_SPI_HZ);
  return true;
}

static float parse_temperature(uint32_t raw) {
  if (!(raw & (1UL << 24))) return NAN;
  int32_t t = (int32_t)(raw & 0x00FFFFFFUL);
  if (t & 0x00800000L) t |= (int32_t)0xFF000000;
  return (float)t / 1024.0f;
}

static FaultCode decode_fault(uint32_t raw) {
  if (!(raw & (1UL << 24))) return FAULT_INVALID;
  uint8_t fb = (raw >> 24) & 0xFF;
  bool hard      = (fb >> 7) & 1;
  bool overrange = (fb >> 6) & 1;
  bool open      = (fb >> 5) & 1;
  bool shrt_vdd  = (fb >> 4) & 1;
  if (!hard && !overrange && !open && !shrt_vdd) return FAULT_OK;

  int32_t temp_raw = (int32_t)(raw & 0x00FFFFFFUL);
  if (temp_raw & 0x00800000L) temp_raw |= (int32_t)0xFF000000;

  if (shrt_vdd) return FAULT_SHORT_VDD;
  if (open || ((hard || overrange) && temp_raw < -100000)) return FAULT_OPEN;
  if (overrange) return (temp_raw > 0) ? FAULT_OVERRANGE : FAULT_UNDERRANGE;
  if (hard) return FAULT_HARD;
  return FAULT_OK;
}

static bool should_publish(FaultCode f) {
  return f != FAULT_OPEN && f != FAULT_SHORT_VDD && f != FAULT_HARD &&
         f != FAULT_TIMEOUT && f != FAULT_INVALID;
}

static bool wait_conversion_done(uint32_t timeout_ms = 3000) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) return true;
    delay(2);
  }
  return false;
}

static void publish_channel(uint8_t ch, esphome::template_::TemplateSensor *sensor,
                            esphome::text_sensor::TextSensor *fault_sensor,
                            bool is_tc) {
  uint32_t raw = read_dword(ch_data_addr(ch));
  FaultCode fault = decode_fault(raw);
  float temp = parse_temperature(raw);

  if (fault != FAULT_OK)
    ESP_LOGW(TAG, "CH%d: raw=0x%08X fault=%s", ch, raw, FAULT_TEXT[fault]);

  bool ok = should_publish(fault) && !std::isnan(temp);
  if (is_tc)
    ok = ok && temp > TC_TEMP_MIN_C && temp < TC_TEMP_MAX_C;

  if (sensor) sensor->publish_state(ok ? temp : NAN);
  if (fault_sensor) fault_sensor->publish_state(FAULT_TEXT[fault]);
}

// Non-blocking for UART: start conversion and return; next call publishes when DONE.
// (Blocking ~2 s waits starved packet_transport → display "RX only" / flaky TX.)
inline void read_ltc2983_sensors(const LtcSensors &s) {
  if (!initialize_ltc2983()) return;

  if (!ltc_conv_pending) {
    write_byte(LTC2983_COMMAND_STATUS_REG, CONV_START_MULTI);
    ltc_conv_pending = true;
    ltc_conv_start_ms = millis();
    return;
  }

  uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
  const bool done =
      (st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0;
  if (!done) {
    if ((millis() - ltc_conv_start_ms) > 3500) {
      ESP_LOGE(TAG, "multi-channel conversion timed out (status=0x%02x)", st);
      ltc_conv_pending = false;
      ltc2983_initialized = false;
      ltc2983_absent = true;
      ltc2983_next_init_ms = millis() + 10000;
      for (int i = 0; i < NUM_RTD; ++i) {
        if (s.rtd[i]) s.rtd[i]->publish_state(NAN);
        if (s.fault_rtd[i]) s.fault_rtd[i]->publish_state(FAULT_TEXT[FAULT_TIMEOUT]);
      }
      for (int i = 0; i < NUM_TC; ++i) {
        if (s.tc[i]) s.tc[i]->publish_state(NAN);
        if (s.fault_tc[i]) s.fault_tc[i]->publish_state(FAULT_TEXT[FAULT_TIMEOUT]);
      }
    }
    return;
  }

  for (int i = 0; i < NUM_RTD; ++i)
    publish_channel(RTD_CHANNELS[i], s.rtd[i], s.fault_rtd[i], false);
  for (int i = 0; i < NUM_TC; ++i)
    publish_channel(TC_CHANNELS[i], s.tc[i], s.fault_tc[i], true);

  // Start next conversion immediately; do not block waiting (keeps UART free).
  write_byte(LTC2983_COMMAND_STATUS_REG, CONV_START_MULTI);
  ltc_conv_pending = true;
  ltc_conv_start_ms = millis();
}
