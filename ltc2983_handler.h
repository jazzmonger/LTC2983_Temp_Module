#pragma once
#include "esphome.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <cmath>

static const char *const TAG = "LTC2983";

// --------------------------------------------------------------------------
// DC2210A Breakout — discrete 2kΩ 1% RSENSE + single 2-wire PT1000
//   CH1  = 2 kΩ RSENSE (external 1% resistor between CH1 and CH2)
//   CH2  = PT1000, 2-wire, 25 µA excitation, European curve, Rsense → CH1
//   CH3  = bottom of PT1000 (implicitly used; no assignment needed)
//
//   LTC2983 current chain (low CH# = source/top of stack):
//     CH1 → [2kΩ RSENSE] → CH2 → [PT1000] → CH3
// --------------------------------------------------------------------------

#define LTC2983_COMMAND_STATUS_REG  0x0000
#define LTC2983_GLOBAL_CONFIG_REG   0x00F0
#define LTC2983_CH_ASSIGN_BASE      0x0200
#define LTC2983_DATA_BASE           0x0010

// Command byte: start conversion on all channels
#define LTC2983_START_ALL   0x80
// Status masks
#define STATUS_DONE_MASK    0x40
#define STATUS_START_MASK   0x80

// SPI pins — match the ESP32-S3 board wiring
#define LTC2983_CS_PIN      GPIO_NUM_2
#define LTC2983_SPI_HOST    SPI2_HOST
#define LTC2983_SPI_CLK     GPIO_NUM_9
#define LTC2983_SPI_MOSI    GPIO_NUM_5
#define LTC2983_SPI_MISO    GPIO_NUM_4

// Channel numbers (1-based)
#define CH_RSENSE   1
#define CH_PT1000   2
#define CH_TC_K     5   // K-type TC positive (+) on CH5, negative (−) auto on CH6

static spi_device_handle_t ltc2983_spi_device  = nullptr;
static bool ltc2983_initialized                = false;
static bool ltc2983_spi_bus_ready              = false;

// --------------- low-level SPI helpers ---------------

static void ltc2983_cs_low()  { gpio_set_level(LTC2983_CS_PIN, 0); }
static void ltc2983_cs_high() { gpio_set_level(LTC2983_CS_PIN, 1); }

static uint8_t ltc2983_spi_transfer(uint8_t data) {
  uint8_t rx = 0;
  spi_transaction_t t = {};
  t.length    = 8;
  t.tx_buffer = &data;
  t.rx_buffer = &rx;
  spi_device_polling_transmit(ltc2983_spi_device, &t);
  return rx;
}

static void write_byte(uint16_t reg, uint8_t data) {
  ltc2983_cs_low();
  ltc2983_spi_transfer(0x02);
  ltc2983_spi_transfer((reg >> 8) & 0xFF);
  ltc2983_spi_transfer( reg       & 0xFF);
  ltc2983_spi_transfer(data);
  ltc2983_cs_high();
}

static void write_dword(uint16_t reg, uint32_t data) {
  ltc2983_cs_low();
  ltc2983_spi_transfer(0x02);
  ltc2983_spi_transfer((reg >> 8) & 0xFF);
  ltc2983_spi_transfer( reg       & 0xFF);
  ltc2983_spi_transfer((data >> 24) & 0xFF);
  ltc2983_spi_transfer((data >> 16) & 0xFF);
  ltc2983_spi_transfer((data >>  8) & 0xFF);
  ltc2983_spi_transfer( data        & 0xFF);
  ltc2983_cs_high();
}

static uint8_t read_byte(uint16_t reg) {
  ltc2983_cs_low();
  ltc2983_spi_transfer(0x03);
  ltc2983_spi_transfer((reg >> 8) & 0xFF);
  ltc2983_spi_transfer( reg       & 0xFF);
  uint8_t val = ltc2983_spi_transfer(0x00);
  ltc2983_cs_high();
  return val;
}

static uint32_t read_dword(uint16_t reg) {
  ltc2983_cs_low();
  ltc2983_spi_transfer(0x03);
  ltc2983_spi_transfer((reg >> 8) & 0xFF);
  ltc2983_spi_transfer( reg       & 0xFF);
  uint32_t val = 0;
  val |= ((uint32_t)ltc2983_spi_transfer(0x00)) << 24;
  val |= ((uint32_t)ltc2983_spi_transfer(0x00)) << 16;
  val |= ((uint32_t)ltc2983_spi_transfer(0x00)) <<  8;
  val |= ((uint32_t)ltc2983_spi_transfer(0x00));
  ltc2983_cs_high();
  return val;
}

// --------------- channel address helpers ---------------

static inline uint16_t ch_assign_addr(uint8_t ch) {
  return LTC2983_CH_ASSIGN_BASE + (uint16_t)(ch - 1) * 4;
}

static inline uint16_t ch_data_addr(uint8_t ch) {
  return LTC2983_DATA_BASE + (uint16_t)(ch - 1) * 4;
}

// --------------- one-time chip init ---------------

static bool initialize_ltc2983() {
  if (ltc2983_initialized) return true;

  // Configure CS pin
  gpio_config_t cs_cfg = {};
  cs_cfg.pin_bit_mask  = (1ULL << LTC2983_CS_PIN);
  cs_cfg.mode          = GPIO_MODE_OUTPUT;
  cs_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
  cs_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  cs_cfg.intr_type     = GPIO_INTR_DISABLE;
  gpio_config(&cs_cfg);
  ltc2983_cs_high();

  // Init SPI bus
  if (!ltc2983_spi_bus_ready) {
    spi_bus_config_t bus = {};
    bus.mosi_io_num   = LTC2983_SPI_MOSI;
    bus.miso_io_num   = LTC2983_SPI_MISO;
    bus.sclk_io_num   = LTC2983_SPI_CLK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 32;

    esp_err_t err = spi_bus_initialize(LTC2983_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
      return false;
    }
    ltc2983_spi_bus_ready = true;
    ESP_LOGI(TAG, "SPI bus ready (CLK=%d MOSI=%d MISO=%d CS=%d)",
             LTC2983_SPI_CLK, LTC2983_SPI_MOSI, LTC2983_SPI_MISO, LTC2983_CS_PIN);
  }

  // Add SPI device
  if (ltc2983_spi_device == nullptr) {
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 1000000;  // 1 MHz (LTC2983 max 2 MHz)
    dev.mode           = 0;        // CPOL=0 CPHA=0
    dev.spics_io_num   = -1;       // manual CS
    dev.queue_size     = 1;

    esp_err_t err = spi_bus_add_device(LTC2983_SPI_HOST, &dev, &ltc2983_spi_device);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
      return false;
    }
    ESP_LOGI(TAG, "SPI device added");
  }

  // Wait for chip POR (Done bit set, Start bit clear)
  uint32_t tries = 0;
  while (true) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) {
      ESP_LOGI(TAG, "Chip ready after POR (status=0x%02x)", st);
      break;
    }
    delay(10);
    if (++tries > 300) {
      ESP_LOGE(TAG, "Chip not ready — status=0x%02x  Check SPI wiring / power",
               read_byte(LTC2983_COMMAND_STATUS_REG));
      return false;
    }
  }

  // ---- Program channels ----

  // CH1: 2 kΩ 1% external sense resistor (discrete, between CH1 and CH2)
  //   Sensor type = 0x1D (29 = sense resistor)  bits 31:27
  //   Value = 2000 Ω → 2000 * 1024 = 2048000 = 0x1F4000  bits 26:0
  uint32_t rsense_cfg = ((uint32_t)0x1D << 27) | 0x001F4000UL;
  write_dword(ch_assign_addr(CH_RSENSE), rsense_cfg);
  uint32_t rsense_rb = read_dword(ch_assign_addr(CH_RSENSE));
  ESP_LOGI(TAG, "CH%d RSENSE  written=0x%08" PRIX32 "  readback=0x%08" PRIX32
           "%s", CH_RSENSE, rsense_cfg, rsense_rb,
           (rsense_rb == rsense_cfg) ? " OK" : " *** MISMATCH - SPI write failed ***");

  // CH2: PT1000 — 2-wire, 25 µA, European (IEC 751), Rsense pointer = CH1
  //   Standard 2-wire mode valid here because CHRSENSE(1) < CHRTD(2).
  //   bits 31:27  sensor type  = 0x0F (PT-1000)
  //   bits 26:22  Rsense ptr   = CH1 (1)
  //   bits 21:18  wire config  = 0x00 (2-wire)
  //   bits 17:14  excitation   = 0x05 (500 µA — large voltages, easy to probe)
  //   bits 13:12  curve        = 0x00 (European / IEC 751)
  uint32_t pt1000_cfg = ((uint32_t)0x0F << 27)
                      | ((uint32_t)CH_RSENSE << 22)
                      | (0x00UL << 18)
                      | (0x05UL << 14)
                      | (0x00UL << 12);
  write_dword(ch_assign_addr(CH_PT1000), pt1000_cfg);
  uint32_t pt1000_rb = read_dword(ch_assign_addr(CH_PT1000));
  ESP_LOGI(TAG, "CH%d PT1000  written=0x%08" PRIX32 "  readback=0x%08" PRIX32
           "%s", CH_PT1000, pt1000_cfg, pt1000_rb,
           (pt1000_rb == pt1000_cfg) ? " OK" : " *** MISMATCH - SPI write failed ***");

  // CH3: MUST remain unassigned — it is the current-return terminal for the
  //      2-wire PT1000 on CH2.  Writing any assignment here severs the RTD
  //      return path and causes a 0.000 °C (zero-differential) phantom reading.
  write_dword(ch_assign_addr(3), 0x00000000UL);

  // CH5: K-type thermocouple (CH5=+, CH6=- auto)
  //   CJC pointer = CH2 (PT1000) for reference; even if CH2 faults, TC may still return data
  //   bits 31:27 = 0x02 (K-type), bits 26:22 = CJC ch (2), bit 21=0 (diff),
  //   bits 20:19 = 0x01 (open-circuit check), bits 18:16 = 0 (no extra delay)
  uint32_t tc_cfg = ((uint32_t)0x02 << 27)
                  | ((uint32_t)CH_PT1000 << 22)
                  | (0x01UL << 19);
  write_dword(ch_assign_addr(CH_TC_K), tc_cfg);
  uint32_t tc_rb = read_dword(ch_assign_addr(CH_TC_K));
  ESP_LOGI(TAG, "CH%d TC-K    written=0x%08" PRIX32 "  readback=0x%08" PRIX32
           "%s", CH_TC_K, tc_cfg, tc_rb,
           (tc_rb == tc_cfg) ? " OK" : " *** MISMATCH ***");

  // Global config: °C, 50/60 Hz rejection
  write_byte(LTC2983_GLOBAL_CONFIG_REG, 0x00 | 0x00);  // °C + 50/60 Hz

  ltc2983_initialized = true;
  ESP_LOGI(TAG, "LTC2983 channel map loaded (CH1=RSENSE, CH2=PT1000, CH5=TC-K+, CH6=TC-K-)");
  return true;
}

// --------------- parse result word ---------------

static float parse_temperature(uint32_t raw) {
  // LTC2983 result word (RTD type):
  //   Bit 31: Hard fault (open/short on RTD or RSENSE) — result = -999 °C sentinel
  //   Bit 30: Soft fault (below -200 °C range)
  //   Bits 29:25: Additional fault flags
  //   Bit 24: Valid data flag (1 = conversion ran; always set even on faults)
  //   Bits 23:0: Signed temperature × 1024 (2's complement, LSB = 1/1024 °C)
  if (raw & (1UL << 31)) {
    ESP_LOGW(TAG, "Hard fault (open/short) raw=0x%08" PRIX32
             " — check probe wires and RSENSE connection", raw);
    return NAN;
  }
  if (raw & (1UL << 30)) {
    ESP_LOGW(TAG, "Soft fault (out of range) raw=0x%08" PRIX32, raw);
    return NAN;
  }
  if (!(raw & (1UL << 24))) {
    ESP_LOGW(TAG, "Data-valid bit not set: 0x%08" PRIX32, raw);
    return NAN;
  }
  // Temperature in bits 23:0, signed, LSB = 1/1024 °C
  int32_t t = (int32_t)(raw & 0x00FFFFFFUL);
  if (t & 0x00800000L) t |= (int32_t)0xFF000000;  // sign-extend
  return (float)t / 1024.0f;
}

// --------------- public poll function ---------------
// YAML calls: read_ltc2983_single_rtd(id(rtd_1))

inline void read_ltc2983_single_rtd(
    esphome::template_::TemplateSensor *rtd) {

  ESP_LOGD(TAG, "Poll start");

  if (!initialize_ltc2983()) {
    return;
  }

  // Start ALL assigned channels — gives stable CH2 readings
  delay(5);  // brief settle before start
  write_byte(LTC2983_COMMAND_STATUS_REG, 0x80);
  delay(5);  // give chip time to assert Start bit

  // Wait for conversion to COMPLETE (Done=1, Start=0)
  uint32_t timeout = 0;
  while (true) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) break;
    delay(10);
    if (++timeout > 1000) {
      ESP_LOGE(TAG, "Conversion timed out (status=0x%02x)", st);
      return;
    }
  }
  ESP_LOGD(TAG, "Conversion complete (start-all)");

  // Read CH1 (RSENSE) result for diagnostics
  uint32_t rsense_raw = read_dword(ch_data_addr(CH_RSENSE));
  ESP_LOGI(TAG, "CH%d RSENSE result raw=0x%08" PRIX32, CH_RSENSE, rsense_raw);

  uint32_t raw = read_dword(ch_data_addr(CH_PT1000));
  float temp   = parse_temperature(raw);
  ESP_LOGI(TAG, "CH%d raw=0x%08" PRIX32 "  temp=%.3f C", CH_PT1000, raw, temp);
  rtd->publish_state(temp);

  // --- Now trigger TC-K on CH5 independently ---
  write_byte(LTC2983_COMMAND_STATUS_REG, 0x80 | CH_TC_K);
  uint32_t tc_timeout = 0;
  // wait for start
  while (!(read_byte(LTC2983_COMMAND_STATUS_REG) & STATUS_START_MASK)) {
    delay(1); if (++tc_timeout > 50) { ESP_LOGE(TAG, "TC conv never started"); goto tc_done; }
  }
  tc_timeout = 0;
  // wait for done
  while (true) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) break;
    delay(10); if (++tc_timeout > 500) { ESP_LOGE(TAG, "TC conv timed out"); goto tc_done; }
  }
  {
    uint32_t tc_raw  = read_dword(ch_data_addr(CH_TC_K));
    float    tc_temp = parse_temperature(tc_raw);
    ESP_LOGI(TAG, "CH%d TC-K   raw=0x%08" PRIX32 "  temp=%.3f C", CH_TC_K, tc_raw, tc_temp);
  }
  tc_done:;
}
