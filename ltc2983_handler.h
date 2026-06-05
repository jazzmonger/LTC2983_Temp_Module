#pragma once
#include "esphome.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <cmath>

static const char *const TAG = "LTC2983";

// --------------------------------------------------------------------------
// DC2213A J3 external RTD connector (per schematic):
//   CH3 → RTDFH (pin 4)   CH7 → RTDSH (pin 3)
//   CH8 → RTDSL (pin 2)   CH9 → RTDFL (pin 1, unused at sensor)
//   LTC2983 assignment: CH8 = PT1000 4-wire Kelvin RSENSE, ptr CH3
//   Sensor wiring on J3 (matches schematic):
//     leg1 → RTDFH + RTDSH (pins 4+3 tied)
//     leg2 → RTDSL (pin 2); RTDFL (pin 1) left open
//   CH5 = onboard demo RTD — NOT J3; do not use for external sensor.
// --------------------------------------------------------------------------

#define LTC2983_COMMAND_STATUS_REG  0x0000
#define LTC2983_GLOBAL_CONFIG_REG   0x00F0
#define LTC2983_CH_ASSIGN_BASE      0x0200
#define LTC2983_DATA_BASE           0x0010
#define LTC2983_VOUT_BASE           0x0060   // measured voltage/resistance result (raw/1024)

#define LTC2983_CONVERSION_CONTROL  0x80     // Start=1, Done=0; OR with channel number

#define STATUS_DONE_MASK    0x40
#define STATUS_START_MASK   0x80

#define LTC2983_CS_PIN      GPIO_NUM_2
#define LTC2983_SPI_HOST    SPI2_HOST
#define LTC2983_SPI_CLK     GPIO_NUM_9
#define LTC2983_SPI_MOSI    GPIO_NUM_5
#define LTC2983_SPI_MISO    GPIO_NUM_4

#define CH_RSENSE     3
#define CH_RSENSE_PTR 3
#define CH_PT1000     8   // J3 external RTD (official DC2213 CH8 slot)

static spi_device_handle_t ltc2983_spi_device  = nullptr;
static bool ltc2983_initialized                = false;
static bool ltc2983_spi_bus_ready              = false;

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

static inline uint16_t ch_assign_addr(uint8_t ch) {
  return LTC2983_CH_ASSIGN_BASE + (uint16_t)(ch - 1) * 4;
}

static inline uint16_t ch_data_addr(uint8_t ch) {
  return LTC2983_DATA_BASE + (uint16_t)(ch - 1) * 4;
}

static inline uint16_t ch_vout_addr(uint8_t ch) {
  return LTC2983_VOUT_BASE + (uint16_t)(ch - 1) * 4;
}

static bool initialize_ltc2983() {
  if (ltc2983_initialized) return true;

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

  if (ltc2983_spi_device == nullptr) {
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 1000000;
    dev.mode           = 0;
    dev.spics_io_num   = -1;
    dev.queue_size     = 1;
    esp_err_t err = spi_bus_add_device(LTC2983_SPI_HOST, &dev, &ltc2983_spi_device);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
      return false;
    }
    ESP_LOGI(TAG, "SPI device added");
  }

  uint32_t tries = 0;
  while (true) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) {
      ESP_LOGI(TAG, "Chip ready (status=0x%02x)", st);
      break;
    }
    delay(10);
    if (++tries > 300) {
      ESP_LOGE(TAG, "Chip not ready — status=0x%02x", st);
      return false;
    }
  }

  // CH3: onboard 2 kΩ sense resistor
  //   type  bits 31:27 = 0x1D (Sense Resistor)
  //   value = 2000 × 1024 = 0x1F4000 (matches DC2213A: SENSE_RESISTOR_VALUE 0x1F4000)
  uint32_t rsense_cfg = ((uint32_t)0x1D << 27) | 0x001F4000UL;
  write_dword(ch_assign_addr(CH_RSENSE), rsense_cfg);
  uint32_t rsense_rb = read_dword(ch_assign_addr(CH_RSENSE));
  ESP_LOGI(TAG, "CH%d RSENSE  written=0x%08" PRIX32 "  readback=0x%08" PRIX32 "%s",
           CH_RSENSE, rsense_cfg, rsense_rb,
           (rsense_rb == rsense_cfg) ? " OK" : " *** MISMATCH ***");

  // Clear onboard demo channels — we only want J3 external on CH8.
  write_dword(ch_assign_addr(5),  0x00000000UL);
  write_dword(ch_assign_addr(11), 0x00000000UL);

  // CH8: PT1000, 4-wire Kelvin RSENSE, ptr CH3 (official DC2213 CH8 topology).
  uint32_t pt1000_cfg = ((uint32_t)0x0F << 27)
                      | ((uint32_t)CH_RSENSE_PTR << 22)
                      | (0x3UL << 20) | (0x2UL << 18) | (0x6UL << 14) | (0x0UL << 12);
  write_dword(ch_assign_addr(CH_PT1000), pt1000_cfg);
  uint32_t pt1000_rb = read_dword(ch_assign_addr(CH_PT1000));
  ESP_LOGI(TAG, "CH%d PT1000 (J3) written=0x%08" PRIX32 " readback=0x%08" PRIX32 "%s",
           CH_PT1000, pt1000_cfg, pt1000_rb,
           (pt1000_rb == pt1000_cfg) ? " OK" : " MISMATCH");

  write_byte(LTC2983_GLOBAL_CONFIG_REG, 0x00);

  // MUX configuration delay (0x0FF): DC2213A official config uses 0 (no extra delay).
  write_byte(0x00FF, 0x00);

  ltc2983_initialized = true;
  ESP_LOGI(TAG, "LTC2983 ready — CH8=J3 external PT1000  CH3=RSENSE(2kΩ)");
  return true;
}

static float parse_temperature(uint32_t raw) {
  if (raw & (1UL << 31)) {
    ESP_LOGW(TAG, "Hard fault raw=0x%08" PRIX32 " — check wiring", raw);
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
  int32_t t = (int32_t)(raw & 0x00FFFFFFUL);
  if (t & 0x00800000L) t |= (int32_t)0xFF000000;
  return (float)t / 1024.0f;
}

static bool wait_conversion_done() {
  delay(20);
  for (uint32_t i = 0; i < 200; ++i) {
    uint8_t st = read_byte(LTC2983_COMMAND_STATUS_REG);
    if ((st & STATUS_START_MASK) == 0 && (st & STATUS_DONE_MASK) != 0) return true;
    delay(5);
  }
  return false;
}

static void log_rtd_channel(uint8_t ch) {
  uint32_t raw  = read_dword(ch_data_addr(ch));
  uint8_t  flt  = (uint8_t)(raw >> 24);
  float    temp = parse_temperature(raw);
  int32_t  vout = (int32_t)read_dword(ch_vout_addr(ch));
  float    ohms = (float)vout / 1024.0f;
  if (std::isnan(temp))
    ESP_LOGI(TAG, "J3 CH%2d: flt=0x%02X R=%7.1fΩ temp=nan", ch, flt, ohms);
  else
    ESP_LOGI(TAG, "J3 CH%2d: flt=0x%02X R=%7.1fΩ temp=%.2fC", ch, flt, ohms, temp);
}

inline void read_ltc2983_single_rtd(
    esphome::template_::TemplateSensor *rtd) {

  if (!initialize_ltc2983()) return;

  write_byte(LTC2983_COMMAND_STATUS_REG, LTC2983_CONVERSION_CONTROL | CH_PT1000);
  if (!wait_conversion_done()) {
    ESP_LOGE(TAG, "Conversion timed out");
    return;
  }

  log_rtd_channel(CH_PT1000);

  float temp = parse_temperature(read_dword(ch_data_addr(CH_PT1000)));
  if (!std::isnan(temp)) rtd->publish_state(temp);
}
