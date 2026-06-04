//https://www.mouser.com/ProductDetail/Analog-Devices/DC2296A?qs=ytflclh7QUW6BFI5eF8mpA%3D%3D
// $172.00
// LTC2983 channel configuration: Physical Layout
      //     CH1 -----------------+

      //                          |
      //                       [2kΩ Rsense]
      //                          |
      //     CH2 -----------------+  <--- Physical CJC Tracking Point

      //                          |
      //                       (PT1000 #1)
      //                          |
      //     CH3 -----------------+

      //                          |
      //                       (PT1000 #2)
      //                          |
      //     CH4 -----------------+

      //                          |
      //                       (PT1000 #3)
      //                          |
      //     CH5 -----------------+

      //                          |
      //                       (PT1000 #4)
      //                          |
      //     CH6 -----------------+

      //                          |
      //                       (PT1000 #5)
      //                          |
      //     CH7 -----------------+----> [ Clean System GND ]


      //     CH8  ---> (+) [Thermocouple 1]
      //     CH9  ---> (-) [Thermocouple 1]
      //                           . . . 
      //     CH16 ---> (+) [Thermocouple 5]
      //     CH17 ---> (-) [Thermocouple 5]

#include <stdint.h>
// Mock SPI function - replace this with your actual microcontroller SPI write driver
void LTC2983_SPI_Write(uint16_t memory_address, uint32_t data_word);

/**
 * @brief Configures the LTC2983 memory map for 5 stacked PT1000 probes and 5 K-Type Thermocouples.
 *        Assumes a 2.00 kOhm Sense Resistor is hooked up to CH1/CH2.
 *        CH2 (PT1000 #1) is assigned as the Cold Junction Compensation (CJC) source.
 */
void Configure_LTC2983_Channels() {
    
    // --- STEP 1: DEFINE BASE MEMORY ADDRESSES FOR CHANNELS ---
    // In the LTC2983, each channel configuration is a 4-byte (32-bit) word starting at 0x0200
    const uint16_t CH_BASE_ADDR = 0x0200;
    
    // --- STEP 2: CONSTRUCT CONFIGURATION WORDS ---
    
    // Sense Resistor Configuration (Assigned to CH1, pointing to CH2 boundary)
    // Sensor Type = 0x1D (Sense Resistor)
    // Value = 2000 Ohms -> (2000 * 1024) = 2,048,000 = 0x001F4000
    uint32_t config_rsense = (0x1DUL << 27) | 0x001F4000UL;

    // PT1000 Configuration (2-Wire, 25uA Excitation, Global 3-Cycle Settling Delay)
    // Sensor Type = 0x0F (RTD PT-1000)
    // RSENSE Channel Pointer = 1 (Signals Rsense is connected at CH1)
    // Wire Config = 0x00 (2-Wire)
    // Excitation Current = 0x01 (25 uA)
    // Standard Curve = 0x00 (European Curve, alpha = 0.00385)
    uint32_t config_pt1000 = (0x0FUL << 27) | // Sensor Type
                             (1UL    << 22) | // Rsense Pointer (CH1)
                             (0x00UL << 18) | // 2-Wire Connection
                             (0x01UL << 14) | // 25uA Excitation
                             (0x00UL << 12);  // European Curve

    // K-Type Thermocouple Configuration (Pointed to CH2 for CJC tracking)
    // Sensor Type = 0x02 (Type K Thermocouple)
    // Cold Junction Pointer = 2 (Points to CH2 for temperature reference)
    // Measurement Mode = 0x00 (Differential)
    // Open Circuit Check = 0x01 (Enabled, 10uA current sense pulse)
    uint32_t config_tc_k = (0x02UL << 27) | // Sensor Type
                           (2UL    << 22) | // CJC Channel Pointer (CH2)
                           (0x00UL << 21) | // Differential Mode
                           (0x01UL << 19) | // Open Circuit Fault Check
                           (0x03UL << 16);  // Global 3-Cycle Delay (For MUX settling)


    // --- STEP 3: FLASH CONFIGURATIONS TO THE LTC2983 ---

    // Write Sense Resistor to Channel 1
    LTC2983_SPI_Write(CH_BASE_ADDR + (0 * 4), config_rsense);

    // Write 5 Stacked PT1000 Probes (Channels 2 through 6)
    for (int i = 1; i <= 5; i++) {
        LTC2983_SPI_Write(CH_BASE_ADDR + (i * 4), config_pt1000);
    }

    // Write 5 K-Type Thermocouples (Assigned to High Channels: 8, 10, 12, 14, 16)
    // Note: The low channels (9, 11, 13, 15, 17) act as negative inputs automatically
    for (int tc = 0; tc < 5; tc++) {
        uint8_t physical_channel = 8 + (tc * 2); 
        LTC2983_SPI_Write(CH_BASE_ADDR + ((physical_channel - 1) * 4), config_tc_k);
    }
}
