/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __STM32F1__
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#endif

#define DEFAULT_MACHINE_NAME "STM32F103VET6"
#define BOARD_NAME "GTM32 Pro VD"

//#define DISABLE_DEBUG
#define DISABLE_JTAG
//#define DISABLE_JTAGSWD

// Ignore temp readings during development.
#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#define FIL_RUNOUT_PIN     PA6  // 1K PULLUP
#define FIL_RUNOUT2_PIN    PA7  // 1K PULLUP
#define FIL_RUNOUT3_PIN    PE2  // 1K PULLUP

//
// Limit Switches
//
#define X_MIN_PIN          PE5
//#define X_MAX_PIN          PE4   //connects to PIN2 J6 which is not soldered, connected to voltage divider R44 and R45. maybe for PWR_DET?
#define Y_MIN_PIN          PE3
//#define Y_MAX_PIN          PE2   // used for FIL_RUNOUT3 on conn H10
#define Z_MIN_PIN          PE1   // connects to H7 on board. used for BLTouch
#define Z_MAX_PIN          PE0   // used with conventional endstop

//
// Steppers
//
#define X_STEP_PIN         PC6
#define X_DIR_PIN          PD13
#define X_ENABLE_PIN       PA8

#define Y_STEP_PIN         PA12
#define Y_DIR_PIN          PA11
#define Y_ENABLE_PIN       PA15

#define Z_STEP_PIN         PD6
#define Z_DIR_PIN          PD3
#define Z_ENABLE_PIN       PB3


#define E0_STEP_PIN        PC14
#define E0_DIR_PIN         PC13
#define E0_ENABLE_PIN      PC15

#define E1_STEP_PIN        PA5  // simultaneously connected to SPI1_SCK
#define E1_DIR_PIN         PB6
#define E1_ENABLE_PIN      PA1

//E2 port is used as Z2 in stock configuration
#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define Z2_STEP_PIN      PB2 // 10K PULLDOWN
  #define Z2_DIR_PIN       PB11
  #define Z2_ENABLE_PIN    PC4
#else
  #define E2_STEP_PIN      PB2 // 10K PULLDOWN
  #define E2_DIR_PIN       PB11
  #define E2_ENABLE_PIN    PC4
#endif
//
// Heaters / Fans
//
#define HEATER_0_PIN       PB4
#define HEATER_1_PIN       PB5
#define HEATER_2_PIN       PB0
#define HEATER_BED_PIN     PB1

#define FAN_PIN            PB7   // TODO: Add functionality
#define FAN1_PIN           PB8   // TODO: Add functionality
#define FAN2_PIN           PB9   // TODO: Add functionality

//
// Temperature Sensors
//
#define TEMP_0_PIN         PC0   // PORT NUMBERING
#define TEMP_1_PIN         PC1   // PORT NUMBERING
#define TEMP_2_PIN         PC2   // PORT NUMBERING
#define TEMP_BED_PIN       PC3   // PORT NUMBERING

//
// Misc. Functions
//
#define LED_PWM            PD12   // External LED, pin 2 on LED labeled connector
//#define RESET_PIN        NRST
#define T_PEN              PE6

#define BEEPER_PIN         PB10

#if ENABLED(BLTOUCH)
  #define SERVO0_PIN         PA0
#endif

//
// LCD / Controller
//
// #if ENABLED(ULTRA_LCD)

//   #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
//     #define LCD_PINS_RS    PD11   // CS chip select /SS chip slave select
//     // #define LCD_PINS_ENABLE PE14 // SID (MOSI)
//     #define LCD_PINS_D4    PD8   // SCK (CLK) clock
//     #define LCD_PINS_D5    PD9
//     #define LCD_PINS_D6    PD10
//     #define LCD_PINS_D7    PE15

//     #define BTN_EN1        PE8
//     #define BTN_EN2        PE9
//     #define BTN_ENC        PE13

//     #define SD_DETECT_PIN  PC7
//     #define KILL_PIN       -1
//   #endif

//   #ifndef ST7920_DELAY_1
//     #define ST7920_DELAY_1 DELAY_NS(96)
//   #endif
//   #ifndef ST7920_DELAY_2
//     #define ST7920_DELAY_2 DELAY_NS(48)
//   #endif
//   #ifndef ST7920_DELAY_3
//     #define ST7920_DELAY_3 DELAY_NS(715)
//   #endif

//   //
//   // UART LCD Pins, if used
//   //
//   //#define LCD_PINS_D14   PD9   // RXD3
//   //#define LCD_PINS_D13   PD8   // TXD3

// #endif // ULTRA_LCD

//
// SPI1
// connections for a 8-pin SOIC/VSOP serial flash chip prepared, but not soldered
// maybe to reuse pins differently
//
// #define SPI1_NSS           PA4  //
// #define SPI1_SCK           PA5  // used for E1_STEP
// #define SPI1_MISO          PA6  // used for FIL_RUNOUT
// #define SPI1_MOSI          PA7  // used for FIL_RUNOUT2

//
// SPI2 (on LCD conn)
//
#define SPI2_MOSI          PB15
#define SPI2_MISO          PB14
#define SPI2_SCK           PB13
#define SPI2_NSS           PB12

//
// SD Card (SD_CARD conn, also on LCD conn)
//
#define SD_CD              PC7
#define SD_DATA0           PC8
#define SD_DATA1           PC9
#define SD_DATA2           PC10
#define SD_DATA3           PC11
#define SD_CMD             PD2
#define SD_CLK             PC12


//
// Debug
//
//#define SDA              PA13
//#define SCL              PA14

//
// Wifi
//
#define USART2_RX          PA3   // Default alternate function
#define USART2_TX          PA2   // Default alternate function

//
// For future use
//
//#undef USB_DM
//#undef CAN_TX
//#undef USB_DP
//#undef CAN_RX
//#undef BOARD_JTDI_PIN
//#define BOOT0            BOOT0

//pin listing GTM32 PRO VD
// NRST	RESET
// PA0	E1_STEP	PIN3 J3	now for 3DTouch->E1_Step moved to PA5(SPI1_SCK)
// PC5	PWR_DET
// PE7	LCD_D4	PIN23 J2
// PE8	LCD_D5	PIN13 J2
// PE9	LCD_D6	PIN12 J2
// PE10	LCD_D7	PIN11 J3
// PE11	LCD_D8	PIN10 J3
// PE12	LCD_D9	PIN9 J3
// PE13	LCD_D10	PIN8 J4
// PE14	LCD_D11	PIN7 J4
// PE15	LCD_D12	PIN6 J4
// PD8	LCD_D13	PIN5 J2
// PD9	LCD_D14	PIN4 J2
// PD10	LCD_D15	PIN3 J2
// PD11	LCD_RS	PIN16 J2
// PD14	LCD_D0	PIN19 J2
// PD15	LCD_D1	PIN20 J2
// PA9	AOFI
// PA10	AIFO
// PD0	LCD_D2	PIN21 J2
// PD1	LCD_D3	PIN22 J2
// PD4	LCD_RD	PIN15 J2
// PD5	LCD_WR	PIN14 J2
// PD7	LCD_CS	PIN17 J2
// BOOT0	BOOT0		10K PULLDOWN
