#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <event_groups.h>
#include <timers.h>
#include <debug_print.hpp>
#include <Wire.h>
#include <TFT_eSPI.h>  
#include <TFT_eWidget.h> 

#ifndef __COMMON_DEFINES_H
#define __COMMON_DEFINES_H
#ifdef __cplusplus
extern "C"
{
#endif

/*~~~~~~~~~~~~~     ECG Analog Frontend     ~~~~~~~~~~~~~*/
#define PIN_ECG_LEADOFF_P 6            // ECG lead off +
#define PIN_ECG_LEADOFF_N 7            // ECG lead off -
// #define PIN_ECG_ADC_DRDY 13             // ECG data ready
#define I2C_HW_ECG_ADC_I2C Wire           // I2C hw handle
#define PIN_ECG_ADC_SCL 9              // ECG data SCL
#define PIN_ECG_ADC_SDA 8              // ECG data SDA
#define CONFIG_ECG_AOUT_QDEPTH 256      // ECG output queue size
#define CONFIG_ECG_POUT_QDEPTH 256      // ECG output queue size
#define CONFIG_ECG_IN_QDEPTH 512        // ECG input queue size
#define CONFIG_ECG_SAMPRATE 200         // ECG sampling frequency
#define CONFIG_RR_LENGTH 10             // How many RR samples to take?
#define CONFIG_ECG_STACKSIZE 8192       // stack size for FR
#define CONFIG_ECG_DOWNSAMPLE_FACTOR 1
#define EB_ECG_LEADOFF_P_BIT (1<<0)
#define EB_ECG_LEADOFF_N_BIT (1<<1)
#define PIN_BUZZER  2

/*~~~~~~~~~~~~~~     PPG Analog Frontend     ~~~~~~~~~~~~~*/
#define I2C_HW_PPG_ADC_I2C Wire1            // I2C hw handle
// #define PIN_PPG_ADC_DRDY 3              // PPG data ready
#define PIN_PPG_ADC_SCL 11               // PPG data SCL
#define PIN_PPG_ADC_SDA 10               // PPG data SDA
#define CONFIG_PPG_AOUT_QDEPTH 256      // PPG output queue size
#define CONFIG_PPG_POUT_QDEPTH 256      // PPG output queue size
#define CONFIG_PPG_IN_QDEPTH 512        // PPG input queue size
#define CONFIG_PPG_FB_QDEPTH 64        // PPG input queue size
#define CONFIG_PPG_SAMPRATE 100         // PPG sampling frequency
#define CONFIG_PPG_STACKSIZE 8192      // stack size for FR

/*~~~~~~~~~     Temperature Analog Frontend     ~~~~~~~~~~*/
#define I2C_HW_TEMPSENSE I2C_HW_PPG_ADC_I2C

/*~~~~~~~~~~~~~~~~~~     LCD Output     ~~~~~~~~~~~~~~~~*/
#define CONFIG_LCD_STACKSIZE    8192
#define CONFIG_LCD_REFRESHRATE  60

/*~~~~~~~~~~~~~~~~~~     Serial Output     ~~~~~~~~~~~~~~~~*/
#define PIN_SERIALOUT_TX 4
#define PIN_SERIALOUT_RX 5

#define PIN_POWERLED 15

#define I2C_CLOCK_100Khz 100'000
#define I2C_CLOCK_400KHz 400'000
#define I2C_CLOCK_1MHz 1'000'000

#define FR_CORE_AFFINITY_CORE0 ((UBaseType_t)(0 << 1 | 1 << 0))
#define FR_CORE_AFFINITY_CORE1 ((UBaseType_t)(1 << 1 | 0 << 0))
#define FR_CORE_AFFINITY_CORE_BOTH ((UBaseType_t)(1 << 1 | 1 << 0))

    static void idle_blink_task(void *task_param)
    {
        SHOW_TASK_STARTED;
        pinMode(PIN_POWERLED, OUTPUT);
        while (true)
        {
            for (size_t i = 0; i < 256; i += 8)
            {
                analogWrite(PIN_POWERLED, i);
                vTaskDelay(pdMS_TO_TICKS(80));
            }

            for (int i = 256 - 1; i >= 0; i -= 8)
            {
                analogWrite(PIN_POWERLED, i);
                vTaskDelay(pdMS_TO_TICKS(80));
            }
        }
    }


#ifdef __cplusplus
}
#endif
#endif
