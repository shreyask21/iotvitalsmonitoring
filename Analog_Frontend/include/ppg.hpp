#include "common_defines.hpp"

#include <stdint.h>
#include "AHTxx.h"
#include "MAX30100.h"
#include "MAX30100_BeatDetector.h"
#include "MAX30100_Filters.h"
#include "MAX30100_SpO2Calculator.h"
#include "autoranging.hpp"

#ifndef __PPG_ALGO_H
#define __PPG_ALGO_H

#define CURRENT_ADJUSTMENT_PERIOD_MS 500
#define DEFAULT_IR_LED_CURRENT MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START MAX30100_LED_CURR_27_1MA
#define DC_REMOVER_ALPHA 0.95

typedef enum PulseOximeterState
{
    PULSEOXIMETER_STATE_INIT,
    PULSEOXIMETER_STATE_IDLE,
    PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;

typedef struct
{
    float ppg_sig;
    bool PPG_Is_Beat;
} ppg_analog_ouput_t;

typedef struct
{
    float PPG_Heart_Rate;
    uint8_t PPG_SPO2;
} ppg_processed_ouput_t;

typedef struct
{
    uint16_t PPG_In_Red;
    uint16_t PPG_In_IR;
} ppg_input_t;

typedef struct
{
    LEDCurrent PPG_Fb_Red;
} ppg_feedback_t;

__weak void ppg_beat_callback();

xQueueHandle ppg_analog_ouput_queue, ppg_processed_ouput_queue, ppg_input_queue, ppg_feedback_queue;
xQueueHandle temp_sensor_q;

void ppg_read_task(void *task_params)
{
    SHOW_TASK_STARTED;
    MAX30100 ppg_adc;
    AHTxx temp_sensor(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
    ppg_input_t ppg_reading;
    ppg_feedback_t ppg_fb;
    float tempr_reading;
    uint32_t last_tempr_reading;

    I2C_HW_PPG_ADC_I2C.setSDA(PIN_PPG_ADC_SDA);
    I2C_HW_PPG_ADC_I2C.setSCL(PIN_PPG_ADC_SCL);
    I2C_HW_PPG_ADC_I2C.setClock(I2C_CLOCK_400KHz);

    if (!temp_sensor.begin(&I2C_HW_TEMPSENSE))
    {
        LOGERROR("Temp. Sensor FAILED to initialize!");
        vTaskDelete(NULL);
    }
    if (!ppg_adc.begin(&I2C_HW_PPG_ADC_I2C))
    {
        LOGERROR("PPG FAILED to initialize!");
        vTaskDelete(NULL);
    }

// Map the sampling rate manually
#if (CONFIG_PPG_SAMPRATE != 100)
#error "ADJUST PPG SAMPLE RATE MANUALLY"
#endif

    ppg_adc.setSamplingRate(MAX30100_SAMPRATE_100HZ);
    ppg_adc.setMode(MAX30100_MODE_SPO2_HR);
    ppg_adc.setLedsCurrent(DEFAULT_IR_LED_CURRENT, RED_LED_CURRENT_START);

    while (true)
    {
        ppg_adc.update();
        ppg_adc.getRawValues(&ppg_reading.PPG_In_IR, &ppg_reading.PPG_In_Red);
        if (xQueueReceive(ppg_feedback_queue, &ppg_fb, 0) == pdTRUE)
        {
            ppg_adc.setLedsCurrent(DEFAULT_IR_LED_CURRENT, ppg_fb.PPG_Fb_Red);
        }
        if (xQueueIsQueueFullFromISR(ppg_input_queue) == pdTRUE)
        {
            LOGWARN("ppg_input_queue FULL!");
        }
        if (ppg_reading.PPG_In_IR > 20000)
        {
            xQueueSend(ppg_input_queue, &ppg_reading, 0);
        }
        else if (xQueueIsQueueEmptyFromISR(ppg_input_queue) != pdTRUE)
        {
            xQueueReset(ppg_input_queue);
        }
        if (millis() - last_tempr_reading > 2000)
        {
            last_tempr_reading = millis();
            tempr_reading = temp_sensor.readTemperature();
            xQueueSend(temp_sensor_q, &tempr_reading, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / CONFIG_PPG_SAMPRATE));
    }
}

void ppg_process_task(void *task_params)
{
    SHOW_TASK_STARTED;
    PulseOximeterState state = PULSEOXIMETER_STATE_IDLE;
    uint32_t tsFirstBeatDetected = 0;
    uint32_t tsLastBeatDetected = 0;
    uint32_t tsLastBiasCheck = 0;
    uint32_t tsLastCurrentAdjustment = 0;
    BeatDetector beatDetector;
    DCRemover irDCRemover = DCRemover(DC_REMOVER_ALPHA);
    DCRemover redDCRemover = DCRemover(DC_REMOVER_ALPHA);
    FilterBuLp1 lpf;
    uint8_t redLedCurrentIndex = ((uint8_t)RED_LED_CURRENT_START);
    SpO2Calculator spO2calculator;
    ppg_input_t ppg_reading;
    ppg_analog_ouput_t ppg_analog;
    ppg_processed_ouput_t ppg_data;
    ppg_feedback_t ppg_fb;
    float adc_ppg_red, adc_ppg_ir, filtered_ppg_ir;
    AnalogAutoRanging<float> ppg_autorange(CONFIG_PPG_SAMPRATE * 2);
    uint32_t ok_state_count, lastreport;

    while (true)
    {
        while (xQueueReceive(ppg_input_queue, &ppg_reading, 0))
        {
            adc_ppg_ir = irDCRemover.step(ppg_reading.PPG_In_IR);
            adc_ppg_red = redDCRemover.step(ppg_reading.PPG_In_Red);
            filtered_ppg_ir = lpf.step(-adc_ppg_ir);
            ppg_analog.PPG_Is_Beat = beatDetector.addSample(filtered_ppg_ir);

            ppg_data.PPG_Heart_Rate = beatDetector.getRate();
            if (ppg_data.PPG_Heart_Rate > 0.0)
            {
                state = PULSEOXIMETER_STATE_DETECTING;
                spO2calculator.update(adc_ppg_ir, adc_ppg_red, ppg_analog.PPG_Is_Beat);
                ppg_data.PPG_SPO2 = spO2calculator.getSpO2();
                ppg_analog.ppg_sig = ppg_autorange.addSample(filtered_ppg_ir, -100, 100);

                if (ok_state_count > CONFIG_PPG_SAMPRATE && ppg_autorange.isValid())
                {
                    xQueueSend(ppg_analog_ouput_queue, &ppg_analog, 0);
                    if (millis() - lastreport > 1000)
                    {
                        xQueueSend(ppg_processed_ouput_queue, &ppg_data, 0);
                        lastreport = millis();
                    }
                }
                else
                {
                    ok_state_count++;
                }
            }
            else if (state == PULSEOXIMETER_STATE_DETECTING)
            {
                state = PULSEOXIMETER_STATE_IDLE;
                ok_state_count = 0;
                if (!ppg_autorange.isReset())
                {
                    ppg_autorange.reset();
                }
                spO2calculator.reset();
            }
            if (ppg_analog.PPG_Is_Beat)
            {
                ppg_beat_callback();
            }

            if (millis() - tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS)
            {
                bool changed = false;
                if (irDCRemover.getDCW() - redDCRemover.getDCW() > 70000 && redLedCurrentIndex < MAX30100_LED_CURR_50MA)
                {
                    ++redLedCurrentIndex;
                    changed = true;
                }
                else if (redDCRemover.getDCW() - irDCRemover.getDCW() > 70000 && redLedCurrentIndex > 0)
                {
                    --redLedCurrentIndex;
                    changed = true;
                }

                if (changed)
                {
                    ppg_fb.PPG_Fb_Red = (LEDCurrent)redLedCurrentIndex;
                    xQueueSend(ppg_feedback_queue, &ppg_fb, 0);
                    tsLastCurrentAdjustment = millis();
                }
                tsLastBiasCheck = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / CONFIG_PPG_SAMPRATE));
    }
}

void ppg_setup_task(void *task_params)
{
    SHOW_TASK_STARTED;
    temp_sensor_q = xQueueCreate(50, sizeof(float));
    ppg_analog_ouput_queue = xQueueCreate(CONFIG_PPG_AOUT_QDEPTH, sizeof(ppg_analog_ouput_t));
    ppg_processed_ouput_queue = xQueueCreate(CONFIG_PPG_POUT_QDEPTH, sizeof(ppg_processed_ouput_t));
    ppg_input_queue = xQueueCreate(CONFIG_PPG_IN_QDEPTH, sizeof(ppg_input_t));
    ppg_feedback_queue = xQueueCreate(CONFIG_PPG_FB_QDEPTH, sizeof(ppg_feedback_t));

    // Start PPG Acquisition Task (Inherited Priority + 1)
    xTaskCreate(
        ppg_read_task, "PPG Read Task", CONFIG_PPG_STACKSIZE,
        NULL, 6, NULL);

    // Start PPG Signal Processing Task
    xTaskCreate(
        ppg_process_task, "PPG Process Task", CONFIG_PPG_STACKSIZE,
        NULL, 5, NULL);

    SHOW_TASK_STOPPED;
    vTaskDelete(NULL);
}
#endif
