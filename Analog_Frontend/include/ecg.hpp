#include "common_defines.hpp"
#include "Adafruit_ADS1X15.h"
#include "autoranging.hpp"
#include "ecg_filter.hpp"

extern "C"
{
#include "PanTompkins.h"
}

#ifndef __ECG_ALGO_H
#define __ECG_ALGO_H
typedef struct
{
    int16_t hr_bpm;
    int16_t hrv_mean_rr;
    int16_t hrv_sdnn;
    int16_t hrv_rmssd;
    int16_t hrv_pnn50;
} ecg_processed_ouput_t;

typedef struct
{
    int16_t ecg_sig;
    bool isBeat;
} ecg_analog_output_t;

xQueueHandle ecg_analog_ouput_queue, ecg_processed_ouput_queue, ecg_input_queue;

__weak void ecg_beat_callback();
__weak void ecg_leadoff_callback(bool isLeadOff);

// ECG adc task
// Reads 16-bit ADC from I2C and pushes it to input queue
void ecg_read_task(void *task_params)
{
    pinMode(LED_BUILTIN, OUTPUT);
    adsGain_t adc_gains[] = {GAIN_TWOTHIRDS,
                             GAIN_ONE,
                             GAIN_TWO,
                             GAIN_FOUR,
                             GAIN_EIGHT,
                             GAIN_SIXTEEN};
    uint8_t gain_index = 1;

    int16_t adc_reading;
    uint32_t leadoff_delay, adc_samp_idx = 0, adc_accum = 0;
    bool isLeadOff = false, isLeadOffLast = true;
    SHOW_TASK_STARTED;
    Adafruit_ADS1115 ecg_adc;
    AnalogAutoRanging<int16_t> ecg_autorange(CONFIG_ECG_SAMPRATE * 2);
    ECGFilter ecg_filter;
    uint32_t start_time;
    I2C_HW_ECG_ADC_I2C.setSDA(PIN_ECG_ADC_SDA);
    I2C_HW_ECG_ADC_I2C.setSCL(PIN_ECG_ADC_SCL);
    I2C_HW_ECG_ADC_I2C.setClock(I2C_CLOCK_400KHz);
    if (!ecg_adc.begin(ADS1X15_ADDRESS, &I2C_HW_ECG_ADC_I2C))
    {
        LOGERROR("ECG ADC FAILED to initialize!");
        vTaskDelete(NULL);
    }
    ecg_adc.setGain(adc_gains[gain_index]);
    ecg_adc.setDataRate(RATE_ADS1115_860SPS);
    ecg_adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    vTaskDelay(10);
    while (true)
    {
        start_time = micros();
        if (!((!digitalRead(PIN_ECG_LEADOFF_N)) && (!digitalRead(PIN_ECG_LEADOFF_P))))
        {
            if (millis() - leadoff_delay > 1000)
            {
                isLeadOff = true;
            }
            else
            {
                leadoff_delay = millis();
            }
        }
        else
        {
            if (millis() - leadoff_delay > 1000)
            {
                isLeadOff = false;
            }
            else
            {
                leadoff_delay = millis();
            }
        }

        digitalWrite(LED_BUILTIN, !isLeadOff);
        if (isLeadOff != isLeadOffLast)
        {
            isLeadOffLast = isLeadOff;
            LOGINFO(isLeadOff ? "ECG LEADS OFF!" : "ECG LEADS OK");
            ecg_leadoff_callback(isLeadOff);
            if (isLeadOff && !ecg_autorange.isReset())
            {
                ecg_autorange.reset();
                // PT_init();
            }
        }
        if ((!isLeadOff))
        {
            adc_reading = ecg_adc.readADC_SingleEnded(0);
            // adc_reading = ecg_filter.process_sample(adc_reading);
            adc_reading = ecg_autorange.addSample(adc_reading, -1000, 1000);
            if (xQueueIsQueueFullFromISR(ecg_input_queue) == pdTRUE)
            {
                LOGWARN("ecg_input_queue FULL!");
            }
            if (ecg_autorange.isValid())
            {
                // if (ecg_autorange.getMax() < ((32768 >> 1) - (32768 >> 3)) && gain_index < 6)
                // {
                //     ecg_adc.setGain(adc_gains[++gain_index]);
                //     ecg_autorange.reset();
                // }
                // else if (ecg_autorange.getMax() > ((32768 >> 1) + (32768 >> 3)) && gain_index > 0)
                // {
                //     ecg_adc.setGain(adc_gains[--gain_index]);
                //     ecg_autorange.reset();
                // }
                // else
                // {
                xQueueSend(ecg_input_queue, &adc_reading, 0);
                // }
            }
            delayMicroseconds(5000 - (micros() - start_time));
            portYIELD();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// ECG signal processing task
// Reads samples from queue and runs pan-tompkins algo state machine
// Analyzes HR, R-R and HRV and detects beats and pushes to output queue
void ecg_process_task(void *task_params)
{
    static ecg_analog_output_t ecg_analog;
    static ecg_processed_ouput_t ecg_data;
    static int16_t adc_reading, adc_max, adc_min;
    static uint16_t rr_intervals[CONFIG_RR_LENGTH];
    static uint8_t rr_index = 0;
    static uint32_t last_timestamp;
    static float accum_rr = 0, count_pnn50 = 0,
                 accum_sdnn = 0, temp_sdnn = 0,
                 temp_rmssd = 0, accum_rmssd = 0;

    AnalogAutoRanging<int16_t> ecg_op_autorange(CONFIG_ECG_SAMPRATE * 2);
    SHOW_TASK_STARTED;
    PT_init();
    while (true)
    {
        // if (xQueueIsQueueEmptyFromISR(ecg_input_queue) == pdTRUE)
        // {
        //     LOGWARN("ecg_input_queue EMPTY!");
        // }
        while (xQueueReceive(ecg_input_queue, &adc_reading, 0))
        {
            ecg_analog.isBeat = false;
            if ((PT_StateMachine(adc_reading) != 0) && (millis() - last_timestamp) > 350)
            {
                ecg_beat_callback();
                ecg_analog.isBeat = true;
                rr_intervals[rr_index] = millis() - last_timestamp;
                last_timestamp = millis();
                rr_index = (rr_index + 1) % CONFIG_RR_LENGTH;
                if (rr_index == 0)
                {
                    accum_rr = 0;
                    count_pnn50 = 0;
                    accum_sdnn = 0;
                    temp_sdnn = 0;
                    temp_rmssd = 0;
                    accum_rmssd = 0;
                    for (size_t i = 0; i < CONFIG_RR_LENGTH; i++)
                    {
                        accum_rr += rr_intervals[i];

                        temp_sdnn = rr_intervals[i] - ecg_data.hrv_mean_rr;
                        temp_sdnn *= temp_sdnn;
                        accum_sdnn += temp_sdnn;

                        if (i > 0)
                        {
                            temp_rmssd = rr_intervals[i] - rr_intervals[i - 1];
                            temp_rmssd *= temp_rmssd;
                            accum_rmssd += temp_rmssd;

                            if (abs(rr_intervals[i] - rr_intervals[i - 1]) > 50.0)
                            {
                                count_pnn50 += 1.0;
                            }
                        }
                    }
                    accum_rr /= CONFIG_RR_LENGTH;
                    ecg_data.hrv_mean_rr = accum_rr;
                    accum_rr = 60'000 / accum_rr;
                    ecg_data.hr_bpm = accum_rr;
                    ecg_data.hrv_sdnn = sqrt(accum_sdnn / (CONFIG_RR_LENGTH - 1));
                    ecg_data.hrv_rmssd = sqrt(accum_rmssd / (CONFIG_RR_LENGTH - 1));
                    ecg_data.hrv_pnn50 = count_pnn50 / (CONFIG_RR_LENGTH - 1) * 100.0;
                    if (xQueueIsQueueFullFromISR(ecg_processed_ouput_queue) == pdTRUE)
                    {
                        LOGWARN("ecg_processed_ouput_queue FULL!");
                    }
                    if (ecg_op_autorange.isValid())
                    {
                        xQueueSend(ecg_processed_ouput_queue, &ecg_data, 0);
                    }
                }
            }
            // ecg_analog.ecg_sig = adc_reading;
            // __PRINTLN(ecg_analog.ecg_sig);
            ecg_analog.ecg_sig = ecg_op_autorange.addSample(PT_get_DRFilter_output(), -100, 100);
            if (xQueueIsQueueFullFromISR(ecg_analog_ouput_queue) == pdTRUE)
            {
                LOGWARN("ecg_analog_ouput_queue FULL!");
            }
            if (ecg_op_autorange.isValid())
            {
                xQueueSend(ecg_analog_ouput_queue, &ecg_analog, 0);
            }
        }
        // else
        // {
        vTaskDelay(pdMS_TO_TICKS((1000 / CONFIG_ECG_SAMPRATE)));
        // }
    }
}

void ecg_setup_task(void *task_params)
{
    SHOW_TASK_STARTED;

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(PIN_ECG_LEADOFF_P, INPUT_PULLUP);
    pinMode(PIN_ECG_LEADOFF_N, INPUT_PULLUP);

    ecg_input_queue = xQueueCreate(CONFIG_ECG_IN_QDEPTH, sizeof(int16_t));
    ecg_analog_ouput_queue = xQueueCreate(CONFIG_ECG_AOUT_QDEPTH, sizeof(ecg_analog_output_t));
    ecg_processed_ouput_queue = xQueueCreate(CONFIG_ECG_POUT_QDEPTH, sizeof(ecg_processed_ouput_t));

    // Start ECG Acquisition Task (Inherited Priority + 1)
    xTaskCreate(
        ecg_read_task, "ECG Read Task", CONFIG_ECG_STACKSIZE,
        NULL, 6, NULL);

    // Start ECG Signal Processing Task
    xTaskCreate(
        ecg_process_task, "ECG Process Task", CONFIG_ECG_STACKSIZE,
        NULL, 5, NULL);

    SHOW_TASK_STOPPED;
    vTaskDelete(NULL);
}
#endif