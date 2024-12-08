#include "common_defines.hpp"
#ifndef __OUTPUT_H
#define __OUTPUT_H
#include "ecg.hpp"
#include "ppg.hpp"
#include "heart_frames.h"
#include "splash_screen.h"

#define GRAPH_X_MIN 000.0
#define GRAPH_X_MAX_PPG 400.0
#define GRAPH_X_MAX_ECG 400.0
#define GRAPH_Y_MIN -120.0
#define GRAPH_Y_MAX +120.0

#define PPG_GR_XPOS 12
#define PPG_GR_YPOS 12
#define PPG_GR_TRACE TFT_CYAN

#define ECG_GR_XPOS 12
#define ECG_GR_YPOS 68
#define ECG_GR_TRACE TFT_RED

#define GRAPH_WIDTH_X 256
#define GRAPH_HEIGHT_Y 50

#define OUTPUT_IDLE_TIME 15000

// #pragma pack(1)
typedef struct
{

    uint8_t start_byte;
    int16_t temperature;
    int16_t hr_bpm;
    int16_t hrv_mean_rr;
    int16_t hrv_sdnn;
    int16_t hrv_rmssd;
    int16_t hrv_pnn50;
    int16_t ecg_sig;
    bool isbeat_ecg;
    int16_t ppg_sig;
    bool isbeat_ppg;
    int16_t ppg_hr;
    uint8_t ppg_spo2;
    bool ecg_idle;
    bool ppg_idle;
    uint8_t stop_byte;
} data_pkt;

typedef union
{
    data_pkt packet;
    uint8_t buffer[sizeof(data_pkt)];
} data_pkt_u;

void data_print_output(void *params)
{
    SHOW_TASK_STARTED;
    Serial2.setTX(PIN_SERIALOUT_TX);
    Serial2.setRX(PIN_SERIALOUT_RX);
    Serial2.begin(921600);
    pinMode(PIN_BUZZER, OUTPUT_12MA);
    digitalWrite(PIN_BUZZER, LOW);
    ppg_analog_ouput_t ppg_analog = {0};
    ppg_processed_ouput_t ppg_data = {0};
    ecg_analog_output_t ecg_analog = {0};
    ecg_processed_ouput_t ecg_data = {0};
    data_pkt_u data_pkt = {0};
    float temp_data;
    bool newData = false;
    TFT_eSPI tft = TFT_eSPI();
    TFT_eSprite spr = TFT_eSprite(&tft);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.initDMA();
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, tft.width(), tft.height(), splash_screen);
    delay(5000);
    spr.createSprite(tft.width(), tft.height());
    spr.setSwapBytes(true);
    uint8_t disp_br = 0;
    analogWriteFreq(1'000);
    analogWrite(TFT_BL, disp_br);

    GraphWidget ecg_graph = GraphWidget(&spr);
    TraceWidget ecg_trace = TraceWidget(&ecg_graph);

    GraphWidget ppg_graph = GraphWidget(&spr);
    TraceWidget ppg_trace = TraceWidget(&ppg_graph);

    static float ecg_point_index = 0.0;
    static float ppg_point_index = 0.0;
    static uint16_t downsample_idx = 0, beat_animation_index = 0;
    static uint32_t buzzer_on_time = 0, beat_decay_time = 0, lastrefresh = 0;
    static uint32_t last_ppg_sample_time = 0, last_ecg_sample_time = 0;
    bool buzzerstatus = false, isECGIdle = true, isPPGIdle = true;

    ppg_graph.createGraph(GRAPH_WIDTH_X, GRAPH_HEIGHT_Y, TFT_BLACK);
    ppg_graph.setGraphScale(GRAPH_X_MIN, GRAPH_X_MAX_PPG, GRAPH_Y_MIN, GRAPH_Y_MAX);
    ppg_graph.setGraphGrid(GRAPH_X_MIN, GRAPH_X_MAX_PPG - GRAPH_X_MIN, GRAPH_Y_MIN, GRAPH_Y_MAX - GRAPH_Y_MIN, TFT_GOLD);

    ecg_graph.createGraph(GRAPH_WIDTH_X, GRAPH_HEIGHT_Y, TFT_BLACK);
    ecg_graph.setGraphScale(GRAPH_X_MIN, GRAPH_X_MAX_ECG, GRAPH_Y_MIN, GRAPH_Y_MAX);
    ecg_graph.setGraphGrid(GRAPH_X_MIN, GRAPH_X_MAX_ECG - GRAPH_X_MIN, GRAPH_Y_MIN, GRAPH_Y_MAX - GRAPH_Y_MIN, TFT_GOLD);
    ecg_graph.drawGraph(ECG_GR_XPOS, ECG_GR_YPOS);
    ecg_trace.startTrace(ECG_GR_TRACE);
    ppg_graph.drawGraph(PPG_GR_XPOS, PPG_GR_YPOS);
    ppg_trace.startTrace(PPG_GR_TRACE);
    spr.setFreeFont(&FreeMonoBold12pt7b);

    spr.setTextColor(TFT_BLUE);
    spr.setCursor(8, 140);
    spr.printf("ECG  :");
    spr.setCursor(8, 220);
    spr.printf("HRV  :");
    spr.setCursor(8, 200);
    spr.printf("Temp :");
    spr.setCursor(8, 160);
    spr.printf("PPG  :");
    spr.setCursor(8, 180);
    spr.printf("SpO2 :");

    spr.setCursor(108, 160);
    spr.setTextColor(PPG_GR_TRACE);
    spr.printf("--- BPM"); // PPG
    spr.setCursor(108, 180);
    spr.printf("--- %%"); // SPO2
    spr.setTextColor(ECG_GR_TRACE);
    spr.setCursor(108, 140);
    spr.printf("--- BPM"); // ECG
    spr.setCursor(108, 220);
    spr.printf("---- ms --%%"); // HRV
    spr.setTextColor(TFT_GREEN);
    spr.setCursor(108, 200);
    spr.printf("--- C");
    spr.pushSprite(0, 0);

    while (true)
    {
        newData = false;
        if (xQueueReceive(ppg_analog_ouput_queue, &ppg_analog, 0))
        {
            ppg_trace.addPoint(ppg_point_index, ppg_analog.ppg_sig);
            ppg_point_index += 1.0;
            if (ppg_point_index > GRAPH_X_MAX_PPG)
            {
                ppg_point_index = 0.0;
                ppg_graph.drawGraph(PPG_GR_XPOS, PPG_GR_YPOS);
                ppg_trace.startTrace(PPG_GR_TRACE);
            }
            if (ppg_analog.PPG_Is_Beat)
            {
                beat_animation_index = 0;
                buzzerstatus = true;
                digitalWrite(PIN_BUZZER, HIGH);
                buzzer_on_time = millis();
            }
            if (xQueueReceive(ppg_processed_ouput_queue, &ppg_data, 0))
            {
                __SER_PERPH_HANDLE.printf(
                    "PPG_hr_bpm: %0.1f\t"
                    "PPG_SPO2: %d\r\n",
                    ppg_data.PPG_Heart_Rate,
                    ppg_data.PPG_SPO2);
            }
            newData = true;
            isPPGIdle = false;
            LOGINFO1("PPG TDELTA:", millis() - last_ppg_sample_time);
            last_ppg_sample_time = millis();
        }
        else
        {
            isPPGIdle = (millis() - last_ppg_sample_time > OUTPUT_IDLE_TIME);
        }

        if (xQueueReceive(ecg_analog_ouput_queue, &ecg_analog, 0))
        {
            if (downsample_idx == CONFIG_ECG_DOWNSAMPLE_FACTOR)
            {
                downsample_idx = 0;
            }
            else
            {
                downsample_idx++;
            }
            if (downsample_idx == 0)
            {
                ecg_trace.addPoint(ecg_point_index, ecg_analog.ecg_sig);
                ecg_point_index += 1.0;
                if (ecg_point_index > GRAPH_X_MAX_ECG)
                {
                    ecg_point_index = 0.0;
                    ecg_graph.drawGraph(ECG_GR_XPOS, ECG_GR_YPOS);
                    ecg_trace.startTrace(ECG_GR_TRACE);
                }
                newData = true;
            }

            if (ecg_analog.isBeat && isPPGIdle)
            {
                beat_animation_index = 0;
                buzzerstatus = true;
                digitalWrite(PIN_BUZZER, HIGH);
                buzzer_on_time = millis();
            }

            if (xQueueReceive(ecg_processed_ouput_queue, &ecg_data, 0))
            {
                __SER_PERPH_HANDLE.printf(
                    "hr_bpm: %d\t"
                    "hrv_mean_rr: %d\t"
                    "hrv_sdnn: %d\t"
                    "hrv_rmssd: %d\t"
                    "hrv_pnn50: %d\r\n",
                    ecg_data.hr_bpm,
                    ecg_data.hrv_mean_rr,
                    ecg_data.hrv_sdnn,
                    ecg_data.hrv_rmssd,
                    ecg_data.hrv_pnn50);
            }
            
            isECGIdle = false;
            // LOGINFO1("ECG TDELTA:", millis() - last_ecg_sample_time);
            last_ecg_sample_time = millis();
        }
        else
        {
            isECGIdle = (millis() - last_ecg_sample_time > OUTPUT_IDLE_TIME);
        }

        xQueueReceive(temp_sensor_q, &temp_data, 0);

        if (buzzerstatus && (millis() - buzzer_on_time > 25))
        {
            buzzerstatus = false;
            digitalWrite(PIN_BUZZER, LOW);
        }

        if ((millis() - beat_decay_time > 24) && beat_animation_index != ((sizeof(frames) / sizeof(frames[0]) - 1)))
        {
            // spr.pushImage(210, 125, 64, 64, frames[beat_animation_index]);
            beat_animation_index++;
            beat_decay_time = millis();
        }

        if (newData && (!isECGIdle || !isPPGIdle))
        {
            data_pkt.packet.start_byte = '\r';
            data_pkt.packet.ecg_sig = ecg_analog.ecg_sig;
            data_pkt.packet.isbeat_ecg = ecg_analog.isBeat;
            data_pkt.packet.hr_bpm = ecg_data.hr_bpm;
            data_pkt.packet.hrv_mean_rr = ecg_data.hrv_mean_rr;
            data_pkt.packet.hrv_pnn50 = ecg_data.hrv_pnn50;
            data_pkt.packet.hrv_rmssd = ecg_data.hrv_rmssd;
            data_pkt.packet.hrv_sdnn = ecg_data.hrv_sdnn;
            data_pkt.packet.ppg_sig = (int16_t)ppg_analog.ppg_sig;
            data_pkt.packet.isbeat_ppg = ppg_analog.PPG_Is_Beat;
            data_pkt.packet.ppg_hr = (int16_t)ppg_data.PPG_Heart_Rate;
            data_pkt.packet.ppg_spo2 = ppg_data.PPG_SPO2;
            data_pkt.packet.ecg_idle = isECGIdle;
            data_pkt.packet.ppg_idle = isPPGIdle;
            data_pkt.packet.temperature = (int16_t)temp_data;
            data_pkt.packet.stop_byte = '\n';
            Serial2.write(data_pkt.buffer, sizeof(data_pkt));
        }

        if ((millis() - lastrefresh > (1000 / CONFIG_LCD_REFRESHRATE)))
        {
            if (!isPPGIdle)
            {
                spr.setTextColor(PPG_GR_TRACE);
                spr.setCursor(108, 160);
                spr.printf("%-3.0f BPM", ppg_data.PPG_Heart_Rate); // PPG
                spr.setCursor(108, 180);
                spr.printf("%-3d %%", ppg_data.PPG_SPO2); // SPO2
            }
            else
            {
                spr.setTextColor(PPG_GR_TRACE);
                spr.setCursor(108, 160);
                spr.printf("--- BPM"); // PPG
                spr.setCursor(108, 180);
                spr.printf("--- %%"); // SPO2
            }
            if (!isECGIdle)
            {
                spr.setTextColor(ECG_GR_TRACE);
                spr.setCursor(108, 140);
                spr.printf("%-3d BPM", ecg_data.hr_bpm); // ECG
                spr.setCursor(108, 220);
                spr.printf("%-4d ms %-2d%%", ecg_data.hrv_sdnn, ecg_data.hrv_pnn50); // HRV
            }
            else
            {

                spr.setTextColor(ECG_GR_TRACE);
                spr.setCursor(108, 140);
                spr.printf("--- BPM"); // ECG
                spr.setCursor(108, 220);
                spr.printf("---- ms --%%"); // HRV
            }
            if (isECGIdle && isPPGIdle)
            {
                spr.setTextColor(TFT_GREEN);
                spr.setCursor(108, 200);
                spr.printf("--- C");
                if (disp_br < 180)
                {
                    disp_br += 4;
                    analogWrite(TFT_BL, disp_br);
                }
            }
            else
            {
                spr.setTextColor(TFT_GREEN);
                spr.setCursor(108, 200);
                spr.printf("%-3d C", (int16_t)temp_data);
                if (disp_br > 0)
                {
                    disp_br--;
                    analogWrite(TFT_BL, disp_br);
                }
            }
            spr.pushImage(210, 125, 64, 64, frames[beat_animation_index]);
            spr.pushSprite(0, 0);
            spr.fillRect(100, 120, tft.width(), tft.height() - 120, TFT_BLACK);
            lastrefresh = millis();
        }
        else if (isECGIdle && isPPGIdle)
        {
            delay((1000 / (CONFIG_ECG_SAMPRATE)));
        }
        else
        {
            portYIELD();
        }
    }
}
#endif