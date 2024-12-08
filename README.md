# **Centralized IoT Patient Vitals Monitoring System**  

A robust system designed for continuous monitoring of patient vitals such as ECG, PPG, and temperature. The project comprises two primary components: **Analog Frontend** for signal acquisition and processing, and a **Webserver** for real-time data visualization and communication.  


## **Project Structure**  

```plaintext
|   LICENSE
|   README.md
+---Analog_Frontend
|   +---include
|   |       autoranging.hpp
|   |       common_defines.hpp
|   |       debug_print.hpp
|   |       ecg.hpp
|   |       ecg_filter.hpp
|   |       heart_frames.h
|   |       output.hpp
|   |       ppg.hpp
|   |       splash_screen.h
|   +---lib
|   |   +---Adafruit_ADS1X15             
|   |   +---Adafruit_BusIO      
|   |   +---AHTxx
|   |   +---MAX30100
|   |   +---Pan-Tompkins
|   |   +---TFT_eSPI
|   |   \---TFT_eWidget
|   |                       
|   +---src
|   |       main.cpp
|           
\---Webserver
    +---data
    |       chart.min.js
    |       index.html
    +---lib
    |   |   
    |   +---ArduinoJson
    |   +---AsyncTCP
    |   \---ESPAsyncWebServer
    +---src
    |       main.cpp
```  

## **Features**  

### **1. Analog Frontend**  
- **Signal Acquisition:**  
  - ECG signals processed via 3-lead electrodes and a 16-bit ADC.  
  - PPG signals for SpO2 and heart rate detection.  
  - Temperature data for continuous body temperature monitoring.  
- **Signal Processing:**  
  - Implements filtering (bandpass, notch) and Pan-Tompkins algorithm for QRS detection.  
  - CMSIS DSP library for real-time signal analysis.  
- **Communication:**  
  - Data sent to the ESP32 for transmission to the webserver.  
---
### **2. Webserver**  
- **Technology Stack:**  
  - **Backend:** Server sent events for real-time communication.  
  - **Frontend:** HTML, CSS, and JavaScript for live data visualization.  
- **Features:**  
  - Displays real-time ECG, PPG, temperature, heart rate, and SpO2 data.  
  - Supports alerts for abnormal vitals.  
  - WebSocket ensures low-latency data updates.  

- **Software:**  
  - PlatformIO 

## **Contributing**  
Contributions are welcome! Feel free to open issues or submit pull requests for improvements.  

![1](https://raw.githubusercontent.com/shreyask21/iotvitalsmonitoring/main/images/1.png)  
---
![2](https://raw.githubusercontent.com/shreyask21/iotvitalsmonitoring/main/images/2.png)
---
![3](https://raw.githubusercontent.com/shreyask21/iotvitalsmonitoring/main/images/3.png)
## **License**  
This project is licensed under the [MIT License](./LICENSE).  