#include "common_defines.hpp"
#include "ecg.hpp"
#include "ppg.hpp"
#include "output.hpp"


void setup()
{

  init_threadsafe_debug_prints();

  LOGINFO3("~~~~ Vital Signs Monitoring: [", __DATE__, __TIME__, "] ~~~~\r\n");

  xTaskCreate(
      ecg_setup_task, "ECG Setup Task", CONFIG_ECG_STACKSIZE,
      NULL, 5, NULL);

  xTaskCreate(
      ppg_setup_task, "PPG Setup Task", CONFIG_PPG_STACKSIZE,
      NULL, 5, NULL);

  xTaskCreate(
      data_print_output, "Data Print Task", CONFIG_ECG_STACKSIZE,
      NULL, 4, NULL);

  xTaskCreate(idle_blink_task, "Idle Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

void loop()
{
  vTaskSuspend(NULL);
}