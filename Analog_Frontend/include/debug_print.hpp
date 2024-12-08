#ifndef __THREADSAFE_PRINT_H
#define __THREADSAFE_PRINT_H

#define __SER_PERPH_HANDLE Serial1
#define __SER_PERPH_BAUDRATE 256'000

////////////////////////////////////////

// Change _LOG_VERBOSITY_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _LOG_VERBOSITY_
#define _LOG_VERBOSITY_      4
#endif

#define __CONFIG_TSPRINTF_QUEUE_SIZE    40960

xQueueHandle __debug_ouput_queue;
xSemaphoreHandle __debug_ouput_sem, __sequence_sem;

class __ts_printclass : public Print
{
    size_t write(uint8_t input)
    {
        xQueueSend(__debug_ouput_queue, &input, portMAX_DELAY);
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size)
    {
        size_t n = 0;
        while (size--)
        {
            n += write(*buffer++);
        }
        return n;
    }
};

static __ts_printclass __ts_printclass_handle;

#define __TAKE_SEM  xSemaphoreTake(__sequence_sem, portMAX_DELAY)
#define __GIVE_SEM  xSemaphoreGive(__sequence_sem)

// #define __PRINT        __ts_printclass_handle.print
// #define __PRINTLN      __ts_printclass_handle.println
// #define __PRINTF       __ts_printclass_handle.printf

#define __PRINT        __SER_PERPH_HANDLE.print
#define __PRINTLN      __SER_PERPH_HANDLE.println
#define __PRINTF       __SER_PERPH_HANDLE.printf

#define __MARK_ERROR    __PRINT("[ERRR]\t")
#define __MARK_WARN     __PRINT("[WARN]\t")
#define __MARK_INFO     __PRINT("[INFO]\t")
#define __MARK_DEBUG    __PRINT("[DEBG]\t")

#define __PRINT_SP     __PRINT(" ")
#define __PRINT_LINE   __PRINTLN("========================================")

////////////////////////////////////////

#define LOGERROR(x)         if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __MARK_ERROR; __PRINTLN(x); __GIVE_SEM; }
#define LOGERROR_LINE(x)    if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __MARK_ERROR; __PRINTLN(x); __PRINT_LINE; __GIVE_SEM; }
#define LOGERROR0(x)        if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __PRINT(x); __GIVE_SEM; }
#define LOGERROR1(x,y)      if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __MARK_ERROR; __PRINT(x); __PRINT_SP; __PRINTLN(y); __GIVE_SEM; }
#define LOGERROR2(x,y,z)    if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __MARK_ERROR; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINTLN(z); __GIVE_SEM; }
#define LOGERROR3(x,y,z,w)  if(_LOG_VERBOSITY_>0) { __TAKE_SEM; __MARK_ERROR; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINT(z); __PRINT_SP; __PRINTLN(w); __GIVE_SEM; }

////////////////////////////////////////

#define LOGWARN(x)          if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __MARK_WARN; __PRINTLN(x); __GIVE_SEM; }
#define LOGWARN_LINE(x)     if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __MARK_WARN; __PRINTLN(x); __PRINT_LINE; __GIVE_SEM; }
#define LOGWARN0(x)         if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __PRINT(x); __GIVE_SEM; }
#define LOGWARN1(x,y)       if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __MARK_WARN; __PRINT(x); __PRINT_SP; __PRINTLN(y); __GIVE_SEM; }
#define LOGWARN2(x,y,z)     if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __MARK_WARN; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINTLN(z); __GIVE_SEM; }
#define LOGWARN3(x,y,z,w)   if(_LOG_VERBOSITY_>1) { __TAKE_SEM; __MARK_WARN; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINT(z); __PRINT_SP; __PRINTLN(w); __GIVE_SEM; }

////////////////////////////////////////

#define LOGINFO(x)          if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __MARK_INFO; __PRINTLN(x); __GIVE_SEM; }
#define LOGINFO_LINE(x)     if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __MARK_INFO; __PRINTLN(x); __PRINT_LINE; __GIVE_SEM; }
#define LOGINFO0(x)         if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __PRINT(x); __GIVE_SEM; }
#define LOGINFO1(x,y)       if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __MARK_INFO; __PRINT(x); __PRINT_SP; __PRINTLN(y); __GIVE_SEM; }
#define LOGINFO2(x,y,z)     if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __MARK_INFO; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINTLN(z); __GIVE_SEM; }
#define LOGINFO3(x,y,z,w)   if(_LOG_VERBOSITY_>2) { __TAKE_SEM; __MARK_INFO; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINT(z); __PRINT_SP; __PRINTLN(w); __GIVE_SEM; }

////////////////////////////////////////

#define LOGDEBUG(x)         if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __MARK_DEBUG; __PRINTLN(x); __GIVE_SEM; }
#define LOGDEBUG_LINE(x)    if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __MARK_DEBUG; __PRINTLN(x); __PRINT_LINE; __GIVE_SEM; }
#define LOGDEBUG0(x)        if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __PRINT(x); __GIVE_SEM; }
#define LOGDEBUG1(x,y)      if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __MARK_DEBUG; __PRINT(x); __PRINT_SP; __PRINTLN(y); __GIVE_SEM; }
#define LOGDEBUG2(x,y,z)    if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __MARK_DEBUG; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINTLN(z); __GIVE_SEM; }
#define LOGDEBUG3(x,y,z,w)  if(_LOG_VERBOSITY_>3) { __TAKE_SEM; __MARK_DEBUG; __PRINT(x); __PRINT_SP; __PRINT(y); __PRINT_SP; __PRINT(z); __PRINT_SP; __PRINTLN(w); __GIVE_SEM; }

////////////////////////////////////////

#define SHOW_TASK_STARTED LOGINFO3("[", __FUNCTION__, "]\t[+] Started @ms->", millis())
#define SHOW_TASK_STOPPED LOGINFO3("[", __FUNCTION__, "]\t[-] Stopped @ms->", millis())

void __flush_queue_task(void *task_param)
{
    static char output_buff;
    __debug_ouput_queue = xQueueCreate(__CONFIG_TSPRINTF_QUEUE_SIZE, sizeof(char));
    vSemaphoreCreateBinary(__debug_ouput_sem);
    vSemaphoreCreateBinary(__sequence_sem);
    xSemaphoreTake(__debug_ouput_sem, portMAX_DELAY);
    __SER_PERPH_HANDLE.setRX(1);
    __SER_PERPH_HANDLE.setTX(0);
    __SER_PERPH_HANDLE.begin(__SER_PERPH_BAUDRATE);
    xSemaphoreGive(__debug_ouput_sem);
    // __SER_PERPH_HANDLE.ignoreFlowControl(true);
    while (true)
    {
        xSemaphoreTake(__debug_ouput_sem, portMAX_DELAY);
        if(xQueueReceive(__debug_ouput_queue, &output_buff, portMAX_DELAY)){
            __SER_PERPH_HANDLE.write(output_buff);
        }
        xSemaphoreGive(__debug_ouput_sem);
    }
}

static void init_threadsafe_debug_prints()
{
    vSemaphoreCreateBinary(__sequence_sem);
    __SER_PERPH_HANDLE.begin(__SER_PERPH_BAUDRATE);
    // xTaskCreate(__flush_queue_task, "__flush_queue_task", 2048, NULL, 3, NULL);
}

#endif
