#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h" // 按你的芯片改：f1/f4/h7...

/* 用 volatile 保存信息，卡死后在调试器里能看到 */
volatile const char *g_fault_file = 0;
volatile int g_fault_line         = 0;
volatile const char *g_fault_task = 0;
volatile uint32_t g_free_heap     = 0;

/* 1) 栈溢出 hook：最重要 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    g_fault_task = pcTaskName; // 记录任务名
    g_free_heap  = xPortGetFreeHeapSize();

    taskDISABLE_INTERRUPTS();

    __BKPT(0); // 进断点（连着调试器会停在这里）
    while (1) {
        // 也可以在这里翻转 LED，但别用 HAL_Delay（它依赖 tick）
    }
}

/* 2) malloc 失败 hook：内存不够/碎片导致 new/malloc 失败会进来 */
void vApplicationMallocFailedHook(void)
{
    g_fault_task = pcTaskGetName(NULL);
    g_free_heap  = xPortGetFreeHeapSize();

    taskDISABLE_INTERRUPTS();
    __BKPT(0);
    while (1) {}
}

/* 3) assert：把 configASSERT 引发的问题定位到具体文件行号 */
void vAssertCalled(const char *file, int line)
{
    g_fault_file = file;
    g_fault_line = line;
    g_fault_task = pcTaskGetName(NULL);
    g_free_heap  = xPortGetFreeHeapSize();

    taskDISABLE_INTERRUPTS();
    __BKPT(0);
    while (1) {}
}
