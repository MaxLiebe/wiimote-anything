#pragma once
#include <stdint.h>
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int prio, TaskHandle_t *h, int core);
void vTaskDelete(TaskHandle_t h);
