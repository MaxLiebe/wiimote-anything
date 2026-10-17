#pragma once
#include <stdio.h>
extern int g_verbose;
#define ESP_LOG_(lvl, tag, fmt, ...) do { if (g_verbose) printf("%s %s: " fmt "\n", lvl, tag, ##__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, fmt, ...) ESP_LOG_("E", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) ESP_LOG_("W", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) ESP_LOG_("I", tag, fmt, ##__VA_ARGS__)
