#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#endif

// Mock ESP Timer
static inline int64_t esp_timer_get_time(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (int64_t)((count.QuadPart * 1000000) / freq.QuadPart);
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
#endif
}

#ifndef _WIN32
/* Match Windows Sleep(ms) for the PC simulator. */
static inline void Sleep(unsigned long ms)
{
    usleep(ms * 1000UL);
}
typedef unsigned long DWORD;
#endif

// Mock FreeRTOS Mutexes
typedef void* SemaphoreHandle_t;
#ifndef NULL
#define NULL ((void *)0)
#endif
#define portMAX_DELAY 0xFFFFFFFF
#define pdTRUE 1

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    return (SemaphoreHandle_t)1; // Dummy non-null pointer
}
static inline int xSemaphoreTake(SemaphoreHandle_t m, uint32_t delay) {
    (void)m;
    (void)delay;
    return pdTRUE;
}
static inline void xSemaphoreGive(SemaphoreHandle_t m) {
    (void)m;
}
