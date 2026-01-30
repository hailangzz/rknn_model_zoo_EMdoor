#pragma once

// ========================
// 环境配置
// ========================
#define ANDROID_ENV 1   // 0: Linux, 1: Android

// ========================
// 日志总开关
// ========================
// 1: 打开日志
// 0: 关闭所有日志（编译期直接移除，零开销）
#define LOG_ENABLE 0


#if ANDROID_ENV

    #include <android/log.h>

    #define LOG_TAG "HandDetectNative"

    #if LOG_ENABLE
        #define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
        #define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
    #else
        #define LOGI(...)
        #define LOGE(...)
    #endif

#else   // ===== Linux =====

    #include <iostream>
    #include <cstdio>

    #if LOG_ENABLE
        #define LOGI(fmt, ...) \
            do { \
                char buf[512]; \
                snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
                std::cout << "[INFO] " << buf << std::endl; \
            } while(0)

        #define LOGE(fmt, ...) \
            do { \
                char buf[512]; \
                snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
                std::cerr << "[ERROR] " << buf << std::endl; \
            } while(0)
    #else
        #define LOGI(...)
        #define LOGE(...)
    #endif

#endif
