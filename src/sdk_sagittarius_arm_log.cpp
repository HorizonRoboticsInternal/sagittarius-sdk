/*
 * @Descripttion:
 * @version: 1.00
 * @Author: ShuDong.Hong@nxrobo.com
 * @Company: NXROBO (深圳创想未来机器人有限公司)
 * @Date: 2022-03-29 17:55:41
 * @LastEditors: ShuDong.Hong@nxrobo.com
 * @LastEditTime: 2022-03-30 11:53:40
 */

#include <sdk_sagittarius_arm/sdk_sagittarius_arm_log.h>
#include <stdarg.h>
#include <stdio.h>
int log_level = 3;
char log_print_buff[1024] = {};

/// @brief log_print
/// 输出指定级别的日志，第一个参数时日志级别标签，后面参数使用与 printf 一致
/// @param level - 日志级别标签，分别是
/// LOG_TYPE_ERROR、LOG_TYPE_WARN、LOG_TYPE_INFO、LOG_TYPE_DEBUG
/// @param log_format, ... - 输出字符串格式与参数，使用方法与 printf 一致
void log_print(int level, char const* log_format, ...)
{
    if (level > log_level) return;

    // char *szBuf = new char[4096];
    va_list args;

    va_start(args, log_format);
    vsnprintf(log_print_buff, 1024, log_format, args);
    va_end(args);

    // if (level < log_level)
    //     return;
    switch (level)
    {
    case LOG_TYPE_DEBUG:
        printf("\e[0;32m"
               "DEBUG: "
               "\e[0;37m"
               "%s"
               "\e[0m",
               log_print_buff);
        break;
    case LOG_TYPE_INFO:
        printf("\e[0;37m"
               "INFO: "
               "%s"
               "\e[0m",
               log_print_buff);
        break;
    case LOG_TYPE_WARN:
        printf("\e[0;33m"
               "WARN: "
               "%s"
               "\e[0m",
               log_print_buff);
        break;
    case LOG_TYPE_ERROR:
        printf("\e[1;31m"
               "ERROR: "
               "%s"
               "\e[0m",
               log_print_buff);
        break;

    default:
        break;
    };
}

/// @brief log_set_level 设置输出日志的级别
/// @param level - 日志级别标签, 4: ERROR, WARNNING, INFO, DEBUG
///                             3: ERROR, WARNNING, INFO
///                             2: ERROR, WARNNING
///                             1: ERROR
///                             0: None
void log_set_level(int level)
{
    if (level >= 0 && level <= 4) log_level = level;
}
