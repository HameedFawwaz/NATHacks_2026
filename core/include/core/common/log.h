#ifndef CORE_COMMON_LOG_H
#define CORE_COMMON_LOG_H

#include <stdio.h>

#define CORE_LOG_INFO(fmt, ...)  fprintf(stderr, "[INFO] " fmt "\n", ##__VA_ARGS__)
#define CORE_LOG_WARN(fmt, ...)  fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#define CORE_LOG_ERROR(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

#endif /* CORE_COMMON_LOG_H */
