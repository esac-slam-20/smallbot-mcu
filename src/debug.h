#pragma once

#ifdef DEBUG
#include "stdio.h"
#define PRINT(x, ...) printf(x, ##__VA_ARGS__)
#else
#define PRINT(x, ...)
#endif