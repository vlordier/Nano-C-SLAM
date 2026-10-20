#ifndef NCS_TEST_DEBUG_H
#define NCS_TEST_DEBUG_H

#include <stdio.h>

#define DEBUG_PRINT(...) ((void)fprintf(stderr, __VA_ARGS__))

#endif
