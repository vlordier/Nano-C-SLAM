#ifndef TEST_DEBUG_H
#define TEST_DEBUG_H

void test_debug_print(const char *format, ...);
#define DEBUG_PRINT(...) test_debug_print(__VA_ARGS__)

#endif
