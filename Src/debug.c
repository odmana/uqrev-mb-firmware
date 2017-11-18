/*
 * debug.c
 *
 *  Created on: 16Nov.,2017
 *      Author: uqracing
 */

// #include <stdio.h>
#include <stdarg.h>
#include <debug.h>

#ifdef DEBUG
extern void initialise_monitor_handles(void); // Source: http://alphaloewe.com/2017/01/24/enable-semi-hosting-with-openstm32-system-workbench/
#endif

void debug_init() {
#ifdef DEBUG
  initialise_monitor_handles();
#endif
}

void debug_log(const char* format, ...) {
#ifdef DEBUG
	printf("DEBUG: ");
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
	printf("\n");
#endif
}

void debug_log_n(const char* format, ...) {
#ifdef DEBUG
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
#endif
}

