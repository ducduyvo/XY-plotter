#ifndef PRINTER_H_
#define PRINTER_H_
#include "ITM_write.h"
#include "LpcUart.h"
#define ARG_BUFFER_SIZE 128

// 1 Disables all uart and ITM debug printing so this option is great to use for release version
#define RELEASE 0

namespace Printer {
    extern LpcUart* debug_uart;
    int uart_print(const char *str);
    int arg_print(int (*callback) (const char *), const char *format, ...);
}

// Global print functions that behave the same as printf, that can be used project wide
// for debugging
#if RELEASE == 1
	#define UART_print(...)
	#define ITM_print(...)
#else
	#define UART_print(...) Printer::arg_print(Printer::uart_print, __VA_ARGS__) // uart_print write is guarded
    #define ITM_print(...)  Printer::arg_print(ITM_write, __VA_ARGS__) // Note ITM_write is not guarded
#endif /* RELEASE */


#endif /* PRINTER_H_ */

