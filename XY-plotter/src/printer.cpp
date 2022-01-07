#include "printer.h"
#include "LpcUart.h"
#include <stdio.h>
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */

LpcPinMap none = { .port = -1, .pin = -1}; // unused pin has negative values in it
LpcPinMap txpin1 = { .port = 0, .pin = 18 }; // transmit pin
LpcPinMap rxpin1 = { .port = 0, .pin = 13 }; // receive pin
LpcUartConfig cfg1 = {
    .pUART = LPC_USART0,
    .speed = 115200,
    .data = UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1,
    .rs485 = false,
    .tx = txpin1,
    .rx = rxpin1,
    .rts = none,
    .cts = none
};


namespace Printer {
    LpcUart *debug_uart = new LpcUart(cfg1); // Global uart communication variable

    // Need to create this function since for to be used in a macro, since the preprocessor cannot use debug_uart variable yet
    int uart_print(const char *str) {
        return Printer::debug_uart->write(str);
    }

// printf style printing, requires a callback for the actual printing of the "string"
    int arg_print(int (*callback) (const char *), const char *format, ...) {
        char buffer [ARG_BUFFER_SIZE];
        va_list argptr;
        va_start(argptr, format);
        vsnprintf (buffer, ARG_BUFFER_SIZE, format, argptr);
        va_end(argptr);
        return callback(buffer);
    }
}
