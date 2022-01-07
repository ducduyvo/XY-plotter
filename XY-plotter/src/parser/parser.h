#ifndef PARSER_H
#define PARSER_H
#include "FreeRTOS.h"
#include "queue.h"

void parseCode(const char *s, QueueHandle_t &queue);

bool m1ExtractData(const char *str);
bool m2ExtractData (const char *str);
bool m4ExtractData (const char *str);
bool m5ExtractData(const char *str);
bool m10ExtractData (const char *str);
bool m11ExtractData (const char *str);
bool g1ExtractData (const char *str);
bool g28ExtractData (const char *str);


#endif /* PARSER_H */
