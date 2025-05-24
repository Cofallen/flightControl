#include "reg52.h"
#include <stdio.h>

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;

typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;


void Delay_ms(uint ms)
{
    uint i, j;
    for(i=0; i<ms; i++)
        for(j=0; j<110; j++);
}
