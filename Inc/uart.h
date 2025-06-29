#ifndef __UART_H
#define __UART_H	

#ifdef __cplusplus 
extern "C"{
#endif

#include "main.h"
#include "string.h"

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

extern void UART_Init(void);
extern void UART_Send_Char(char ch);
extern void UART_Send_String(char *str);

#ifdef __cplusplus 
}
#endif

#endif