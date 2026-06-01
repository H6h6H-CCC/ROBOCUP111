#include "stm32f4xx.h"
#include <stdint.h>

/*
 * 0: disabled
 * 1: semihosting to ST-LINK TCP console
 *
 * USART3 is dedicated to the servo bus, so debug printf must not share it.
 */
volatile uint32_t g_debug_io_mode = 0U;

static uint32_t DebugIO_IsDebuggerAttached(void)
{
    return ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) ? 1U : 0U;
}

static void DebugIO_SemihostWriteChar(char ch)
{
    uintptr_t arg = (uintptr_t)&ch;
    __asm volatile (
        "mov r0, %0\n"
        "mov r1, %1\n"
        "bkpt 0xAB\n"
        :
        : "r"(0x03U), "r"(arg)
        : "r0", "r1", "memory");
}

int __io_putchar(int ch)
{
    if (ch == '\n')
    {
        if ((g_debug_io_mode == 1U) && (DebugIO_IsDebuggerAttached() != 0U))
        {
            DebugIO_SemihostWriteChar('\r');
        }
    }

    if ((g_debug_io_mode == 1U) && (DebugIO_IsDebuggerAttached() != 0U))
    {
        DebugIO_SemihostWriteChar((char)ch);
    }

    return ch;
}
