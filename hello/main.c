/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <stdio.h>

#include <alif.h>
#include <RTE_Components.h>

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
#include <retarget_config.h>
#include <retarget_init.h>
#endif

int main(void)
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    if (stdout_init() != 0) {
        while (1) {
            __WFE();
        }
    }

    printf("\r\nHello World!\r\n");
#endif

    while (1) {
        __WFE();
    }

    return 0;
}
