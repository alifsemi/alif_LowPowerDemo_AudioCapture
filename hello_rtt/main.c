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

int main(void)
{
    printf("\r\nHello World!\r\n");

    while (1) {
        __WFE();
    }

    return 0;
}
