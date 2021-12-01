/*
 * common.c
 *
 *  Created on: Oct. 2, 2020
 *      Author: Michael
 */

#include "main.h"
#include "app_common.h"

char *writeInt32ToBuf(char *ptr, int32_t val, int8_t dec, int8_t dot, char delimiter)
{
    int32_t value = val > 0 ? val : -val;

    *--ptr = delimiter;
    while (value > 0 || dec > 0)
    {
        ldiv_t res = ldiv(value, 10);
        *--ptr = res.rem + '0';
        value = res.quot;
        if (--dec == 0 && dot)
        {
            *--ptr = '.';
        }
    }
    if (*ptr == '.' || *ptr == delimiter)
    {
        *--ptr = '0';
    }
    if (val < 0)
    {
        *--ptr = '-';
    }

	return ptr;
}
