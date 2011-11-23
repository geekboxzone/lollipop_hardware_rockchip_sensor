/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: log_printf_linux.c 4894 2011-02-28 23:01:53Z prao $ 
 *
 ******************************************************************************/
 
/**
 * @addtogroup MPL_LOG
 *
 * @{
 *      @file   log_printf.c
 *      @brief  printf replacement for _MLWriteLog.
 */

#include <stdio.h>
#include "log.h"

int _MLWriteLog (const char * buf, int buflen)
{
    return fputs(buf, stdout);
}

/**
 * @}
 */

