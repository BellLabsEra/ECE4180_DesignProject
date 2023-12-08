/*
 * Check Mbed revision
 *
 * Copyright (c) 2019,'20 Kenji Arai / JH1PJL
 *  http://www7b.biglobe.ne.jp/~kenjia/
 *  https://os.mbed.com/users/kenjiArai/
 *      Created:    July      17th, 2019
 *      Revised:    August     1st, 2020
 */

#include "mbed.h"
 
//    RUN ONLY ON mbed-os-6.2.0
//      https://github.com/ARMmbed/mbed-os/releases/tag/mbed-os-6.2.0
#if (MBED_MAJOR_VERSION == 6) &&\
    (MBED_MINOR_VERSION == 2) &&\
    (MBED_PATCH_VERSION == 0)
#else
#   error "Please use mbed-os-6.2.0"
#endif

void print_revision(void)
{
    printf("MBED_MAJOR_VERSION = %d, ", MBED_MAJOR_VERSION);
    printf("MINOR = %d, ", MBED_MINOR_VERSION);
    printf("PATCH = %d\r\n", MBED_PATCH_VERSION);
}
