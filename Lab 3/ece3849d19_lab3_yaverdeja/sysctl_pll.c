//*****************************************************************************
//
// sysctl.c - Driver for the system controller.
//
// Copyright (c) 2005-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Excerpt exporting the normally static function _SysCtlFrequencyGet.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_flash.h"
#include "driverlib/cpu.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

//*****************************************************************************
//
// Calculate the system frequency from the register settings base on the
// oscillator input.
//
//*****************************************************************************
uint32_t SysCtlFrequencyGet(uint32_t ui32Xtal)
{
    uint32_t ui32Result;
    uint_fast16_t ui16F1, ui16F2;
    uint_fast16_t ui16PInt, ui16PFract;
    uint_fast8_t ui8Q, ui8N;

    //
    // Extract all of the values from the hardware registers.
    //
    ui16PFract = ((HWREG(SYSCTL_PLLFREQ0) & SYSCTL_PLLFREQ0_MFRAC_M) >>
                  SYSCTL_PLLFREQ0_MFRAC_S);
    ui16PInt = HWREG(SYSCTL_PLLFREQ0) & SYSCTL_PLLFREQ0_MINT_M;
    ui8Q = (((HWREG(SYSCTL_PLLFREQ1) & SYSCTL_PLLFREQ1_Q_M) >>
             SYSCTL_PLLFREQ1_Q_S) + 1);
    ui8N = (((HWREG(SYSCTL_PLLFREQ1) & SYSCTL_PLLFREQ1_N_M) >>
             SYSCTL_PLLFREQ1_N_S) + 1);

    //
    // Divide the crystal value by N.
    //
    ui32Xtal /= (uint32_t)ui8N;

    //
    // Calculate the multiplier for bits 9:5.
    //
    ui16F1 = ui16PFract / 32;

    //
    // Calculate the multiplier for bits 4:0.
    //
    ui16F2 = ui16PFract - (ui16F1 * 32);

    //
    // Get the integer portion.
    //
    ui32Result = ui32Xtal * (uint32_t)ui16PInt;

    //
    // Add first fractional bits portion(9:0).
    //
    ui32Result += (ui32Xtal * (uint32_t)ui16F1) / 32;

    //
    // Add the second fractional bits portion(4:0).
    //
    ui32Result += (ui32Xtal * (uint32_t)ui16F2) / 1024;

    //
    // Divide the result by Q.
    //
    ui32Result = ui32Result / (uint32_t)ui8Q;

    //
    // Return the resulting PLL frequency.
    //
    return(ui32Result);
}
