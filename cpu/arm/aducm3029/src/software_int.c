/*! *****************************************************************************
 * @file:    software_int.c
 * @brief:   Provides Software interrupt functionalities.
 * @details: 
 * @version: $Revision:
 * @date:    $Date:
 -----------------------------------------------------------------------------
Copyright (c) 2010-2014 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

********************************************************************************/

/*******************************************************************************
* File inclusion
*******************************************************************************/

#include "common.h"
#include <ssdd_common/common_def.h>

ADI_INT_HANDLER(SPORT0A_Int_Handler);
ADI_INT_HANDLER(SPORT0B_Int_Handler);
ADI_INT_HANDLER(Beep_Int_Handler);
void HIF_SoftInterruptHandler();
void MAC_SoftInterruptHandler();
void TRX_SoftInterruptHandler();

void software_interrupt_init()
{
    ADI_INSTALL_HANDLER(SPORT_A_EVT_IRQn, SPORT0A_Int_Handler);
    ADI_INSTALL_HANDLER(SPORT_B_EVT_IRQn, SPORT0B_Int_Handler);
    ADI_INSTALL_HANDLER(BEEP_EVT_IRQn, Beep_Int_Handler);
    
    ADI_ENABLE_INT(SPORT_A_EVT_IRQn);
    ADI_ENABLE_INT(SPORT_B_EVT_IRQn);
    ADI_ENABLE_INT(BEEP_EVT_IRQn);
}

ADI_INT_HANDLER(SPORT0A_Int_Handler)
{
     //MAC_SoftInterruptHandler();
}

ADI_INT_HANDLER(SPORT0B_Int_Handler)
{
     //HIF_SoftInterruptHandler();
}

ADI_INT_HANDLER(Beep_Int_Handler)
{
    //TRX_SoftInterruptHandler();
}
