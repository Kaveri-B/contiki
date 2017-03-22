/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */


#include "lib/random.h"
#include "sys/clock.h"

#include <stdlib.h>
#include <drivers/rng/adi_rng.h>

/* RNG Device number */
#define RNG_DEV_NUM              (0u)

/* Sample Len Prescaler value to be set */
#define RNG_DEV_LEN_PRESCALER    (1u)

/* Sample Len Reload value to be set */
#define RNG_DEV_LEN_RELOAD       (256u)

/* RNG Device Handle */
ADI_RNG_HANDLE     hRngDevice;

/* Memory to handle CRC Device */
uint8_t       RngDevMem[ADI_RNG_MEMORY_SIZE];
/*---------------------------------------------------------------------------*/
void random_init(unsigned short seed)
{
#if 0 
  srand(seed);
#else
  ADI_RNG_RESULT eResult = ADI_RNG_SUCCESS; 
  
  if(eResult = adi_rng_Open(RNG_DEV_NUM,RngDevMem,sizeof(RngDevMem),&hRngDevice))
  {
    return; 
  }
  
  if( eResult = adi_rng_SetSampleLen(hRngDevice,RNG_DEV_LEN_PRESCALER,RNG_DEV_LEN_RELOAD))
  {
    return; 
  }
  if(eResult = adi_rng_EnableBuffering(hRngDevice,true)) //Getting 32 bit random number
  {
    return;
  }
#endif  
}
/*---------------------------------------------------------------------------*/
unsigned short random_rand(void)
{
#if 0
  /* In gcc int rand() uses RAND_MAX and long random() uses RANDOM_MAX=0x7FFFFFFF */
  /* RAND_MAX varies depending on the architecture */

  return (unsigned short)rand();
  
#else  
  ADI_RNG_RESULT eResult = ADI_RNG_SUCCESS; 
  bool_t bRNGRdy = 0;
  uint32_t nRandomNum;
  
  adi_rng_Enable(hRngDevice, true);
  
  do
  {      
    adi_rng_GetRdyStatus(hRngDevice, &bRNGRdy);
  }while(!bRNGRdy);
  
  adi_rng_GetRngData(hRngDevice, &nRandomNum);
  
  //adi_rng_Close(hRngDevice);
  
  return (unsigned short)nRandomNum;
#endif
}
/*---------------------------------------------------------------------------*/
