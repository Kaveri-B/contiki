/** @addtogroup RNG_Driver RNG Driver
*  @{
*/
/*!
 *****************************************************************************
 @file    adi_rng.h
 @brief   Random Number Generator Device Implementations for ADuCM302x
 @version $Revision: 29156 $
 @date    $Date: 2014-12-11 04:55:14 -0500 (Thu, 11 Dec 2014) $
 -----------------------------------------------------------------------------
Copyright (c) 2012-2014 Analog Devices, Inc.

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

*****************************************************************************/

/*! \addtogroup RNG_Driver RNG Driver
 *  Random Number Generator Device Driver
 *  @{
 */

#ifndef __ADI_RNG_H__
#define __ADI_RNG_H__

#include <adi_processor.h>

#ifndef __ADUCM30xx__
#error "Not supported processor"
#endif

#include <adi_rng_config.h>
#include  <services/int/adi_int.h>
#include <adi_types.h>

/* TODO: Include and other things above will be modified as the device files finalizes */

/* C++ linkage */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*!
 * \enum ADI_RNG_RESULT
 * Random Number Generator API return codes
 */
typedef enum
{
    ADI_RNG_SUCCESS = 0,                /*!<  No Error, API suceeded */
    ADI_RNG_ERR_UNKNOWN_ERROR,           /*!<  An unknown error was detected */
    ADI_RNG_ERR_ALREADY_INITIALIZED,    /*!<  RNG is already initialized */
    ADI_RNG_ERR_INVALID_PARAM,          /*!<  Invalid function parameter */
    ADI_RNG_ERR_BAD_DEV_HANDLE,         /*!<  Invalid device handle passed */
    ADI_RNG_ERR_BAD_DEVICE_NUM,         /*!<  Asking to initialize an unknown instance */
    ADI_RNG_ERR_NOT_INITIALIZED,        /*!<  RNG not yet initialized */
    ADI_RNG_ERR_INVALID_STATE           /*!<  Operation failed since the device is in an invalid state */
} ADI_RNG_RESULT;

/*!
 * \enum ADI_RNG_EVENT
 * Random Number Generator callback events
 */
typedef enum
{
    ADI_RNG_EVENT_RNG_READY,        /*!< Random number ready event */
    ADI_RNG_EVENT_STUCK             /*!< The ring oscillator got stuck event */
} ADI_RNG_EVENT;


/*! The amount of application supplied memory required by the RNG driver */
#define ADI_RNG_MEMORY_SIZE        (12u)


/*! RNG Device handle typedef. */
typedef void* ADI_RNG_HANDLE;

/*================ E X T E R N A L S ==================*/

/*
 * RNG API
 */

/* Open a random number generator device */
extern ADI_RNG_RESULT adi_rng_Open(
                            uint32_t             const  nDeviceNum,
                            void*                const  pMemory,
                            uint32_t             const  MemorySize,
                            ADI_RNG_HANDLE*      const  ppDevice
                            );

/* Close the RNG Device */
extern ADI_RNG_RESULT adi_rng_Close(ADI_RNG_HANDLE hDevice);

/* Enable/Disable the device */
extern ADI_RNG_RESULT adi_rng_Enable (
                                      ADI_RNG_HANDLE const hDevice,
                                      bool_t         const bFlag
                                      );
/* Enable/Disable buffering */
extern ADI_RNG_RESULT adi_rng_EnableBuffering (
                                               ADI_RNG_HANDLE const hDevice,
                                               bool_t         const bFlag
                                               );

/* Set the sample length */
extern ADI_RNG_RESULT adi_rng_SetSampleLen (
                                            ADI_RNG_HANDLE const hDevice,
                                            uint16_t       const nLenPrescaler,
                                            uint16_t       const nLenReload
                                            );

/* Get whether the random number is ready */
extern ADI_RNG_RESULT adi_rng_GetRdyStatus (
                                            ADI_RNG_HANDLE const hDevice,
                                            bool_t*        const pbFlag
                                            );

/* Get whether the ring oscillator output is stuck or not */
extern ADI_RNG_RESULT adi_rng_GetStuckStatus (
                                              ADI_RNG_HANDLE const hDevice,
                                              bool_t*        const pbFlag
                                              );

/* Get the random number */
extern ADI_RNG_RESULT adi_rng_GetRngData (
                                          ADI_RNG_HANDLE const hDevice,
                                          uint32_t*      const pRegData
                                          );

/* Get the oscillator count */
extern ADI_RNG_RESULT adi_rng_GetOscCount (
                                           ADI_RNG_HANDLE const hDevice,
                                           uint32_t*      const pOscCount
                                           );

/* Get the oscillator count difference value */
extern ADI_RNG_RESULT adi_rng_GetOscDiff (
                                          ADI_RNG_HANDLE const hDevice,
                                          uint32_t       const nIndex,
                                          uint8_t*       const pOscDiff
                                          );

/* Register a callback */
extern ADI_RNG_RESULT adi_rng_RegisterCallback (
                                                ADI_RNG_HANDLE  hDevice,
                                                ADI_CALLBACK    cbFunc,
                                                void           *pCBParam
                                                );

/* Retrieve the current RNG sample length prescale and reload value configured in the device. */
extern ADI_RNG_RESULT adi_rng_GetSampleLen (
                                            ADI_RNG_HANDLE const hDevice,
                                            uint16_t*      const pLenPrescaler,
                                            uint16_t*      const pLenReload
                                            );

/* C++ linkage */
#ifdef __cplusplus
}
#endif

#endif /* include guard */

/*
** EOF
*/

/*@}*/
/*@}*/
