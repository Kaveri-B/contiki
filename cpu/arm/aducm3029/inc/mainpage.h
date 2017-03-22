/*!
 *****************************************************************************
 * @file:    mainpage.h
 * @brief:   CMSIS Cortex-M3 Main Documentation Page for
 *           ADI ADuCM302x Device Series
 * @version: $Revision: 29310 $
 * @date:    $Date: 2014-12-18 03:31:03 -0500 (Thu, 18 Dec 2014) $
 *-----------------------------------------------------------------------------
 *
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

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/* "Mainpage" is the main page for the DoxyGen output.
   Use it to present overview of the chip, drivers, release notes, etc.
   Use the "\section" directive to create section headers.
   Use the "\htmlinclude" directive to import html documentation.
 */

/*! \mainpage ADuCM302x_EZ-KIT_Lite Device Drivers API Reference Manual Overview
   \n
   \section ChipIntro ADuCM302x Introduction
   \n
\n The ADuCM320x is an ultra low-power integrated mixed-signal micro-controller system for processing, control and connectivity. 
\n The MCU system is based on an ARM Cortex-M3 processor, a collection of digital peripherals, embedded SRAM and flash memory, 
\n and an analog subsystem which provides clocking, reset and power management capability along with ADC. 
 
   \n
   \n
    System features include (device drivers are not provided for all system peripherals):
   \n    26 MHz ARM Cortex-M3 processor
   \n    Up to 256 KB of embedded flash memory
   \n    64 KB system SRAM
   \n    Power management unit (PMU)
   \n    Multilayer advanced microcontroller bus architecture(AMBA) bus matrix
   \n    Central direct memory access (DMA) controller
   \n    Beeper interface
   \n    SPORT
   \n    SPI
   \n    I2C
   \n    UART 
   \n    Crypto hardware support with AES and SHA256
   \n    Real time clock (RTC)
   \n    General-purpose wakeup and watchdog timers
   \n    Programmable general-purpose I/O pins
   \n    Power-on reset (POR) and power-supply monitor (PSM)
   \n    12-bit SAR analog-to-digital converter
   \n    True random number generator (TRNG)
   \n
   \n
      In order to support extremely low dynamic and standby power management,
      ADuCM302x provides a collection of power modes and features such as dynamic
      and software-controlled clock gating and power gating.  
   \n
   \n
   \section DriverIntro API Reference Manual
   \n This "ADuCM302x Device Drivers API Reference Manual" documents a high-level language C
   \n Programming environment providing an Application Programming Interface (API)
   \n supporting a quick and easy functional interface to the ADuCM302x peripherals.
   \n
   \n
   \n Detailed knowledge of the various controller internals (control registers,
   \n register mapping, bitmaps, etc.) is not required to make effective use of the
   \n peripherals; simply open the device driver and use it.  Basic familiarity with
   \n the Cortex M3 architecture, relevant communications protocols and the third-party
   \n development tools is assumed.
   \n
   \n
   \n Various example  are included to demonstrate use of the Device Driver programming model.
   \n
   \n
   \section SettingStarted Getting Started Guide
   \n Please read the "ADuCM302x Device Drivers Getting Started Guide.pdf" for general
   \n information on the ADuCM302x Device Drivers installation and how to use it.
   \n
   \n
   \section ReleaseNotes Release Notes
   \n
   \n Please read the "ADuCM302x Device Drivers Release Notes.pdf" for the latest
   \n information on the current release describing features, limitations, requirements,
   \n supported drivers, usage notes, examples, tests, etc.
   \n
   \n
   \section UserGuide Hardware User Guide and Device Data Sheet
   \n Please read the latest "ADuCM302x User Guide" and device Data Sheet for device
   \n reference information and performance specifications for the ADuCM302xZ microcontroller.
   \n
   \n
   \section Support Technical or Customer Support
   \n You can reach Analog Devices, Inc. Customer Support at:

        -    Web access at
                 - http://www.analog.com/support

        -    For IAR tool chain support please visit
                 -http://www.iar.com/support 

        -    E-mail processor questions to
                 - processor.support@analog.com
                 - processor.china@analog.com (China and Taiwan only)

        -    Phone questions to 1-800-ANALOGD

        -    Contact your Analog Devices, Inc. local sales office or authorized distributor

        -    Send questions by mail to:
   \code
   Analog Devices, Inc.
   3 Technology Way
   P.O. Box 9106
   Norwood, MA 02062-9106 USA
   \endcode

 */

/*
** EOF
*/

/*@}*/
