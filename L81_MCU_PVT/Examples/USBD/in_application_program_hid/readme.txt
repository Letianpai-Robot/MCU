/*!
    \file    readme.txt
    \brief   description of the USB HID device for IAP demo

    \version 2021-08-04, V1.0.0, firmware for GD32L23x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

  This example is based on the GD32L233R-EVAL-V1.0 board, it provides a description of 
how to use the USBD peripheral.

  The GD32 device is enumerated as an USB Custom HID device for IAP, that uses the 
native PC Host HID driver.

  To test the demo, you need a configuration hex image or bin image. The hex image
or the bin image should set application address at 0x8008000. You can refer to 
"Utilities/Binary/DFU_Images" folder and use the hex images or bin images in it.

  To test the demo, you need to:
    - Download the "USB IAP Client" PC applet
    - Start the "USB IAP Client" PC applet and connect GD32 USB to PC (Note: maybe
      the driver installion is failed in the first time, try to reset again.)
    - The device should be detected and shown in the PC applet
    - Select the hex image and download it to the flash
    - After each device reset, the mcu will run the new application
    - After each device reset, hold down the WAKEUP key on the GD32L233R-EVAL board 
      to enter IAP mode

  Note: considering difference of compiler, MDK project should be opened by KEIL Version 5.25 above

