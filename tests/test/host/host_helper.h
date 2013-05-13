/*
 * host_helper.h
 *
 *  Created on: May 13, 2013
 *      Author: hathach
 */

/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2012, hathach (tinyusb.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the tiny usb stack.
 */

#include "common/common.h"
#include "host/usbh.h"
#include "host/usbh_hcd.h"

static inline void class_init_expect(void)
{
  hidh_init_Expect();
  //TODO update more classes
}

static inline void helper_usbh_init(void)
{
  osal_semaphore_create_IgnoreAndReturn( (osal_semaphore_handle_t) 0x1234);
  osal_task_create_IgnoreAndReturn(TUSB_ERROR_NONE);
  osal_queue_create_IgnoreAndReturn( (osal_queue_handle_t) 0x4566 );
  class_init_expect();

  usbh_init();
}

static inline void helper_usbh_device_emulate(uint8_t dev_addr, uint8_t hub_addr, uint8_t hub_port, uint8_t hostid, tusb_speed_t speed)
{
  usbh_devices[dev_addr].core_id  = hostid;
  usbh_devices[dev_addr].hub_addr = hub_addr;
  usbh_devices[dev_addr].hub_port = hub_port;
  usbh_devices[dev_addr].speed    = speed;
  usbh_devices[dev_addr].state    = dev_addr ? TUSB_DEVICE_STATE_CONFIGURED : TUSB_DEVICE_STATE_UNPLUG;
}






