/**
 * \file
 *
 * \brief USB configuration file
 *
 * Copyright (c) 2009 - 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef _CONF_USB_H_
#define _CONF_USB_H_

#include "compiler.h"

#warning You must refill the following definitions with a correct values

/**
 * USB Device Configuration
 * @{
 */

//! Device definition (mandatory)
#define  USB_DEVICE_VENDOR_ID             USB_VID_ATMEL
#define  USB_DEVICE_PRODUCT_ID            0x6800 //USB_PID_ATMEL_ASF_HIDGENERIC
#define  USB_DEVICE_MAJOR_VERSION         1
#define  USB_DEVICE_MINOR_VERSION         0
#define  USB_DEVICE_POWER                 100 // Consumption on Vbus line (mA)
#define  USB_DEVICE_ATTR                  \
	(USB_CONFIG_ATTR_BUS_POWERED)
//		(USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
// (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)

//! USB Device string definitions (Optional)
 #define  USB_DEVICE_MANUFACTURE_NAME      "WolfWare Tech"
 #define  USB_DEVICE_PRODUCT_NAME          "Howler Controller"
 #define  USB_DEVICE_SERIAL_NAME           "000001" // Disk SN for MSC
 #define  USB_DEVICE_MAJOR_VERSION			1
 #define  USB_DEVICE_MINOR_VERSION			1

/**
 * Device speeds support
 * @{
 */
//! To define a Low speed device
//#define  USB_DEVICE_LOW_SPEED

//! To authorize the High speed
#if (UC3A3||UC3A4)
//#define  USB_DEVICE_HS_SUPPORT
#endif
//@}

/**
 * USB Device Callbacks definitions (Optional)
 * @{
 */
// #define  UDC_VBUS_EVENT(b_vbus_high)      user_callback_vbus_action(b_vbus_high)
// extern void user_callback_vbus_action(bool b_vbus_high);
#define  UDC_SOF_EVENT()                  user_callback_sof_action()
extern void user_callback_sof_action(void);
//#define  UDC_SUSPEND_EVENT()              user_callback_suspend_action()
//extern void user_callback_suspend_action(void);
//#define  UDC_RESUME_EVENT()               user_callback_resume_action()
//extern void user_callback_resume_action(void);
//! Mandatory when USB_DEVICE_ATTR authorizes remote wakeup feature
// #define  UDC_REMOTEWAKEUP_ENABLE()        user_callback_remotewakeup_enable()
// extern void user_callback_remotewakeup_enable(void);
// #define  UDC_REMOTEWAKEUP_DISABLE()       user_callback_remotewakeup_disable()
// extern void user_callback_remotewakeup_disable(void);
//! When a extra string descriptor must be supported
//! other than manufacturer, product and serial string
// #define  UDC_GET_EXTRA_STRING()
//@}

/**
 * USB Device low level configuration
 * When only one interface is used, these configurations are defined by the class module.
 * For composite device, these configuration must be defined here
 * @{
 */
//! Control endpoint size
#define  USB_DEVICE_EP_CTRL_SIZE       8

//! Number of interfaces for this device
#define  USB_DEVICE_NB_INTERFACE     5

//! Total endpoint used by all interfaces
//! Note:
//! It is possible to define an IN and OUT endpoints with the same number on XMEGA product only
//! E.g. MSC class can be have IN endpoint 0x81 and OUT endpoint 0x01
#define  USB_DEVICE_MAX_EP           6// 0 to max endpoint requested by interfaces
//@}

#define  UDI_HID_JOYSTICK_ENABLE_EXT()      true
#define  UDI_HID_JOYSTICK_DISABLE_EXT()
// #define UDI_HID_JOYSTICK_ENABLE_EXT() my_callback_joystick_enable()
// extern bool my_callback_joystick_enable(void);
// #define UDI_HID_JOYSTICK_DISABLE_EXT() my_callbackjoystick_disable()
// extern void my_callback_joystick_disable(void);
//! Endpoint numbers definition
#define  UDI_HID_JOYSTICK_EP_IN           (3 | USB_EP_DIR_IN)

//! Interface number
#define  UDI_HID_JOYSTICK_IFACE_NUMBER    1





#define  UDI_HID_JOYSTICK2_ENABLE_EXT()      true
#define  UDI_HID_JOYSTICK2_DISABLE_EXT()
// #define UDI_HID_JOYSTICK_ENABLE_EXT() my_callback_joystick_enable()
// extern bool my_callback_joystick_enable(void);
// #define UDI_HID_JOYSTICK_DISABLE_EXT() my_callbackjoystick_disable()
// extern void my_callback_joystick_disable(void);
//! Endpoint numbers definition
#define  UDI_HID_JOYSTICK2_EP_IN           (6 | USB_EP_DIR_IN)

//! Interface number
#define  UDI_HID_JOYSTICK2_IFACE_NUMBER    4





//! Interface callback definition
#define  UDI_HID_MOUSE_ENABLE_EXT()      true
#define  UDI_HID_MOUSE_DISABLE_EXT()
// #define UDI_HID_MOUSE_ENABLE_EXT() my_callback_mouse_enable()
// extern bool my_callback_mouse_enable(void);
// #define UDI_HID_MOUSE_DISABLE_EXT() my_callback_mouse_disable()
// extern void my_callback_mouse_disable(void);


//! Endpoint numbers definition
#define  UDI_HID_MOUSE_EP_IN           (4 | USB_EP_DIR_IN)

//! Interface number
#define  UDI_HID_MOUSE_IFACE_NUMBER    2



//! Interface callback definition
#define  UDI_HID_KBD_ENABLE_EXT()      true
#define  UDI_HID_KBD_DISABLE_EXT()
// #define UDI_HID_KBD_ENABLE_EXT() my_callback_keyboard_enable()
// extern bool my_callback_keyboard_enable(void);
// #define UDI_HID_KBD_DISABLE_EXT() my_callback_keyboard_disable()
// extern void my_callback_keyboard_disable(void);
#define  UDI_HID_KBD_CHANGE_LED(value)
// #define  UDI_HID_KBD_CHANGE_LED(value) my_callback_keyboard_led(value)
// extern void my_callback_keyboard_led(uint8_t value)


//! Endpoint numbers definition
#define  UDI_HID_KBD_EP_IN           (5 | USB_EP_DIR_IN)

//! Interface number
#define  UDI_HID_KBD_IFACE_NUMBER    3

//Generic HID
#define  UDI_HID_GENERIC_ENABLE_EXT()        true //main_generic_enable()
#define  UDI_HID_GENERIC_DISABLE_EXT()       //main_generic_disable()
#define  UDI_HID_GENERIC_REPORT_OUT(ptr)     usb_generic_hid_out(ptr)
#define  UDI_HID_GENERIC_SET_FEATURE(report) main_hid_set_feature(report)

#define UDI_HID_REPORT_IN_SIZE			24
#define UDI_HID_REPORT_OUT_SIZE			24
#define UDI_HID_REPORT_FEATURE_SIZE		4
#define UDI_HID_GENERIC_EP_SIZE			8

#define UDI_HID_GENERIC_EP_IN	(1 | USB_EP_DIR_IN)
#define UDI_HID_GENERIC_EP_OUT	(2 | USB_EP_DIR_OUT)

#define  UDI_HID_GENERIC_IFACE_NUMBER    0

// Example for device with cdc, msc and hid mouse interface
#define UDI_COMPOSITE_DESC_T \
	udi_hid_generic_desc_t udi_hid_generic; \
	udi_hid_joystick_desc_t udi_hid_joystick; \
	udi_hid_mouse_desc_t udi_hid_mouse; \
	udi_hid_kbd_desc_t udi_hid_kbd; \
	udi_hid_joystick2_desc_t udi_hid_joystick2; 	


//! USB Interfaces descriptor value for Full Speed
#define UDI_COMPOSITE_DESC_FS \
	.udi_hid_generic		  = UDI_HID_GENERIC_DESC, \
	.udi_hid_joystick             = UDI_HID_JOYSTICK_DESC, \
	.udi_hid_mouse             = UDI_HID_MOUSE_DESC, \
	.udi_hid_kbd              = UDI_HID_KBD_DESC, \
	.udi_hid_joystick2             = UDI_HID_JOYSTICK2_DESC

	
//! USB Interfaces descriptor value for High Speed
#define UDI_COMPOSITE_DESC_HS \
	.udi_hid_generic		  = UDI_HID_GENERIC_DESC, \
	.udi_hid_joystick             = UDI_HID_JOYSTICK_DESC, \
	.udi_hid_mouse             = UDI_HID_MOUSE_DESC, \
	.udi_hid_kbd              = UDI_HID_KBD_DESC, \
	.udi_hid_joystick2             = UDI_HID_JOYSTICK2_DESC
	


//! USB Interface APIs
#define UDI_COMPOSITE_API  \
	&udi_api_hid_generic, \
	&udi_api_hid_joystick, \
	&udi_api_hid_mouse, \
	&udi_api_hid_kbd, \
	&udi_api_hid_joystick2	

//@}


//! The includes of classes and other headers must be done at the end of this file to avoid compile error

// Example of include for interface
#include "udi_hid_generic.h"
#include "udi_hid_joystick.h"
#include "udi_hid_joystick2.h"
#include "udi_hid_kbd.h"
#include "udi_hid_mouse.h"
#include "main.h"

/* Declaration of callbacks used by USB
#include "callback_def.h"
*/

#endif // _CONF_USB_H_
