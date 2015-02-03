/**
 * \file
 *
 * \brief USB Device Human Interface Device (HID) joystick interface.
 *
 * Copyright (c) 2013 - Josh Wolf
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

#include "conf_usb.h"
#include "usb_protocol.h"
#include "udd.h"
#include "udc.h"
#include "udi_hid.h"
#include "udi_hid_joystick.h"
#include <string.h>

/**
 * \ingroup udi_hid_mouse_group
 * \defgroup udi_hid_mouse_group_udc Interface with USB Device Core (UDC)
 *
 * Structures and functions required by UDC.
 *
 * @{
 */
bool udi_hid_joystick_enable(void);
void udi_hid_joystick_disable(void);
bool udi_hid_joystick_setup(void);
uint8_t udi_hid_joystick_getsetting(void);

//! Global structure which contains standard UDI interface for UDC
UDC_DESC_STORAGE udi_api_t udi_api_hid_joystick = {
	.enable = (bool(*)(void))udi_hid_joystick_enable,
	.disable = (void (*)(void))udi_hid_joystick_disable,
	.setup = (bool(*)(void))udi_hid_joystick_setup,
	.getsetting = (uint8_t(*)(void))udi_hid_joystick_getsetting,
	.sof_notify = NULL,
};
//@}


/**
 * \ingroup udi_hid_joystick_group
 * \defgroup udi_hid_joystick_group_internal Implementation of UDI HID Mouse
 *
 * Class internal implementation
 * @{
 */

/**
 * \name Internal defines and variables to manage HID joystick
 */
//@{

//! Size of report for standard HID joystick
#define UDI_HID_JOYSTICK_REPORT_SIZE  12 //JW NEED TO CHANGE

//! To store current rate of HID joystick
static uint8_t udi_hid_joystick_rate;
//! To store current protocol of HID joystick
static uint8_t udi_hid_joystick_protocol;
//! To signal if a valid report is ready to send
static bool udi_hid_joystick_b_report_valid;
//! Report ready to send
static uint8_t udi_hid_joystick_report[UDI_HID_JOYSTICK_REPORT_SIZE];
//! Signal if a report transfer is on going
static bool udi_hid_joystick_report_trans_ongoing;
//! Buffer used to send report
COMPILER_WORD_ALIGNED
		static uint8_t
		udi_hid_joystick_report_trans[UDI_HID_JOYSTICK_REPORT_SIZE];


/**
 * \brief Callback for set report setup request
 *
 * \return \c 1 always, because it is not used on joystick interface
 */
static bool udi_hid_joystick_setreport(void);

//@}

//! HID report descriptor for standard HID joystick
UDC_DESC_STORAGE udi_hid_joystick_report_desc_t udi_hid_joystick_report_desc = {
{
	0x05,0x01,        //USAGE_PAGE (Generic Desktop)
   0x15,0x00,        //LOGICAL_MINIMUM (0)         // <--------- redundant, can be deleted
   0x09,0x05,        //USAGE (Game Pad)
   0xA1,0x01,        //COLLECTION (Application)
   0x05,0x02,            //USAGE_PAGE (Simulation Controls)
   0x09,0xBB,            //USAGE (Throttle)
   0x15,0x81,            //LOGICAL_MINIMUM (-127)
   0x25,0x7F,            //LOGICAL_MAXIMUM (127)
   0x75,0x08,            //REPORT_SIZE (8)
   0x95,0x01,            //REPORT_COUNT (1)
   0x81,0x02,            //INPUT(Data, Var, Abs)
   0x05,0x01,            //USAGE_PAGE (Generic Desktop)
   0x09,0x01,            //USAGE (Pointer)
   0xA1,0x00,            //COLLECTION (Physical)
   0x09,0x30,                //USAGE (X)
   0x09,0x31,                //USAGE (Y)
   0x09,0x32,                //USAGE (Z)
   0x09,0x33,                //USAGE (Rx)
   0x09,0x34,                //USAGE (Ry)
   0x09,0x35,                //USAGE (Rz)
   0x09,0x36,                //USAGE (Slider)
   0x95,0x07,                //REPORT_COUNT (8)
   0x81,0x03,                //INPUT (Data, Var, Abs)
   0xC0,                 //END_COLLECTION
   0x19,0x01,            //USAGE_MINIMUM (Button 1)
   0x29,0x20,            //USAGE_MAXIMUM (Button 32)
   0x15,0x00,            //LOGICAL_MINIMUM (0)
   0x25,0x01,            //LOGICAL_MAXIMUM (1)
   0x75,0x01,            //REPORT_SIZE (1)
   0x95,0x20,            //REPORT_COUNT (32)
   0x55,0x00,            //UNIT_EXPONENT (0)
   0x65,0x00,            //UNIT (None)
   0x81,0x02,            //INPUT (Data, Var, Abs)   
   0x05,0x09,            //USAGE_PAGE (Button)                                                   
   0x75,0x00,            //REPORT_SIZE (0)				//  bit padding
   0x95,0x01,            //REPORT_COUNT (1)            // <--------- add this line
   0x81,0x03,            //INPUT (Const, Var, Abs)
   0xC0              //END_COLLECTION
 
}	

};

/**
 * \name Internal routines
 */
//@{


/**
 * \brief Changes a button state
 *
 * \param b_state    New button state
 * \param btn        Index of button to change (4=middle, 2=right, 1=left)
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
//static bool udi_hid_joystick_btn(bool b_state, uint8_t btn);

/**
 * \brief Send the report
 *
 * \return \c 1 if send on going, \c 0 if delay.
 */
static bool udi_hid_joystick_send_report(void);

static bool udi_hid_joystick_btn(bool b_state, uint8_t btn);

/**
 * \brief Callback called when the report is sent
 *
 * \param status     UDD_EP_TRANSFER_OK, if transfer finish
 * \param status     UDD_EP_TRANSFER_ABORT, if transfer aborted
 * \param nb_sent    number of data transfered
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
static void udi_hid_joystick_report_sent(udd_ep_status_t status,
		iram_size_t nb_sent, udd_ep_id_t ep);

/**
 * \brief Callback called to update report from USB host
 * udi_hid_joystick_report_set is updated before callback execution
 */
static void udi_hid_joystick_setreport_valid(void);

//@}


//--------------------------------------------
//------ Interface for UDI HID level

bool udi_hid_joystick_enable(void)
{
	//JW NEED TO SET FOR JOYSTICK
	// Initialize internal value
	udi_hid_joystick_rate = 0;
	udi_hid_joystick_protocol = 0;
	udi_hid_joystick_report_trans_ongoing = false;
	memset(udi_hid_joystick_report, 0, UDI_HID_JOYSTICK_REPORT_SIZE);
	udi_hid_joystick_b_report_valid = false;
	return UDI_HID_JOYSTICK_ENABLE_EXT();
}


void udi_hid_joystick_disable(void)
{
	UDI_HID_JOYSTICK_DISABLE_EXT();
}


bool udi_hid_joystick_setup(void)
{
	//JW NEED TO SET FOR JOYSTICK
	return udi_hid_setup(&udi_hid_joystick_rate,
								&udi_hid_joystick_protocol,
								(uint8_t *) &udi_hid_joystick_report_desc,
								udi_hid_joystick_setreport);
}


uint8_t udi_hid_joystick_getsetting(void)
{
	//JW NEED TO SET FOR JOYSTICK
	return 0;
}


static bool udi_hid_joystick_setreport(void)
{
	//JW NEED TO SET FOR JOYSTICK
	return false;
}


//--------------------------------------------
//------ Interface for application


//JW NEED TO SET FOR JOYSTICK
/*
bool udi_hid_mouse_btnright(bool b_state)
{
	return udi_hid_mouse_btn(b_state, 0x02);
}*/


//JW NEED TO SET FOR JOYSTICK

//--------------------------------------------
//------ Internal routines
/*
bool udi_hid_joystick_btn1(bool b_state)
{
	return udi_hid_joystick_btn(b_state, 0x01);
}

bool udi_hid_joystick_btn2(bool b_state)
{
	return udi_hid_joystick_btn(b_state, 0x02);
}

bool udi_hid_joystick_btn3(bool b_state)
{
	return udi_hid_joystick_btn(b_state, 0x04);
}

bool udi_hid_joystick_btn4(bool b_state)
{
	return udi_hid_joystick_btn(b_state, 0x08);
}

bool udi_hid_joystick_X(uint8_t value)
{
	irqflags_t flags = cpu_irq_save();
	
	udi_hid_joystick_report[1] = value;
	
	udi_hid_joystick_b_report_valid = true;
	udi_hid_joystick_send_report();	
	
	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_joystick_Y(uint8_t value)
{
	irqflags_t flags = cpu_irq_save();
	
	udi_hid_joystick_report[2] = value;
	
	udi_hid_joystick_b_report_valid = true;
	udi_hid_joystick_send_report();
	
	cpu_irq_restore(flags);
	return true;
}*/

bool send_joystick_report(uint8_t *joybuf){
	irqflags_t flags = cpu_irq_save();
	
	memcpy(udi_hid_joystick_report, joybuf,	UDI_HID_JOYSTICK_REPORT_SIZE);
	
	udi_hid_joystick_b_report_valid = true;
	udi_hid_joystick_send_report();	
	
	cpu_irq_restore(flags);
	return true;
	
}
/*
static bool udi_hid_joystick_btn(bool b_state, uint8_t btn)
{
	
	irqflags_t flags = cpu_irq_save();
	
	// Modify buttons report
	if (HID_JOYSTICK_BTN_DOWN == b_state)
		udi_hid_joystick_report[3] |= btn;
	else
		udi_hid_joystick_report[3] &= ~(unsigned)btn;
		
	//udi_hid_joystick_report[0] = 0x78; 
	//udi_hid_joystick_report[1] = 0x78;
	//udi_hid_joystick_report[2] = 0x78;
	//udi_hid_joystick_report[3] = 0x05; //Buttons
	
	udi_hid_joystick_b_report_valid = true;
	udi_hid_joystick_send_report();	
	
	cpu_irq_restore(flags);
	return true;
}*/


static bool udi_hid_joystick_send_report(void)
{
	if (udi_hid_joystick_report_trans_ongoing)
		return false;	// Transfer on going then send this one after transfer complete

	// Copy report on other array used only for transfer
	memcpy(udi_hid_joystick_report_trans, udi_hid_joystick_report,
			UDI_HID_JOYSTICK_REPORT_SIZE);
	//memset(&udi_hid_joystick_report[1], 0, 3);	// Keep status of btn for next report
	udi_hid_joystick_b_report_valid = false;

	// Send report
	udi_hid_joystick_report_trans_ongoing =
			udd_ep_run(	UDI_HID_JOYSTICK_EP_IN,
							false,
							udi_hid_joystick_report_trans,
							UDI_HID_JOYSTICK_REPORT_SIZE,
							udi_hid_joystick_report_sent);
	return udi_hid_joystick_report_trans_ongoing;
}


static void udi_hid_joystick_report_sent(udd_ep_status_t status,
		iram_size_t nb_sent, udd_ep_id_t ep)
{
	UNUSED(ep);
	UNUSED(status);
	UNUSED(nb_sent);
	// Valid report sending
	udi_hid_joystick_report_trans_ongoing = false;
	if (udi_hid_joystick_b_report_valid) {
		// Send new valid report
		udi_hid_joystick_send_report();
	}
}

//@}
