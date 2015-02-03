//HowlerInit.c 
//

#include <ioport.h>
#include <avr/eeprom.h>
#include "HowlerConfig.h"
#include "HowlerInit.h"

void Howler_Init(void)
{
	//Default Configuration (need to update for changing board configurations)
	
	//Uncomment to Disable JTAG to allow use of pins 
	CCP = CCP_IOREG_gc;
	MCU.MCUCR = 0x01;

	//Set LEDs as outputs
	ioport_configure_pin(LED_USBINIT, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(LED_COP, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(TWI_NOE, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	//Set joystick pins as inputs
	ioport_configure_pin(JOY1_U, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY1_D, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY1_L, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY1_R, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY2_U, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY2_D, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY2_L, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY2_R, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);	
	
	ioport_configure_pin(JOY3_U, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY3_D, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY3_L, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY3_R, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	
	ioport_configure_pin(JOY4_U, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY4_D, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY4_L, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(JOY4_R, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	
	//Set button pins as inputs
	ioport_configure_pin(BUTT1, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT2, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT3, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT4, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT5, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT6, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT7, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT8, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT9, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT10, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT11, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT12, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT13, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT14, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT15, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT16, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT17, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT18, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT19, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT20, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT21, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT22, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT23, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT24, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT25, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	ioport_configure_pin(BUTT26, IOPORT_DIR_INPUT | IOPORT_LEVEL | IOPORT_PULL_UP);
	
}
