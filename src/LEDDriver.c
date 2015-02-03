//LEDDriver.c

#include <twi_master.h>
#include "HowlerConfig.h"
#include "I2C.h"
#include "LEDDriver.h"





void LED_Drivers_Init(void){
	//Sets up the PCA9635 LED Driver chips
		
	//Set LED Drivers to Normal Mode using MODE1 register
	writeI2C(LED_SLAVE1_ADDR, LED_REG_MODE1, 0x01);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_MODE1, 0x01);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_MODE1, 0x01);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_MODE1, 0x01);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_MODE1, 0x01);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_MODE1, 0x01);
	
	//Set LED Drivers to totem-pole and LEDs off when nOE = 1 using MODE2 register
	writeI2C(LED_SLAVE1_ADDR, LED_REG_MODE2, 0x01);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_MODE2, 0x01);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_MODE2, 0x01);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_MODE2, 0x01);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_MODE2, 0x01);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_MODE2, 0x01);

	
	//Set LED Driver Output states to individual and group control
	writeI2C(LED_SLAVE1_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE1_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE1_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE1_ADDR, LED_REG_LEDOUT3, 0xFF);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_LEDOUT3, 0xFF);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_LEDOUT3, 0xFF);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_LEDOUT3, 0xFF);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_LEDOUT3, 0xFF);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_LEDOUT0, 0xFF);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_LEDOUT1, 0xFF);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_LEDOUT2, 0xFF);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_LEDOUT3, 0xFF);
	
	//Set Group PWM brightness to maximum
	writeI2C(LED_SLAVE1_ADDR, LED_REG_GRPPWM, 0xff);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_GRPPWM, 0xff);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_GRPPWM, 0xff);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_GRPPWM, 0xff);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_GRPPWM, 0xff);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_GRPPWM, 0xff);

	//gpio_set_pin_low(TWI_NOE);
	
}

void setLED(uint16_t ledToSet, uint8_t pwmLevel){
	//Set Individual LED
	writeI2C(ledToSet>>8, ledToSet, pwmLevel);
}

void setLEDBank(uint16_t bankToSet, uint8_t bankValues[]){
	uint8_t deviceAddr = 0xff;
	
	switch(bankToSet){
		case 1:
			deviceAddr = LED_SLAVE1_ADDR;
			break;
		case 2:
			deviceAddr = LED_SLAVE2_ADDR;
			break;
		case 3:
			deviceAddr = LED_SLAVE3_ADDR;
			break;
		case 4:
			deviceAddr = LED_SLAVE4_ADDR;
			break;
		case 5:
			deviceAddr = LED_SLAVE5_ADDR;
			break;
		case 6:
			deviceAddr = LED_SLAVE6_ADDR;
			break;
		default:
			deviceAddr = 0xff;
			break;
	}
	
	//Set LED Bank
	writeI2CPacket(deviceAddr, LED_REG_PWM0|LED_AUTO_INC_IND_BRT, bankValues, 16);
}

void setLEDRGB(uint64_t ledToSet, uint32_t rgbLevel){

	//set red LED
	writeI2C(ledToSet>>40, ledToSet>>32, rgbLevel>>16);
	
	//set green LED
	writeI2C(ledToSet>>24, ledToSet>>16, rgbLevel>>8);
	
	//set blue LED
	writeI2C(ledToSet>>8, ledToSet, rgbLevel);
}


void getLED(uint16_t ledToGet, uint8_t* pwmLevel){
	uint8_t level = 0;
	
	readI2C(ledToGet>>8, ledToGet, &level);
	
	*pwmLevel = level;

}

void setLEDglobalbrightness(uint8_t globalBrightness){
	writeI2C(LED_SLAVE1_ADDR, LED_REG_GRPPWM, globalBrightness);
	writeI2C(LED_SLAVE2_ADDR, LED_REG_GRPPWM, globalBrightness);
	writeI2C(LED_SLAVE3_ADDR, LED_REG_GRPPWM, globalBrightness);
	writeI2C(LED_SLAVE4_ADDR, LED_REG_GRPPWM, globalBrightness);
	writeI2C(LED_SLAVE5_ADDR, LED_REG_GRPPWM, globalBrightness);
	writeI2C(LED_SLAVE6_ADDR, LED_REG_GRPPWM, globalBrightness);
}

