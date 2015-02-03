//MMA8453_Driver.c

#include <twi_master.h>
#include "HowlerConfig.h"
#include "I2C.h"
#include "MMA8453_Driver.h"

void MMA8453_Init(void){
	uint8_t regRead = 0;

	//put device in active mode
	
	//active mode with fast 8 bit, +/-2 G, 400Hz sample rate
	writeI2C(MMA8453_ADDR, MMA8453_REG_CTRL_REG1, 0x00);
	writeI2C(MMA8453_ADDR, MMA8453_REG_XYZ_DATA_CFG, 0x00);
	writeI2C(MMA8453_ADDR, MMA8453_REG_CTRL_REG1, 0x0B);
	//writeI2C(MMA8453_ADDR, MMA8453_REG_CTRL_REG1, 0x09);

}

uint8_t MMA8453_Read_Output(uint8_t axis){
	uint8_t reading = 0xFF;	 
		 
	if(axis == 1){
		//X Axis
		readI2C(MMA8453_ADDR, MMA8453_REG_OUT_X_MSB, &reading); 
	}
	else if(axis == 2){
		//Y axis
		readI2C(MMA8453_ADDR, MMA8453_REG_OUT_Y_MSB, &reading);
	}
	else if(axis == 3){
		//Z Axis
		readI2C(MMA8453_ADDR, MMA8453_REG_OUT_Z_MSB, &reading);
	}
	else{
		//invalid axis
		//reading = 0xFF;
	}
	return reading;
}

uint8_t MMA8453_Read_XYZ(uint8_t readings[]){
	uint8_t packet[6] = {0,0,0,0,0,0};
	uint8_t statusReg = 0;
	
	readI2C(MMA8453_ADDR, MMA8453_REG_STATUS, &statusReg);
	
	//Check if data is ready for reading
	if((statusReg & 0x04) == 0x04){
		//Read Data into buffer, need 3 byte buffer
		readI2CPacket(MMA8453_ADDR, MMA8453_REG_OUT_X_MSB, readings, 3);
		//readI2CPacket(MMA8453_ADDR, MMA8453_REG_OUT_X_MSB, packet, 6);
		
		//Convert 10 bit to 8 bit without truncation
		/*if((packet[0] & 0x02) == 0x02){
			//number is negative
			readings[0] = packet[1] | 0x80;
		}
		else{
			//number is positive
			readings[0] = packet[1] & 0x7F;
		}
		
		if((packet[2] & 0x02) == 0x02){
			//number is negative
			readings[1] = packet[3] | 0x80;
		}
		else{
			//number is positive
			readings[1] = packet[3] & 0x7F;
		}
		
		if((packet[4] & 0x02) == 0x02){
			//number is negative
			//readings[2] = packet[5] | 0x80;
			readings[2] = packet[5];
		}
		else{
			//number is positive
			readings[2] = packet[5] & 0x7F;
		}*/
		

	}
}