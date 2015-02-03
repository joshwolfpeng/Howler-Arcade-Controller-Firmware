//I2C.c
//XMEGA I2C Driver

#include <twi_master.h>
#include "HowlerConfig.h"
#include "I2C.h"

void I2C_Init(void){
	
	//enable the TWI_MASTER clock
	
	twi_options_t t_options = {
		.speed     = TWI_SPEED,
		.chip      = TWI_MASTER_ADDR,
		.speed_reg = TWI_BAUD(CPU_SPEED, TWI_SPEED)
	};
	
	gpio_set_pin_low(TWI_NOE);
	
	sysclk_enable_peripheral_clock(TWI_MASTER);

	//initialize the twi interface
	twi_master_init(TWI_MASTER, &t_options);
	twi_master_enable(TWI_MASTER);
}

status_code_t readI2C(uint8_t addr, uint8_t registerNumber, uint8_t *value){
	int status = 0;
	
	uint8_t twi_packet[] = {0};
	
	twi_package_t twi_slave_packet = {
		.addr[0]      = registerNumber,      // TWI slave memory address data MSB
		.addr_length  = 1,     // TWI slave memory address data size
		.chip         = addr,    // TWI slave bus address
		.buffer       = &twi_packet, // transfer data source buffer
		.length       = 1,   // transfer data size (bytes)
		.no_wait	= true//false//true
	};
	status = twi_master_read(TWI_MASTER, &twi_slave_packet);

	if(status == TWI_SUCCESS){
		*value = twi_packet[0];
		return(STATUS_OK);
	}
	
	else{
		return(ERR_IO_ERROR);
	}
}

status_code_t readI2CPacket(uint8_t addr, uint8_t registerNumber, uint8_t packet[], unsigned int packetLength){
	int status = 0;
	
	//uint8_t twi_packet[] = {0};
	
	twi_package_t twi_slave_packet = {
		.addr[0]      = registerNumber,      // TWI slave memory address data MSB
		.addr_length  = 1,     // TWI slave memory address data size
		.chip         = addr,    // TWI slave bus address
		.buffer       = packet, // transfer data source buffer
		.length       = packetLength,   // transfer data size (bytes)
		.no_wait	= false//true
	};
	status = twi_master_read(TWI_MASTER, &twi_slave_packet);

	if(status == TWI_SUCCESS){
		//*value = twi_packet[0];
		return(STATUS_OK);
	}
	
	else{
		return(ERR_IO_ERROR);
	}
}

status_code_t writeI2C(uint8_t addr, uint8_t registerNumber, uint8_t value){
	uint8_t twi_packet[] = {0};

	twi_packet[0] = value;

	twi_package_t twi_slave_packet = {
		.addr[0]      = registerNumber,      // TWI slave memory address data MSB
		.addr_length  = 1,     // TWI slave memory address data size
		.chip         = addr,      // TWI slave bus address
		.buffer       = &twi_packet, // transfer data source buffer
		.length       = 1,   // transfer data size (bytes)
		.no_wait	= true//false //true
	};
	while (twi_master_write(TWI_MASTER, &twi_slave_packet) != TWI_SUCCESS);

}

status_code_t writeI2CPacket(uint8_t addr, uint8_t registerNumber, uint8_t packet[], unsigned int packetLength){
	int status = 0;
	
	//uint8_t twi_packet[] = {0};
	
	twi_package_t twi_slave_packet = {
		.addr[0]      = registerNumber,      // TWI slave memory address data MSB
		.addr_length  = 1,     // TWI slave memory address data size
		.chip         = addr,    // TWI slave bus address
		.buffer       = packet, // transfer data source buffer
		.length       = packetLength,   // transfer data size (bytes)
		.no_wait	= true//true
	};
	while (twi_master_write(TWI_MASTER, &twi_slave_packet) != TWI_SUCCESS);

	if(status == TWI_SUCCESS){
		//*value = twi_packet[0];
		return(STATUS_OK);
	}
	
	else{
		return(ERR_IO_ERROR);
	}
}