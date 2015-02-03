// LEDDriver.h


//TWI Definitions
#define TWI_MASTER        &TWIC
#define TWI_MASTER_PORT   PORTC
#define TWI_SPEED         1000000 //EXTREME SPEED I2C!!!!!!! 1MHz
#define CPU_SPEED		  32000000
#define TWI_MASTER_ADDR   0x50
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, TWI_SPEED)

//function definitions
void I2C_Init(void);
status_code_t readI2C(uint8_t addr, uint8_t registerNumber, uint8_t *value);
status_code_t readI2CPacket(uint8_t addr, uint8_t registerNumber, uint8_t packet[], unsigned int packetLength);
status_code_t writeI2C(uint8_t addr, uint8_t registerNumber, uint8_t value);
status_code_t writeI2CPacket(uint8_t addr, uint8_t registerNumber, uint8_t packet[], unsigned int packetLength);