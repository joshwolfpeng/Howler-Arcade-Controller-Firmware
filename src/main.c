/**
 * \file
 *
 * \brief Howler Controller Main File.
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <avr/eeprom.h>
#include <string.h>
#include "HowlerConfig.h"
#include "HowlerInit.h"
#include "LEDDriver.h"
#include "MMA8453_Driver.h"
#include "I2C.h"
#include "main.h"

static uint8_t howler_hid_report[UDI_HID_REPORT_IN_SIZE];
static uint64_t gKeyPressed = 0;

uint8_t  gJoy1Buf[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t  gJoy2Buf[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint16_t ledr_look_up[NUM_RGB_LEDS] = {JOY1_LEDR, JOY2_LEDR, JOY3_LEDR, JOY4_LEDR, BUTT1_LEDR, BUTT2_LEDR, BUTT3_LEDR, BUTT4_LEDR, BUTT5_LEDR, BUTT6_LEDR, BUTT7_LEDR, BUTT8_LEDR, BUTT9_LEDR, BUTT10_LEDR, BUTT11_LEDR, BUTT12_LEDR, BUTT13_LEDR, BUTT14_LEDR, BUTT15_LEDR, BUTT16_LEDR, BUTT17_LEDR, BUTT18_LEDR, BUTT19_LEDR, BUTT20_LEDR, BUTT21_LEDR, BUTT22_LEDR, BUTT23_LEDR, BUTT24_LEDR, BUTT25_LEDR, BUTT26_LEDR, HP1_LEDR, HP2_LEDR};
uint16_t ledg_look_up[NUM_RGB_LEDS] = {JOY1_LEDG, JOY2_LEDG, JOY3_LEDG, JOY4_LEDG, BUTT1_LEDG, BUTT2_LEDG, BUTT3_LEDG, BUTT4_LEDG, BUTT5_LEDG, BUTT6_LEDG, BUTT7_LEDG, BUTT8_LEDG, BUTT9_LEDG, BUTT10_LEDG, BUTT11_LEDG, BUTT12_LEDG, BUTT13_LEDG, BUTT14_LEDG, BUTT15_LEDG, BUTT16_LEDG, BUTT17_LEDG, BUTT18_LEDG, BUTT19_LEDG, BUTT20_LEDG, BUTT21_LEDG, BUTT22_LEDG, BUTT23_LEDG, BUTT24_LEDG, BUTT25_LEDG, BUTT26_LEDG, HP1_LEDG, HP2_LEDG};
uint16_t ledb_look_up[NUM_RGB_LEDS] = {JOY1_LEDB, JOY2_LEDB, JOY3_LEDB, JOY4_LEDB, BUTT1_LEDB, BUTT2_LEDB, BUTT3_LEDB, BUTT4_LEDB, BUTT5_LEDB, BUTT6_LEDB, BUTT7_LEDB, BUTT8_LEDB, BUTT9_LEDB, BUTT10_LEDB, BUTT11_LEDB, BUTT12_LEDB, BUTT13_LEDB, BUTT14_LEDB, BUTT15_LEDB, BUTT16_LEDB, BUTT17_LEDB, BUTT18_LEDB, BUTT19_LEDB, BUTT20_LEDB, BUTT21_LEDB, BUTT22_LEDB, BUTT23_LEDB, BUTT24_LEDB, BUTT25_LEDB, BUTT26_LEDB, HP1_LEDB, HP2_LEDB};
uint16_t led_look_up[NUM_LEDS] = {JOY1_LEDR, JOY1_LEDG,JOY1_LEDB,JOY2_LEDR,JOY2_LEDG,JOY2_LEDB,JOY3_LEDR,JOY3_LEDG,JOY3_LEDB,JOY4_LEDR,JOY4_LEDG,JOY4_LEDB,BUTT1_LEDR,BUTT1_LEDG,BUTT1_LEDB,BUTT2_LEDR,BUTT2_LEDG,BUTT2_LEDB,BUTT3_LEDR,BUTT3_LEDG,BUTT3_LEDB,BUTT4_LEDR,BUTT4_LEDG,BUTT4_LEDB,BUTT5_LEDR,BUTT5_LEDG,BUTT5_LEDB, BUTT6_LEDR,BUTT6_LEDG,BUTT6_LEDB,BUTT7_LEDR,BUTT7_LEDG,BUTT7_LEDB,BUTT8_LEDR,BUTT8_LEDG,BUTT8_LEDB,BUTT9_LEDR,BUTT9_LEDG,BUTT9_LEDB,BUTT10_LEDR,BUTT10_LEDG,BUTT10_LEDB,BUTT11_LEDR,BUTT11_LEDG,BUTT11_LEDB,BUTT12_LEDR,BUTT12_LEDG,BUTT12_LEDB,BUTT13_LEDR,BUTT13_LEDG,BUTT13_LEDB,BUTT14_LEDR,BUTT14_LEDG,BUTT14_LEDB,BUTT15_LEDR,BUTT15_LEDG,BUTT15_LEDB,BUTT16_LEDR,BUTT16_LEDG,BUTT16_LEDB,BUTT17_LEDR,BUTT17_LEDG,BUTT17_LEDB,BUTT18_LEDR,BUTT18_LEDG,BUTT18_LEDB,BUTT19_LEDR,BUTT19_LEDG,BUTT19_LEDB,BUTT20_LEDR,BUTT20_LEDG,BUTT20_LEDB,BUTT21_LEDR,BUTT21_LEDG,BUTT21_LEDB,BUTT22_LEDR,BUTT22_LEDG,BUTT22_LEDB,BUTT23_LEDR,BUTT23_LEDG,BUTT23_LEDB,BUTT24_LEDR,BUTT24_LEDG,BUTT24_LEDB,BUTT25_LEDR,BUTT25_LEDG,BUTT25_LEDB,BUTT26_LEDR,BUTT26_LEDG,BUTT26_LEDB, HP1_LEDR, HP1_LEDG, HP1_LEDB, HP2_LEDR, HP2_LEDG, HP2_LEDB};


//EEPROM MEMORY LOCATION lOOKUP
uint16_t eeprom_input_value_look_up[NUM_INPUTS] = {JOY1_U_VALUE_ADDR, JOY1_D_VALUE_ADDR, JOY1_L_VALUE_ADDR, JOY1_R_VALUE_ADDR, JOY2_U_VALUE_ADDR, JOY2_D_VALUE_ADDR, JOY2_L_VALUE_ADDR, JOY2_R_VALUE_ADDR, JOY3_U_VALUE_ADDR, JOY3_D_VALUE_ADDR, JOY3_L_VALUE_ADDR, JOY3_R_VALUE_ADDR, JOY4_U_VALUE_ADDR, JOY4_D_VALUE_ADDR, JOY4_L_VALUE_ADDR, JOY4_R_VALUE_ADDR, BUTT1_VALUE_ADDR, BUTT2_VALUE_ADDR, BUTT3_VALUE_ADDR, BUTT4_VALUE_ADDR, BUTT5_VALUE_ADDR, BUTT6_VALUE_ADDR, BUTT7_VALUE_ADDR, BUTT8_VALUE_ADDR, BUTT9_VALUE_ADDR, BUTT10_VALUE_ADDR, BUTT11_VALUE_ADDR, BUTT12_VALUE_ADDR, BUTT13_VALUE_ADDR, BUTT14_VALUE_ADDR, BUTT15_VALUE_ADDR, BUTT16_VALUE_ADDR, BUTT17_VALUE_ADDR, BUTT18_VALUE_ADDR, BUTT19_VALUE_ADDR, BUTT20_VALUE_ADDR, BUTT21_VALUE_ADDR, BUTT22_VALUE_ADDR, BUTT23_VALUE_ADDR, BUTT24_VALUE_ADDR, BUTT25_VALUE_ADDR, BUTT26_VALUE_ADDR, ACCEL_XAXIS_VALUE_ADDR, ACCEL_YAXIS_VALUE_ADDR, ACCEL_ZAXIS_VALUE_ADDR};
uint16_t eeprom_input_value2_look_up[NUM_INPUTS] = {JOY1_U_VALUE2_ADDR, JOY1_D_VALUE2_ADDR, JOY1_L_VALUE2_ADDR, JOY1_R_VALUE2_ADDR, JOY2_U_VALUE2_ADDR, JOY2_D_VALUE2_ADDR, JOY2_L_VALUE2_ADDR, JOY2_R_VALUE2_ADDR, JOY3_U_VALUE2_ADDR, JOY3_D_VALUE2_ADDR, JOY3_L_VALUE2_ADDR, JOY3_R_VALUE2_ADDR, JOY4_U_VALUE2_ADDR, JOY4_D_VALUE2_ADDR, JOY4_L_VALUE2_ADDR, JOY4_R_VALUE2_ADDR, BUTT1_VALUE2_ADDR, BUTT2_VALUE2_ADDR, BUTT3_VALUE2_ADDR, BUTT4_VALUE2_ADDR, BUTT5_VALUE2_ADDR, BUTT6_VALUE2_ADDR, BUTT7_VALUE2_ADDR, BUTT8_VALUE2_ADDR, BUTT9_VALUE2_ADDR, BUTT10_VALUE2_ADDR, BUTT11_VALUE2_ADDR, BUTT12_VALUE2_ADDR, BUTT13_VALUE2_ADDR, BUTT14_VALUE2_ADDR, BUTT15_VALUE2_ADDR, BUTT16_VALUE2_ADDR, BUTT17_VALUE2_ADDR, BUTT18_VALUE2_ADDR, BUTT19_VALUE2_ADDR, BUTT20_VALUE2_ADDR, BUTT21_VALUE2_ADDR, BUTT22_VALUE2_ADDR, BUTT23_VALUE2_ADDR, BUTT24_VALUE2_ADDR, BUTT25_VALUE2_ADDR, BUTT26_VALUE2_ADDR, ACCEL_XAXIS_VALUE2_ADDR, ACCEL_YAXIS_VALUE2_ADDR, ACCEL_ZAXIS_VALUE2_ADDR};
uint16_t eeprom_input_type_look_up[NUM_INPUTS] = {JOY1_U_TYPE_ADDR, JOY1_D_TYPE_ADDR, JOY1_L_TYPE_ADDR, JOY1_R_TYPE_ADDR, JOY2_U_TYPE_ADDR, JOY2_D_TYPE_ADDR, JOY2_L_TYPE_ADDR, JOY2_R_TYPE_ADDR, JOY3_U_TYPE_ADDR, JOY3_D_TYPE_ADDR, JOY3_L_TYPE_ADDR, JOY3_R_TYPE_ADDR, JOY4_U_TYPE_ADDR, JOY4_D_TYPE_ADDR, JOY4_L_TYPE_ADDR, JOY4_R_TYPE_ADDR, BUTT1_TYPE_ADDR, BUTT2_TYPE_ADDR, BUTT3_TYPE_ADDR, BUTT4_TYPE_ADDR, BUTT5_TYPE_ADDR, BUTT6_TYPE_ADDR, BUTT7_TYPE_ADDR, BUTT8_TYPE_ADDR, BUTT9_TYPE_ADDR, BUTT10_TYPE_ADDR, BUTT11_TYPE_ADDR, BUTT12_TYPE_ADDR, BUTT13_TYPE_ADDR, BUTT14_TYPE_ADDR, BUTT15_TYPE_ADDR, BUTT16_TYPE_ADDR, BUTT17_TYPE_ADDR, BUTT18_TYPE_ADDR, BUTT19_TYPE_ADDR, BUTT20_TYPE_ADDR, BUTT21_TYPE_ADDR, BUTT22_TYPE_ADDR, BUTT23_TYPE_ADDR, BUTT24_TYPE_ADDR, BUTT25_TYPE_ADDR, BUTT26_TYPE_ADDR, ACCEL_XAXIS_TYPE_ADDR, ACCEL_YAXIS_TYPE_ADDR, ACCEL_ZAXIS_TYPE_ADDR};
uint16_t eeprom_ledr_look_up[NUM_RGB_LEDS] = {JOY1_LEDR_ADDR, JOY2_LEDR_ADDR, JOY3_LEDR_ADDR, JOY4_LEDR_ADDR, BUTT1_LEDR_ADDR, BUTT2_LEDR_ADDR, BUTT3_LEDR_ADDR, BUTT4_LEDR_ADDR, BUTT5_LEDR_ADDR, BUTT6_LEDR_ADDR, BUTT7_LEDR_ADDR, BUTT8_LEDR_ADDR, BUTT9_LEDR_ADDR, BUTT10_LEDR_ADDR, BUTT11_LEDR_ADDR, BUTT12_LEDR_ADDR, BUTT13_LEDR_ADDR, BUTT14_LEDR_ADDR, BUTT15_LEDR_ADDR, BUTT16_LEDR_ADDR, BUTT17_LEDR_ADDR, BUTT18_LEDR_ADDR, BUTT19_LEDR_ADDR, BUTT20_LEDR_ADDR, BUTT21_LEDR_ADDR, BUTT22_LEDR_ADDR, BUTT23_LEDR_ADDR, BUTT24_LEDR_ADDR, BUTT25_LEDR_ADDR, BUTT26_LEDR_ADDR, HP1_LEDR_ADDR, HP2_LEDR_ADDR};
uint16_t eeprom_ledg_look_up[NUM_RGB_LEDS] = {JOY1_LEDG_ADDR, JOY2_LEDG_ADDR, JOY3_LEDG_ADDR, JOY4_LEDG_ADDR, BUTT1_LEDG_ADDR, BUTT2_LEDG_ADDR, BUTT3_LEDG_ADDR, BUTT4_LEDG_ADDR, BUTT5_LEDG_ADDR, BUTT6_LEDG_ADDR, BUTT7_LEDG_ADDR, BUTT8_LEDG_ADDR, BUTT9_LEDG_ADDR, BUTT10_LEDG_ADDR, BUTT11_LEDG_ADDR, BUTT12_LEDG_ADDR, BUTT13_LEDG_ADDR, BUTT14_LEDG_ADDR, BUTT15_LEDG_ADDR, BUTT16_LEDG_ADDR, BUTT17_LEDG_ADDR, BUTT18_LEDG_ADDR, BUTT19_LEDG_ADDR, BUTT20_LEDG_ADDR, BUTT21_LEDG_ADDR, BUTT22_LEDG_ADDR, BUTT23_LEDG_ADDR, BUTT24_LEDG_ADDR, BUTT25_LEDG_ADDR, BUTT26_LEDG_ADDR, HP1_LEDG_ADDR, HP2_LEDG_ADDR};
uint16_t eeprom_ledb_look_up[NUM_RGB_LEDS] = {JOY1_LEDB_ADDR, JOY2_LEDB_ADDR, JOY3_LEDB_ADDR, JOY4_LEDB_ADDR, BUTT1_LEDB_ADDR, BUTT2_LEDB_ADDR, BUTT3_LEDB_ADDR, BUTT4_LEDB_ADDR, BUTT5_LEDB_ADDR, BUTT6_LEDB_ADDR, BUTT7_LEDB_ADDR, BUTT8_LEDB_ADDR, BUTT9_LEDB_ADDR, BUTT10_LEDB_ADDR, BUTT11_LEDB_ADDR, BUTT12_LEDB_ADDR, BUTT13_LEDB_ADDR, BUTT14_LEDB_ADDR, BUTT15_LEDB_ADDR, BUTT16_LEDB_ADDR, BUTT17_LEDB_ADDR, BUTT18_LEDB_ADDR, BUTT19_LEDB_ADDR, BUTT20_LEDB_ADDR, BUTT21_LEDB_ADDR, BUTT22_LEDB_ADDR, BUTT23_LEDB_ADDR, BUTT24_LEDB_ADDR, BUTT25_LEDB_ADDR, BUTT26_LEDB_ADDR, HP1_LEDB_ADDR, HP2_LEDB_ADDR};

uint8_t input_type_lookup[NUM_INPUTS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t input_value_lookup[NUM_INPUTS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t input_value2_lookup[NUM_INPUTS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//QDEC Variables
qdec_config_t gConfig1;
qdec_config_t gConfig2;
qdec_config_t gConfig3;

uint16_t gQdec1_prev_pos = 0;
uint16_t gQdec2_prev_pos = 0;
uint16_t gQdec3_prev_pos = 0;

uint16_t gJoy1_u_ana_val = 0;
uint16_t gJoy1_d_ana_val = 0;
uint16_t gJoy1_l_ana_val = 0;
uint16_t gJoy1_r_ana_val = 0;

uint8_t gReport[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t gHidData = 0;	
int gCopCountdown = 0;
int gTestTimer = 0;

uint8_t gAccelData[3] = {0,0,0};
	
uint8_t gAccelMinTrig[3] = {0,0,0};
uint8_t gAccelMaxTrig[3] = {0,0,0};	

int main (void)
{
	int cnt = 0;
	//Initialize I/O on Howler PCB
	Howler_Init();
	//initialize system clocks
	sysclk_init();
	//Init I2C
	I2C_Init();
	//initialize LED drivers
	LED_Drivers_Init();	
	
	MMA8453_Init();
	
	qdec1_init();
	qdec2_init();
	qdec3_init();
	
	adc_init();
	
	pmic_init();
	
	cpu_irq_enable();
	
	adc_enable(&JOY_ADC_MODULE);
	
	//check to see if eeprom is empty, if so initialize to defaults
	if((eeprom_read_byte(eeprom_input_type_look_up[0]) == 0xFF)&&(eeprom_read_byte(eeprom_input_type_look_up[44]) == 0xFF)&&(eeprom_read_byte(eeprom_input_value_look_up[0]) == 0xFF)&&(eeprom_read_byte(eeprom_input_value_look_up[44]) == 0xFF)){
		setInputDefaults();
		setLedDefaults();
	}	
		
	//get eeprom memory to sram
	for(cnt = 0; cnt < NUM_INPUTS; cnt++){
		input_type_lookup[cnt] = eeprom_read_byte(eeprom_input_type_look_up[cnt]);
		input_value_lookup[cnt] = eeprom_read_byte(eeprom_input_value_look_up[cnt]);
		input_value2_lookup[cnt] = eeprom_read_byte(eeprom_input_value2_look_up[cnt]);
	}
	gAccelMinTrig[0] = eeprom_read_byte(ACCEL_XAXIS_MIN_TRIG_ADDR);
	gAccelMinTrig[1] = eeprom_read_byte(ACCEL_YAXIS_MIN_TRIG_ADDR);
	gAccelMinTrig[2] = eeprom_read_byte(ACCEL_ZAXIS_MIN_TRIG_ADDR);
	gAccelMaxTrig[0] = eeprom_read_byte(ACCEL_XAXIS_MAX_TRIG_ADDR);
	gAccelMaxTrig[1] = eeprom_read_byte(ACCEL_YAXIS_MAX_TRIG_ADDR);
	gAccelMaxTrig[2] = eeprom_read_byte(ACCEL_ZAXIS_MAX_TRIG_ADDR);
	
	
	irq_initialize_vectors();
	cpu_irq_enable();
	sleepmgr_init();
	
	//Set LEDs from eeprom
	setLeds();

	udc_start();
	udc_attach();
			
	while(1){

		parseHidReport();

		if(gTestTimer == 200){
			gTestTimer= 0;
			MMA8453_Read_XYZ(gAccelData);
		}
		gTestTimer++;
		
	}
}


//Start of frame callback (check for button presses and act accordingly!)
void user_callback_sof_action(void){
	uint8_t i = 0;
	bool qdec1_dir = 0;
	uint16_t qdec1_pos = 0;
	uint16_t qdec1_freq = 0;
	bool qdec2_dir = 0;
	uint16_t qdec2_pos = 0;
	uint16_t qdec2_freq = 0;
	bool qdec3_dir = 0;
	uint16_t qdec3_pos = 0;
	uint16_t qdec3_freq = 0;
	uint16_t accelReadingX = 0;

	
	for(i = 0; i < 12; i++){
		//initialize joybufs
		gJoy1Buf[i] = 0x00;
		gJoy2Buf[i] = 0x00;
	}		
	
	if(gpio_pin_is_low(LED_USBINIT)){
		gpio_set_pin_high(LED_USBINIT);
	}
	
	//Set COP LED	
	if(gCopCountdown == COP_TIMEOUT){
		gCopCountdown = 0;
		gpio_toggle_pin(LED_COP);
	}	
	gCopCountdown++;
	
	//Check if Joy1_U pin is an analog input
	if((input_type_lookup[JOY1_U_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//JOY1_U is an analog input on Joystick 1
		gJoy1Buf[input_type_lookup[JOY1_U_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = (int8_t)(gJoy1_u_ana_val-128);
	}
	else if((input_type_lookup[JOY1_U_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//JOY1_U is an analog input on Joystick 2
		gJoy2Buf[input_type_lookup[JOY1_U_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = (int8_t)(gJoy1_u_ana_val-128);
	}
	else{
		//JOY1_U is a digital input
		if(ioport_pin_is_low(JOY1_U)){
			checkJoystickDown(JOY1_U_LKUP_INDEX, JOY1_U_MASK);
		}	
		else{
			checkJoystickUp(JOY1_U_LKUP_INDEX, JOY1_U_MASK);	
		}
	}	
		
	if((input_type_lookup[JOY1_D_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//JOY1_D is an analog input on Joystick 1
		gJoy1Buf[input_type_lookup[JOY1_D_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = (int8_t)(gJoy1_d_ana_val-128);
	}
	else if((input_type_lookup[JOY1_D_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//JOY1_D is an analog input on Joystick 2
		gJoy2Buf[input_type_lookup[JOY1_D_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = (int8_t)(gJoy1_d_ana_val-128);
	}
	else{
		//JOY1_D is a digital input
		if(ioport_pin_is_low(JOY1_D)){
			checkJoystickDown(JOY1_D_LKUP_INDEX, JOY1_D_MASK);
		}
		else{
			checkJoystickUp(JOY1_D_LKUP_INDEX, JOY1_D_MASK);
		}
	}
	
	if((input_type_lookup[JOY1_L_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//JOY1_L is an analog input on joystick 1
		gJoy1Buf[input_type_lookup[JOY1_L_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = (int8_t)(gJoy1_l_ana_val-128);
	}
	else if((input_type_lookup[JOY1_L_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//JOY1_L is an analog input on joystick 2
		gJoy2Buf[input_type_lookup[JOY1_L_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = (int8_t)(gJoy1_l_ana_val-128);
	}
	else{
		//JOY1_L is a digital input	
		if(ioport_pin_is_low(JOY1_L)){
			checkJoystickDown(JOY1_L_LKUP_INDEX, JOY1_L_MASK);
		}
		else{
			checkJoystickUp(JOY1_L_LKUP_INDEX, JOY1_L_MASK);
		}
	}	
	
	if((input_type_lookup[JOY1_R_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//JOY1_R is an analog input on joystick 1
		gJoy1Buf[input_type_lookup[JOY1_R_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = (int8_t)(gJoy1_r_ana_val-128);
	}
	else if((input_type_lookup[JOY1_R_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//JOY1_R is an analog input on joystick 2
		gJoy2Buf[input_type_lookup[JOY1_R_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = (int8_t)(gJoy1_r_ana_val-128);
	}
	else{
		//JOY1_R is a digital input
		if(ioport_pin_is_low(JOY1_R)){
			checkJoystickDown(JOY1_R_LKUP_INDEX, JOY1_R_MASK);
		}
		else{
			checkJoystickUp(JOY1_R_LKUP_INDEX, JOY1_R_MASK);
		}
	}	
	
	//check joystick 2 inputs
	if(ioport_pin_is_low(JOY2_U)){
		checkJoystickDown(JOY2_U_LKUP_INDEX, JOY2_U_MASK);
	}
	else{
		checkJoystickUp(JOY2_U_LKUP_INDEX, JOY2_U_MASK);
	}
	if(ioport_pin_is_low(JOY2_D)){
		checkJoystickDown(JOY2_D_LKUP_INDEX, JOY2_D_MASK);
	}
	else{
		checkJoystickUp(JOY2_D_LKUP_INDEX, JOY2_D_MASK);
	}
	if(ioport_pin_is_low(JOY2_L)){
		checkJoystickDown(JOY2_L_LKUP_INDEX, JOY2_L_MASK);
	}
	else{
		checkJoystickUp(JOY2_L_LKUP_INDEX, JOY2_L_MASK);
	}
	if(ioport_pin_is_low(JOY2_R)){
		checkJoystickDown(JOY2_R_LKUP_INDEX, JOY2_R_MASK);
	}
	else{
		checkJoystickUp(JOY2_R_LKUP_INDEX, JOY2_R_MASK);
	}
	
	//check JOY3_U and JOY3_D inputs (Can be QEC1 for Mouse Axis)
	if((input_type_lookup[JOY3_U_LKUP_INDEX] >= IT_MOUSE_XAXIS)&&(input_type_lookup[JOY3_U_LKUP_INDEX] <= IT_MOUSE_ZAXIS)){
		//QEC1 Enabled
		qdec1_dir = qdec_get_direction(&gConfig1);
		qdec1_pos = qdec_get_position(&gConfig1);
		qdec1_freq = qdec_get_frequency(&gConfig1);
		if(qdec1_pos != gQdec1_prev_pos){
			gQdec1_prev_pos = qdec1_pos;
			if(qdec1_freq < 2){
				qdec1_freq = 1;
			}
			
			if(qdec1_dir == 1){
				qdec1_freq *= -1;				
			}
			
			switch(input_type_lookup[JOY3_U_LKUP_INDEX]){
				case IT_MOUSE_XAXIS:
					udi_hid_mouse_moveX(qdec1_freq);
					break;
				case IT_MOUSE_YAXIS:
					udi_hid_mouse_moveY(qdec1_freq);
					break;
				case IT_MOUSE_ZAXIS:
					udi_hid_mouse_moveScroll(qdec1_freq);
					break;
				default:
					break;
			}
		}
	}
	else{
		//QEC1 Disabled
		//if(ioport_pin_is_low(JOY3_U)){
		if(ioport_pin_is_high(JOY3_U)){ //inverted logic
			checkJoystickDown(JOY3_U_LKUP_INDEX, JOY3_U_MASK);
		}
		else{
			checkJoystickUp(JOY3_U_LKUP_INDEX, JOY3_U_MASK);
		}
		//if(ioport_pin_is_low(JOY3_D)){
		if(ioport_pin_is_high(JOY3_D)){ //inverted logic
			checkJoystickDown(JOY3_D_LKUP_INDEX, JOY3_D_MASK);
		}
		else{
			checkJoystickUp(JOY3_D_LKUP_INDEX, JOY3_D_MASK);
		}
	}
	
	//check JOY3_L and JOY3_R inputs (Can be QEC2 for Mouse Axis)
	if((input_type_lookup[JOY3_L_LKUP_INDEX] >= IT_MOUSE_XAXIS)&&(input_type_lookup[JOY3_L_LKUP_INDEX] <= IT_MOUSE_ZAXIS)){
		//QEC2 Enabled
		qdec2_dir = qdec_get_direction(&gConfig2);
		qdec2_pos = qdec_get_position(&gConfig2);
		qdec2_freq = qdec_get_frequency(&gConfig2);
		if(qdec2_pos != gQdec2_prev_pos){
			gQdec2_prev_pos = qdec2_pos;
			if(qdec2_freq < 2){
				qdec2_freq = 1;
			}
			
			if(qdec2_dir == 1){
				qdec2_freq *= -1;
			}
			
			switch(input_type_lookup[JOY3_L_LKUP_INDEX]){
				case IT_MOUSE_XAXIS:
					udi_hid_mouse_moveX(qdec2_freq);
					break;
				case IT_MOUSE_YAXIS:
					udi_hid_mouse_moveY(qdec2_freq);
					break;
				case IT_MOUSE_ZAXIS:
					udi_hid_mouse_moveScroll(qdec2_freq);
					break;
				default:
					break;
			}
		}
	}
	else{
		//QEC2 Disabled
		//if(ioport_pin_is_low(JOY3_L)){
		if(ioport_pin_is_high(JOY3_L)){ //inverted logic
			checkJoystickDown(JOY3_L_LKUP_INDEX, JOY3_L_MASK);
		}
		else{
			checkJoystickUp(JOY3_L_LKUP_INDEX, JOY3_L_MASK);
		}
		//if(ioport_pin_is_low(JOY3_R)){
		if(ioport_pin_is_high(JOY3_R)){ //inverted logic
			checkJoystickDown(JOY3_R_LKUP_INDEX, JOY3_R_MASK);
		}
		else{
			checkJoystickUp(JOY3_R_LKUP_INDEX, JOY3_R_MASK);
		}	
	}

	
	//check JOY4_L and JOY4_R inputs (Can be QEC3 for Mouse Axis)
	if((input_type_lookup[JOY4_L_LKUP_INDEX] >= IT_MOUSE_XAXIS)&&(input_type_lookup[JOY4_R_LKUP_INDEX] <= IT_MOUSE_ZAXIS)){
		//QEC3 Enabled
		qdec3_dir = qdec_get_direction(&gConfig3);
		qdec3_pos = qdec_get_position(&gConfig3);
		qdec3_freq = qdec_get_frequency(&gConfig3);
		if(qdec3_pos != gQdec3_prev_pos){
			gQdec3_prev_pos = qdec3_pos;
			if(qdec3_freq < 2){
				qdec3_freq = 1;
			}
			
			if(qdec3_dir == 1){
				qdec3_freq *= -1;
			}
			
			switch(input_type_lookup[JOY4_L_LKUP_INDEX]){
				case IT_MOUSE_XAXIS:
					udi_hid_mouse_moveX(qdec3_freq);
					break;
				case IT_MOUSE_YAXIS:
					udi_hid_mouse_moveY(qdec3_freq);
					break;
				case IT_MOUSE_ZAXIS:
					udi_hid_mouse_moveScroll(qdec3_freq);
					break;
				default:
					break;
			}
		}
	}
	else{
		//QEC3 Disabled
		//if(ioport_pin_is_low(JOY4_L)){
		if(ioport_pin_is_high(JOY4_L)){ //inverted logic
			checkJoystickDown(JOY4_L_LKUP_INDEX, JOY4_L_MASK);
		}
		else{
			checkJoystickUp(JOY4_L_LKUP_INDEX, JOY4_L_MASK);
		}
		//if(ioport_pin_is_low(JOY4_R)){
		if(ioport_pin_is_high(JOY4_R)){ //inverted logic
			checkJoystickDown(JOY4_R_LKUP_INDEX, JOY4_R_MASK);
		}
		else{
			checkJoystickUp(JOY4_R_LKUP_INDEX, JOY4_R_MASK);
		}
	}

	//check JOY4_U and JOY4_D buttons
	if(ioport_pin_is_low(JOY4_U)){
		checkJoystickDown(JOY4_U_LKUP_INDEX, JOY4_U_MASK);
	}
	else{
		checkJoystickUp(JOY4_U_LKUP_INDEX, JOY4_U_MASK);
	}
	if(ioport_pin_is_low(JOY4_D)){
		checkJoystickDown(JOY4_D_LKUP_INDEX, JOY4_D_MASK);
	}
	else{
		checkJoystickUp(JOY4_D_LKUP_INDEX, JOY4_D_MASK);
	}
	
	//Check Button inputs
	if(ioport_pin_is_low(BUTT1)){
		checkButtonDown(BUTT1_LKUP_INDEX, BUTT1_MASK);
	}
	else{
		checkButtonUp(BUTT1_LKUP_INDEX, BUTT1_MASK);
	}	
	if(ioport_pin_is_low(BUTT2)){
		checkButtonDown(BUTT2_LKUP_INDEX, BUTT2_MASK);
	}
	else{
		checkButtonUp(BUTT2_LKUP_INDEX, BUTT2_MASK);
	}
	
	if(ioport_pin_is_low(BUTT3)){
		checkButtonDown(BUTT3_LKUP_INDEX, BUTT3_MASK);
	}
	else{
		checkButtonUp(BUTT3_LKUP_INDEX, BUTT3_MASK);
	}
	
	if(ioport_pin_is_low(BUTT4)){
		checkButtonDown(BUTT4_LKUP_INDEX, BUTT4_MASK);
	}
	else{
		checkButtonUp(BUTT4_LKUP_INDEX, BUTT4_MASK);
	}
	
	if(ioport_pin_is_low(BUTT5)){
		checkButtonDown(BUTT5_LKUP_INDEX, BUTT5_MASK);
	}
	else{
		checkButtonUp(BUTT5_LKUP_INDEX, BUTT5_MASK);
	}
	
	if(ioport_pin_is_low(BUTT6)){
		checkButtonDown(BUTT6_LKUP_INDEX, BUTT6_MASK);
	}
	else{
		checkButtonUp(BUTT6_LKUP_INDEX, BUTT6_MASK);
	}
	
	if(ioport_pin_is_low(BUTT7)){
		checkButtonDown(BUTT7_LKUP_INDEX, BUTT7_MASK);
	}
	else{
		checkButtonUp(BUTT7_LKUP_INDEX, BUTT7_MASK);
	}
	
	if(ioport_pin_is_low(BUTT8)){
		checkButtonDown(BUTT8_LKUP_INDEX, BUTT8_MASK);
	}
	else{
		checkButtonUp(BUTT8_LKUP_INDEX, BUTT8_MASK);
	}
	
	if(ioport_pin_is_low(BUTT9)){
		checkButtonDown(BUTT9_LKUP_INDEX, BUTT9_MASK);
	}
	else{
		checkButtonUp(BUTT9_LKUP_INDEX, BUTT9_MASK);
	}
	
	if(ioport_pin_is_low(BUTT10)){
		checkButtonDown(BUTT10_LKUP_INDEX, BUTT10_MASK);
	}
	else{
		checkButtonUp(BUTT10_LKUP_INDEX, BUTT10_MASK);
	}
	
	if(ioport_pin_is_low(BUTT11)){
		checkButtonDown(BUTT11_LKUP_INDEX, BUTT11_MASK);
	}
	else{
		checkButtonUp(BUTT11_LKUP_INDEX, BUTT11_MASK);
	}
	
	if(ioport_pin_is_low(BUTT12)){
		checkButtonDown(BUTT12_LKUP_INDEX, BUTT12_MASK);
	}
	else{
		checkButtonUp(BUTT12_LKUP_INDEX, BUTT12_MASK);
	}
	
	if(ioport_pin_is_low(BUTT13)){
		checkButtonDown(BUTT13_LKUP_INDEX, BUTT13_MASK);
	}
	else{
		checkButtonUp(BUTT13_LKUP_INDEX, BUTT13_MASK);
	}
	
	if(ioport_pin_is_low(BUTT14)){
		checkButtonDown(BUTT14_LKUP_INDEX, BUTT14_MASK);
	}
	else{
		checkButtonUp(BUTT14_LKUP_INDEX, BUTT14_MASK);
	}
	
	if(ioport_pin_is_low(BUTT15)){
		checkButtonDown(BUTT15_LKUP_INDEX, BUTT15_MASK);
	}
	else{
		checkButtonUp(BUTT15_LKUP_INDEX, BUTT15_MASK);
	}
	
	if(ioport_pin_is_low(BUTT16)){
		checkButtonDown(BUTT16_LKUP_INDEX, BUTT16_MASK);
	}
	else{
		checkButtonUp(BUTT16_LKUP_INDEX, BUTT16_MASK);
	}
	
	if(ioport_pin_is_low(BUTT17)){
		checkButtonDown(BUTT17_LKUP_INDEX, BUTT17_MASK);
	}
	else{
		checkButtonUp(BUTT17_LKUP_INDEX, BUTT17_MASK);
	}
	
	if(ioport_pin_is_low(BUTT18)){
		checkButtonDown(BUTT18_LKUP_INDEX, BUTT18_MASK);
	}
	else{
		checkButtonUp(BUTT18_LKUP_INDEX, BUTT18_MASK);
	}
	
	if(ioport_pin_is_low(BUTT19)){
		checkButtonDown(BUTT19_LKUP_INDEX, BUTT19_MASK);
	}
	else{
		checkButtonUp(BUTT19_LKUP_INDEX, BUTT19_MASK);
	}
	
	if(ioport_pin_is_low(BUTT20)){
		checkButtonDown(BUTT20_LKUP_INDEX, BUTT20_MASK);
	}
	else{
		checkButtonUp(BUTT20_LKUP_INDEX, BUTT20_MASK);
	}
	
	if(ioport_pin_is_low(BUTT21)){
		checkButtonDown(BUTT21_LKUP_INDEX, BUTT21_MASK);
	}
	else{
		checkButtonUp(BUTT21_LKUP_INDEX, BUTT21_MASK);
	}
	
	if(ioport_pin_is_low(BUTT22)){
		checkButtonDown(BUTT22_LKUP_INDEX, BUTT22_MASK);
	}
	else{
		checkButtonUp(BUTT22_LKUP_INDEX, BUTT22_MASK);
	}
	
	if(ioport_pin_is_low(BUTT23)){
		checkButtonDown(BUTT23_LKUP_INDEX, BUTT23_MASK);
	}
	else{
		checkButtonUp(BUTT23_LKUP_INDEX, BUTT23_MASK);
	}
	
	if(ioport_pin_is_low(BUTT24)){
		checkButtonDown(BUTT24_LKUP_INDEX, BUTT24_MASK);
	}
	else{
		checkButtonUp(BUTT24_LKUP_INDEX, BUTT24_MASK);
	}
	
	if(ioport_pin_is_low(BUTT25)){
		checkButtonDown(BUTT25_LKUP_INDEX, BUTT25_MASK);
	}
	else{
		checkButtonUp(BUTT25_LKUP_INDEX, BUTT25_MASK);
	}
	
	if(ioport_pin_is_low(BUTT26)){
		checkButtonDown(BUTT26_LKUP_INDEX, BUTT26_MASK);
	}
	else{
		checkButtonUp(BUTT26_LKUP_INDEX, BUTT26_MASK);
	}
	
	//Accelerometer code here:
	if((input_type_lookup[ACCEL_XAXIS_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//ACCEL_XAXIS is an analog input
		gJoy1Buf[input_type_lookup[ACCEL_XAXIS_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = gAccelData[0];
	}
	else if((input_type_lookup[ACCEL_XAXIS_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//ACCEL_XAXIS is an analog input
		gJoy2Buf[input_type_lookup[ACCEL_XAXIS_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = gAccelData[0];
	}
	else{
		//ACCEL_XAXIS is a digital input
		if((gAccelData[0] >= gAccelMinTrig[0]) &&(gAccelData[0] <= gAccelMaxTrig[0])){
			checkButtonDown(ACCEL_XAXIS_LKUP_INDEX, ACCEL_XAXIS_MASK);
		}
		else{
			checkButtonUp(ACCEL_XAXIS_LKUP_INDEX, ACCEL_XAXIS_MASK);
		}
	}
	
	if((input_type_lookup[ACCEL_YAXIS_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//ACCEL_YAXIS is an analog input
		gJoy1Buf[input_type_lookup[ACCEL_YAXIS_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = gAccelData[1];
	}
	else if((input_type_lookup[ACCEL_YAXIS_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//ACCEL_YAXIS is an analog input
		gJoy2Buf[input_type_lookup[ACCEL_YAXIS_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = gAccelData[1];
	}
	else{
		//ACCEL_YAXIS is a digital input
		if((gAccelData[1] >= gAccelMinTrig[1]) &&(gAccelData[1] <= gAccelMaxTrig[1])){
			checkButtonDown(ACCEL_YAXIS_LKUP_INDEX, ACCEL_YAXIS_MASK);
		}
		else{
			checkButtonUp(ACCEL_YAXIS_LKUP_INDEX, ACCEL_YAXIS_MASK);
		}
	}
	if((input_type_lookup[ACCEL_ZAXIS_LKUP_INDEX] & IT_JOY1_ANA_OFFSET) == IT_JOY1_ANA_OFFSET){
		//ACCEL_ZAXIS is an analog input
		gJoy1Buf[input_type_lookup[ACCEL_ZAXIS_LKUP_INDEX] - IT_JOY1_ANA_OFFSET] = gAccelData[2];
	}
	else if((input_type_lookup[ACCEL_ZAXIS_LKUP_INDEX] & IT_JOY2_ANA_OFFSET) == IT_JOY2_ANA_OFFSET){
		//ACCEL_ZAXIS is an analog input
		gJoy2Buf[input_type_lookup[ACCEL_ZAXIS_LKUP_INDEX] - IT_JOY2_ANA_OFFSET] = gAccelData[2];
	}
	else{
		//ACCEL_ZAXIS is a digital input
		if((gAccelData[2] >= gAccelMinTrig[2]) &&(gAccelData[2] <= gAccelMaxTrig[2])){
			checkButtonDown(ACCEL_ZAXIS_LKUP_INDEX, ACCEL_ZAXIS_MASK);
		}
		else{
			checkButtonUp(ACCEL_ZAXIS_LKUP_INDEX, ACCEL_ZAXIS_MASK);
		}
	}
	
	send_joystick_report(gJoy1Buf);
	send_joystick2_report(gJoy2Buf);
	

}

static void qdec1_init(void)
{
	qdec_get_config_defaults(&gConfig1);
	qdec_config_disable_index_pin (&gConfig1);
	qdec_config_event_channel(&gConfig1, 0);
	qdec_config_freq_event_channel(&gConfig1, 1);
	qdec_config_enable_freq(&gConfig1, 1200);
	qdec_config_tc(&gConfig1, &TCC0);	
	qdec_config_freq_tc(&gConfig1, &TCC1);
	//qdec_config_phase_pins(&gConfig1, &PORTD, 2, false, 50);
	qdec_config_phase_pins(&gConfig1, &PORTD, 2, true, 50);
	qdec_config_revolution(&gConfig1, 200);	
	qdec_enabled(&gConfig1);
}

static void qdec2_init(void)
{
	qdec_get_config_defaults(&gConfig2);
	qdec_config_disable_index_pin (&gConfig2);
	qdec_config_event_channel(&gConfig2, 2);
	qdec_config_freq_event_channel(&gConfig2, 3);
	qdec_config_enable_freq(&gConfig2, 1200);
	qdec_config_tc(&gConfig2, &TCD0);	
	qdec_config_freq_tc(&gConfig2, &TCD1);
	//qdec_config_phase_pins(&gConfig2, &PORTD,  0, false, 50);
	qdec_config_phase_pins(&gConfig2, &PORTD,  0, true, 50);
	qdec_config_revolution(&gConfig2, 200);	
	qdec_enabled(&gConfig2);
}

static void qdec3_init(void)
{
	qdec_get_config_defaults(&gConfig3);
	qdec_config_disable_index_pin (&gConfig3);
	qdec_config_event_channel(&gConfig3, 4);
	qdec_config_freq_event_channel(&gConfig3, 5);
	qdec_config_enable_freq(&gConfig3, 1200);
	qdec_config_tc(&gConfig3, &TCE0);
	qdec_config_freq_tc(&gConfig3, &TCE1);
	//qdec_config_phase_pins(&gConfig3, &PORTD, 4, false, 50);
	qdec_config_phase_pins(&gConfig3, &PORTD, 4, true, 50);
	qdec_config_revolution(&gConfig3, 200);
	qdec_enabled(&gConfig3);
}

static void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result)
{
	switch (ch_mask) {
		case ADC_CH0:
			gJoy1_u_ana_val = result>>4;
			break;
		case ADC_CH1:
			gJoy1_d_ana_val = result>>4;
			break;
		case ADC_CH2:
			gJoy1_l_ana_val = result>>4;
			break;
		case ADC_CH3:
			gJoy1_r_ana_val = result>>4;
			break;
		default:
			break;
	}
}

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&JOY_ADC_MODULE, &adc_conf);
	adcch_read_configuration(&JOY_ADC_MODULE, ADC_CH0, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_AREFB);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 4, 0);
	adc_set_clock_rate(&adc_conf, 5000UL);
	adc_set_callback(&JOY_ADC_MODULE, &adc_handler);
	adc_write_configuration(&JOY_ADC_MODULE, &adc_conf);
	
	adcch_enable_interrupt(&adcch_conf);
	adcch_set_input(&adcch_conf, JOY1_U_ADC_INPUT, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&JOY_ADC_MODULE, ADC_CH0, &adcch_conf);
	
	adcch_set_input(&adcch_conf, JOY1_D_ADC_INPUT, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&JOY_ADC_MODULE, ADC_CH1, &adcch_conf);
	
	adcch_set_input(&adcch_conf, JOY1_L_ADC_INPUT, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&JOY_ADC_MODULE, ADC_CH2, &adcch_conf);
	
	adcch_set_input(&adcch_conf, JOY1_R_ADC_INPUT, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&JOY_ADC_MODULE, ADC_CH3, &adcch_conf);
	
}

void checkJoystickDown(uint8_t index, uint64_t mask){
	if((input_type_lookup[index] & IT_JOY1_DIG_OFFSET) == IT_JOY1_DIG_OFFSET){
		//digital joystick input
		gJoy1Buf[input_type_lookup[index] - IT_JOY1_DIG_OFFSET] = input_value_lookup[index]; 
	}
	else if((input_type_lookup[index] & IT_JOY2_DIG_OFFSET) == IT_JOY2_DIG_OFFSET){
		//digital joystick input
		gJoy2Buf[input_type_lookup[index] - IT_JOY2_DIG_OFFSET] = input_value_lookup[index];
	}
	else if(input_type_lookup[index] == IT_JOY1_BUTTON){
		//joystick button input
		joyButtonPressed(input_value_lookup[index], 1);
	}
	else if(input_type_lookup[index] == IT_JOY2_BUTTON){
		//joystick button input
		joyButtonPressed(input_value_lookup[index], 2);
	}
	else if(input_type_lookup[index] == IT_MOUSE_BUTTON){
		//mouse button input
		if((gKeyPressed & mask) != mask){
			gKeyPressed |= mask;
			if(input_value_lookup[index] == 1){
				udi_hid_mouse_btnleft(1);
			}
			else if(input_value_lookup[index] == 2){
				udi_hid_mouse_btnright(1);
			}
			else if(input_value_lookup[index] == 3){
				udi_hid_mouse_btnmiddle(1);
			}
		}		
	}
	else if(input_type_lookup[index] == IT_KEYBOARD){
		//keyboard input
		if((gKeyPressed & mask) != mask){
			gKeyPressed |= mask;
			udi_hid_kbd_modifier_down(input_value2_lookup[index]);
			udi_hid_kbd_down(input_value_lookup[index]);
		}
	}	
	
}

void checkJoystickUp(uint8_t index, uint64_t mask){
	if(input_type_lookup[index] == IT_KEYBOARD){
		//keyboard input
		if((gKeyPressed & mask) == mask){
			gKeyPressed ^= mask;
			udi_hid_kbd_modifier_up(input_value2_lookup[index]);
			udi_hid_kbd_up(input_value_lookup[index]);
		}	
	}	
	else if(input_type_lookup[index] == IT_MOUSE_BUTTON){
		//mouse button input
		if((gKeyPressed & mask) == mask){
			gKeyPressed ^= mask;
			if(input_value_lookup[index] == 1){
				udi_hid_mouse_btnleft(0);
			}
			else if(input_value_lookup[index] == 2){
				udi_hid_mouse_btnright(0);
			}
			else if(input_value_lookup[index] == 3){
				udi_hid_mouse_btnmiddle(0);
			}
		}		
	}
}

void checkButtonDown(uint8_t index, uint64_t mask){
	if(input_type_lookup[index] == IT_JOY1_BUTTON){
		//joystick button input
		joyButtonPressed(input_value_lookup[index], 1);
	}
	else if(input_type_lookup[index] == IT_JOY2_BUTTON){
		//joystick button input
		joyButtonPressed(input_value_lookup[index], 2);
	}
	else if(input_type_lookup[index] == IT_KEYBOARD){
		//keyboard input
		if((gKeyPressed & mask) != mask){
			gKeyPressed |= mask;
			udi_hid_kbd_modifier_down(input_value2_lookup[index]);
			udi_hid_kbd_down(input_value_lookup[index]);
		}
	}	
	else if(input_type_lookup[index] == IT_MOUSE_BUTTON){
		//mouse button input
		if((gKeyPressed & mask) != mask){
			gKeyPressed |= mask;
			if(input_value_lookup[index] == 1){
				udi_hid_mouse_btnleft(1);
			}	
			else if(input_value_lookup[index] == 2){
				udi_hid_mouse_btnright(1);
			}
			else if(input_value_lookup[index] == 3){
				udi_hid_mouse_btnmiddle(1);
			}
		}		
	}
}

void checkButtonUp(uint8_t index, uint64_t mask){
	if(input_type_lookup[index] == IT_KEYBOARD){
		//keyboard input
		if((gKeyPressed & mask) == mask){
			gKeyPressed ^= mask;
			udi_hid_kbd_modifier_up(input_value2_lookup[index]);
			udi_hid_kbd_up(input_value_lookup[index]);
		}	
	}
	else if(input_type_lookup[index] == IT_MOUSE_BUTTON){
		//mouse button input
		if((gKeyPressed & mask) == mask){
			gKeyPressed ^= mask;
			if(input_value_lookup[index] == 1){
				udi_hid_mouse_btnleft(0);
			}
			else if(input_value_lookup[index] == 2){
				udi_hid_mouse_btnright(0);
			}
			else if(input_value_lookup[index] == 3){
				udi_hid_mouse_btnmiddle(0);
			}
		}			
	}		
}

void joyButtonPressed(uint8_t button, uint8_t joystick){
	uint8_t index = 0;
	uint8_t value = 0;
		
	switch(button){
		case 1:
			index = 8;
			value = 0x01;
			break;
		case 2:
			index = 8;
			value = 0x02;
			break;
		case 3:
			index = 8;
			value = 0x04;
			break;
		case 4:
			index = 8;
			value = 0x08;
			break;
		case 5:
			index = 8;
			value = 0x10;
			break;
		case 6:
			index = 8;
			value = 0x20;
			break;
		case 7:
			index = 8;
			value = 0x40;
			break;
		case 8:
			index = 8;
			value = 0x80;
			break;
		case 9:
			index = 9;
			value = 0x01;
			break;
		case 10:
			index = 9;
			value = 0x02;
			break;
		case 11:
			index = 9;
			value = 0x04;
			break;
		case 12:
			index = 9;
			value = 0x08;
			break;
		case 13:
			index = 9;
			value = 0x10;
			break;
		case 14:
			index = 9;
			value = 0x20;
			break;
		case 15:
			index = 9;
			value = 0x40;
			break;
		case 16:
			index = 9;
			value = 0x80;
			break;
		case 17:
			index = 10;
			value = 0x01;
			break;
		case 18:
			index = 10;
			value = 0x02;
			break;
		case 19:
			index = 10;
			value = 0x04;
			break;
		case 20:
			index = 10;
			value = 0x08;
			break;
		case 21:
			index = 10;
			value = 0x10;
			break;
		case 22:
			index = 10;
			value = 0x20;
			break;
		case 23:
			index = 10;
			value = 0x40;
			break;
		case 24:
			index = 10;
			value = 0x80;
			break;
		case 25:
			index = 11;
			value = 0x01;
			break;
		case 26:
			index = 11;
			value = 0x02;
			break;
		case 27:
			index = 11;
			value = 0x04;
			break;
		case 28:
			index = 11;
			value = 0x08;
			break;
		case 29:
			index = 11;
			value = 0x10;
			break;
		case 30:
			index = 11;
			value = 0x20;
			break;
		case 31:
			index = 11;
			value = 0x40;
			break;
		case 32:
			index = 11;
			value = 0x80;
			break;
		default:
			break;	
	}
	if(joystick == 1){
		gJoy1Buf[index] |= value;
	}
	else if(joystick == 2){
		gJoy2Buf[index] |= value;
	}
}

void parseHidReport(void){
	int i = 0;
	uint16_t positionX = 0;
	uint16_t freqX = 0;
	bool qdec_dirX = 0;
	uint16_t positionY = 0;
	uint16_t freqY = 0;
	bool qdec_dirY = 0;
	uint8_t controlSet = 0xAF;
	uint16_t accelReadingX = 0;
	uint8_t ledRed = 0;
	uint8_t ledGreen = 0;
	uint8_t ledBlue = 0;
	uint8_t bankValues[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
	memset(howler_hid_report, 0x00, UDI_HID_REPORT_IN_SIZE);
	
	if(gReport[0] == HOWLER_ID){ //eventually make this the HowlerID
		switch(gReport[1]){
			case CMD_SET_RGB_LED:
				//set RGB Led				
				if(gReport[2] >= 0 && gReport[2] <= (NUM_RGB_LEDS-1)){
					setLED(ledr_look_up[gReport[2]] ,gReport[3]);
					setLED(ledg_look_up[gReport[2]] ,gReport[4]);
					setLED(ledb_look_up[gReport[2]] ,gReport[5]);
				}	
				break;
			case CMD_GET_RGB_LED:
				//get RGB Led
				if(gReport[2] >= 0 && gReport[2] <= (NUM_RGB_LEDS-1)){
					getLED(ledr_look_up[gReport[2]] ,&ledRed);
					getLED(ledg_look_up[gReport[2]] ,&ledGreen);
					getLED(ledb_look_up[gReport[2]] ,&ledBlue);
					
					howler_hid_report[0] = HOWLER_ID; //eventually make howlerid
					howler_hid_report[1] = CMD_GET_RGB_LED;
					howler_hid_report[2] = ledRed;
					howler_hid_report[3] = ledGreen;
					howler_hid_report[4] = ledBlue;
					howler_hid_report[5] = 0x00;
					howler_hid_report[6] = 0x00;
					howler_hid_report[7] = 0x00;
					udi_hid_generic_send_report_in(howler_hid_report);
				}
				break;
			case CMD_SET_RGB_LED_BANK:
				//set RGB Led Bank
				if(gReport[2] >= 1 && gReport[2] <= 6){	
					
					for(i = 0; i < 16; i++)
					{
						bankValues[i] = gReport[i + 3];
					}
								
					setLEDBank(gReport[2], bankValues);
				}
				break;
				
			case CMD_SET_INDIVIDUAL_LED:
				//set individual LED
				if(gReport[2] >= 0 && gReport[2] <= (NUM_LEDS-1)){
					setLED(led_look_up[gReport[2]] ,gReport[3]);
				}
				break;
			case CMD_SET_INPUT:
				//Set Input, Sets the input type and value of the specified Input
				if(gReport[2] >= JOY1_U_LKUP_INDEX && gReport[2] <= JOY1_R_LKUP_INDEX){
					//JOY1 Digital AND Analog Axis
					if(gReport[3] == IT_NOTHING){
						//No assignment for this pin
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
					else if((gReport[3] == IT_JOY1_BUTTON)||(gReport[3] == IT_JOY2_BUTTON)){
						//Joystick Button
						if(gReport[4] > 0 && gReport[4] <= NUM_JOYSTICK_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if(gReport[3] == IT_KEYBOARD){
						//Keyboard Button
						//check for valid key (0 if only modifier key is being sent)
						if((gReport[4] == 0) ||(gReport[4] >= HID_A && gReport[4] <= HID_KEYPAD_0)){
							//check for valid modifier key (0 if no modifier key)
							if((gReport[5] == 0) ||(gReport[5] >= HID_MODIFIER_NONE && gReport[5] <= HID_MODIFIER_RIGHT_UI)){
								eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
								eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
								eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
								input_type_lookup[gReport[2]] = gReport[3];
								input_value_lookup[gReport[2]] = gReport[4];
								input_value2_lookup[gReport[2]] = gReport[5];
								controlSet = 1;
							}
						}
					}
					else if(gReport[3] == IT_MOUSE_BUTTON){
						//Mouse Button
						if(gReport[4] > 0 && gReport[4] <= NUM_MOUSE_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if((gReport[3] >= IT_JOY1_DIG_THROTTLE && gReport[3] <= IT_JOY1_DIG_SLIDER)||(gReport[3] >= IT_JOY2_DIG_THROTTLE && gReport[3] <= IT_JOY2_DIG_SLIDER)){
						//Digital Axis
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						//axis value
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
						
					}
					else if((gReport[3] >= IT_JOY1_ANA_THROTTLE && gReport[3] <= IT_JOY1_ANA_SLIDER)||(gReport[3] >= IT_JOY2_ANA_THROTTLE && gReport[3] <= IT_JOY2_ANA_SLIDER)){
						//Analog Axis
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						//no need to set value for analog axis
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
				}
				
				//JOY2 and JOY4_L, JOY4_R
				if((gReport[2] >= JOY2_U_LKUP_INDEX && gReport[2] <= JOY2_R_LKUP_INDEX)||(gReport[2] >= JOY4_U_LKUP_INDEX && gReport[2] <= JOY4_D_LKUP_INDEX)){
					//JOY2 and JOY4_U, JOY4_D Digital Axis
					if(gReport[3] == IT_NOTHING){
						//No assignment for this pin
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
					else if((gReport[3] == IT_JOY1_BUTTON)||(gReport[3] == IT_JOY2_BUTTON)){
						//Joystick Button
						if(gReport[4] > 0 && gReport[4] <= NUM_JOYSTICK_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if(gReport[3] == IT_KEYBOARD){
						//Keyboard Button
						//check for valid key (0 if only modifier key is being sent)
						if((gReport[4] == 0) ||(gReport[4] >= HID_A && gReport[4] <= HID_KEYPAD_0)){
							//check for valid modifier key (0 if no modifier key)
							if((gReport[5] == 0) ||(gReport[5] >= HID_MODIFIER_NONE && gReport[5] <= HID_MODIFIER_RIGHT_UI)){
								eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
								eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
								eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
								input_type_lookup[gReport[2]] = gReport[3];
								input_value_lookup[gReport[2]] = gReport[4];
								input_value2_lookup[gReport[2]] = gReport[5];
								controlSet = 1;
							}
						}
					}
					else if(gReport[3] == IT_MOUSE_BUTTON){
						//Mouse Button
						if(gReport[4] > 0 && gReport[4] <= NUM_MOUSE_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if((gReport[3] >= IT_JOY1_DIG_THROTTLE && gReport[3] <= IT_JOY1_DIG_SLIDER)||(gReport[3] >= IT_JOY2_DIG_THROTTLE && gReport[3] <= IT_JOY2_DIG_SLIDER)){
						//Digital Axis
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						//axis value
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
						
					}
				}
				
				//JOY3 and JOY4_U, JOY4_D
				else if((gReport[2] >= JOY3_U_LKUP_INDEX && gReport[2] <= JOY3_R_LKUP_INDEX)||(gReport[2] >= JOY4_L_LKUP_INDEX && gReport[2] <= JOY4_R_LKUP_INDEX)){
					//JOY3 and JOY4U, JOY4_D, Digital Axis AND Quadrature Mouse Axis
					
					//Need to invert pin for qudrature encoding
					/*if((gReport[2] == JOY3_U_LKUP_INDEX)&&(gReport[3] >= IT_MOUSE_XAXIS && gReport[3] <= IT_MOUSE_ZAXIS)){
						qdec_config_phase_pins(&gConfig1, &PORTD, 2, true, 50);
					}
					else{
						qdec_config_phase_pins(&gConfig1, &PORTD, 2, false, 50);
					}
					if((gReport[2] == JOY3_L_LKUP_INDEX)&&(gReport[3] >= IT_MOUSE_XAXIS && gReport[3] <= IT_MOUSE_ZAXIS)){
						qdec_config_phase_pins(&gConfig1, &PORTD, 0, true, 50);
					}
					else{
						qdec_config_phase_pins(&gConfig1, &PORTD, 0, false, 50);
					}
					if((gReport[2] == JOY4_U_LKUP_INDEX)&&(gReport[3] >= IT_MOUSE_XAXIS && gReport[3] <= IT_MOUSE_ZAXIS)){
						qdec_config_phase_pins(&gConfig1, &PORTD, 4, true, 50);
					}
					else{
						qdec_config_phase_pins(&gConfig1, &PORTD, 4, false, 50);
					}
					*/
					
					if(gReport[3] == IT_NOTHING){
						//No assignment for this pin
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
					else if((gReport[3] == IT_JOY1_BUTTON)||(gReport[3] == IT_JOY2_BUTTON)){
						//Joystick Button
						if(gReport[4] > 0 && gReport[4] <= NUM_JOYSTICK_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if(gReport[3] == IT_KEYBOARD){
						//Keyboard Button
						//check for valid key (0 if only modifier key is being sent)
						if((gReport[4] == 0) ||(gReport[4] >= HID_A && gReport[4] <= HID_KEYPAD_0)){
							//check for valid modifier key (0 if no modifier key)
							if((gReport[5] == 0) ||(gReport[5] >= HID_MODIFIER_NONE && gReport[5] <= HID_MODIFIER_RIGHT_UI)){
								eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
								eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
								eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
								input_type_lookup[gReport[2]] = gReport[3];
								input_value_lookup[gReport[2]] = gReport[4];
								input_value2_lookup[gReport[2]] = gReport[5];
								controlSet = 1;
							}							
						}
					}
					else if(gReport[3] == IT_MOUSE_BUTTON){
						//Mouse Button
						if(gReport[4] > 0 && gReport[4] <= NUM_MOUSE_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if(gReport[3] >= IT_MOUSE_XAXIS && gReport[3] <= IT_MOUSE_ZAXIS){
						if((gReport[2] == JOY3_U_LKUP_INDEX) || (gReport[2] == JOY3_L_LKUP_INDEX) || (gReport[2] == JOY4_L_LKUP_INDEX)){							
						//if((gReport[2] >= JOY3_U_LKUP_INDEX) && (gReport[2] <= JOY4_D_LKUP_INDEX)){
							//3 Quadrature Mouse axes (JOY3_L & JOY3R are QDEC1, JOY3_U and JOY3_D are QEC2, JOY4_U and JOY4_D are QES3)
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							//no need to set value for mouse axis
							input_value_lookup[gReport[2]] = gReport[4];	
							input_value2_lookup[gReport[2]] = gReport[5];	
							
							//Set both pins the same
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]+1],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]+1],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]+1],gReport[5]);
							input_type_lookup[gReport[2]+1] = gReport[3];
							//no need to set value for mouse axis
							input_value_lookup[gReport[2]+1] = gReport[4];	
							input_value2_lookup[gReport[2]+1] = gReport[5];	
							controlSet = 1;
						}
					}
					else if((gReport[3] >= IT_JOY1_DIG_THROTTLE && gReport[3] <= IT_JOY1_DIG_SLIDER)||(gReport[3] >= IT_JOY2_DIG_THROTTLE && gReport[3] <= IT_JOY2_DIG_SLIDER)){
						//Digital Axis
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						//axis value
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
						
					}
				}
				
				//Buttons
				else if(gReport[2] >= BUTT1_LKUP_INDEX && gReport[2] <= BUTT26_LKUP_INDEX){
					//Buttons can be IT_NOTHING, IT_JOY_BUTTON, IT_KEYBOARD, or IT_MOUSE_BUTTON
					if(gReport[3] == IT_NOTHING){
						//No assignment for this pin
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
					else if((gReport[3] == IT_JOY1_BUTTON)||(gReport[3] == IT_JOY2_BUTTON)){
						//Joystick Button
						if(gReport[4] > 0 && gReport[4] <= NUM_JOYSTICK_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];	
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}						
					}
					else if(gReport[3] == IT_KEYBOARD){
						//Keyboard Button
						//check for valid key (0 if only modifier key is being sent)
						if((gReport[4] == 0) ||(gReport[4] >= HID_A && gReport[4] <= HID_KEYPAD_0)){
							//check for valid modifier key (0 if no modifier key)
							if((gReport[5] == 0) ||(gReport[5] >= HID_MODIFIER_NONE && gReport[5] <= HID_MODIFIER_RIGHT_UI)){
								eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
								eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
								eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
								input_type_lookup[gReport[2]] = gReport[3];
								input_value_lookup[gReport[2]] = gReport[4];
								input_value2_lookup[gReport[2]] = gReport[5];
								controlSet = 1;
							}
						}						
					}
					else if(gReport[3] == IT_MOUSE_BUTTON){
						//Mouse Button
						if(gReport[4] > 0 && gReport[4] <= NUM_MOUSE_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}				
				}
				
				//Accelerometer
				else if(gReport[2] >= ACCEL_XAXIS_LKUP_INDEX && gReport[2] <= ACCEL_ZAXIS_LKUP_INDEX){
					//Accelerometer can be IT_NOTHING, IT_JOY_BUTTON, IT_KEYBOARD, or IT_MOUSE_BUTTON, JOY_ANA_x
					
					if(gReport[2] == ACCEL_XAXIS_LKUP_INDEX){
						eeprom_write_byte(ACCEL_XAXIS_MIN_TRIG_ADDR, gReport[6]);
						eeprom_write_byte(ACCEL_XAXIS_MAX_TRIG_ADDR, gReport[7]);
						gAccelMinTrig[0] = gReport[6];
						gAccelMaxTrig[0] = gReport[7];
					}
					else if(gReport[2] == ACCEL_YAXIS_LKUP_INDEX){
						eeprom_write_byte(ACCEL_YAXIS_MIN_TRIG_ADDR, gReport[6]);
						eeprom_write_byte(ACCEL_YAXIS_MAX_TRIG_ADDR, gReport[7]);
						gAccelMinTrig[1] = gReport[6];
						gAccelMaxTrig[1] = gReport[7];
					}
					else if(gReport[2] == ACCEL_ZAXIS_LKUP_INDEX){
						eeprom_write_byte(ACCEL_ZAXIS_MIN_TRIG_ADDR, gReport[6]);
						eeprom_write_byte(ACCEL_ZAXIS_MAX_TRIG_ADDR, gReport[7]);
						gAccelMinTrig[2] = gReport[6];
						gAccelMaxTrig[2] = gReport[7];
					}
					
					if(gReport[3] == IT_NOTHING){
						//No assignment for this pin
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}
					else if((gReport[3] == IT_JOY1_BUTTON)||(gReport[3] == IT_JOY2_BUTTON)){
						//Joystick Button (the value is the accelerometer threshold to trigger joystick button)
						if(gReport[4] > 0 && gReport[4] <= NUM_JOYSTICK_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if(gReport[3] == IT_KEYBOARD){
						//Keyboard Button (the value is the accelerometer threshold to trigger key press)
						//check for valid key (0 if only modifier key is being sent)
						if((gReport[4] == 0) ||(gReport[4] >= HID_A && gReport[4] <= HID_KEYPAD_0)){
							//check for valid modifier key (0 if no modifier key)
							if((gReport[5] == 0) ||(gReport[5] >= HID_MODIFIER_NONE && gReport[5] <= HID_MODIFIER_RIGHT_UI)){
								eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
								eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
								eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
								input_type_lookup[gReport[2]] = gReport[3];
								input_value_lookup[gReport[2]] = gReport[4];
								input_value2_lookup[gReport[2]] = gReport[5];
								controlSet = 1;
							}
						}
					}
					else if(gReport[3] == IT_MOUSE_BUTTON){
						//Mouse Button (the value is the accelerometer threshold to trigger mouse button)
						if(gReport[4] > 0 && gReport[4] <= NUM_MOUSE_BUTTONS){
							eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
							eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
							eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
							input_type_lookup[gReport[2]] = gReport[3];
							input_value_lookup[gReport[2]] = gReport[4];
							input_value2_lookup[gReport[2]] = gReport[5];
							controlSet = 1;
						}
					}
					else if((gReport[3] >= IT_JOY1_ANA_THROTTLE && gReport[3] <= IT_JOY1_ANA_SLIDER)||(gReport[3] >= IT_JOY2_ANA_THROTTLE && gReport[3] <= IT_JOY2_ANA_SLIDER)){
						//Analog Axis
						eeprom_write_byte(eeprom_input_type_look_up[gReport[2]],gReport[3]);
						eeprom_write_byte(eeprom_input_value_look_up[gReport[2]],gReport[4]);
						eeprom_write_byte(eeprom_input_value2_look_up[gReport[2]],gReport[5]);
						input_type_lookup[gReport[2]] = gReport[3];
						input_value_lookup[gReport[2]] = gReport[4];
						input_value2_lookup[gReport[2]] = gReport[5];
						controlSet = 1;
					}				
				}
			
				howler_hid_report[0] = HOWLER_ID; //eventually make howlerid
				howler_hid_report[1] = CMD_SET_INPUT;
				howler_hid_report[2] = gReport[2];
				howler_hid_report[3] = gReport[3];
				howler_hid_report[4] = gReport[4];
				howler_hid_report[5] = gReport[5];
				howler_hid_report[6] = gReport[6];
				howler_hid_report[7] = gReport[7];
				howler_hid_report[8] = controlSet; //1 if set, 0xAF if not set
				udi_hid_generic_send_report_in(howler_hid_report);
				break;
			case CMD_GET_INPUT:
				//Get Input, Returns the Input Type and Value of requested Input				
				howler_hid_report[0] = HOWLER_ID; 
				howler_hid_report[1] = CMD_GET_INPUT; //Get Input Command Code
				howler_hid_report[2] = gReport[2]; //Input Requested
				howler_hid_report[3] = eeprom_read_byte(eeprom_input_type_look_up[gReport[2]]); //Input Type
				howler_hid_report[4] = eeprom_read_byte(eeprom_input_value_look_up[gReport[2]]); //Input Value
				howler_hid_report[5] = eeprom_read_byte(eeprom_input_value2_look_up[gReport[2]]); //Input Value2
				
				if(gReport[2] == ACCEL_XAXIS_LKUP_INDEX){
					howler_hid_report[6] = eeprom_read_byte(ACCEL_XAXIS_MIN_TRIG_ADDR);
					howler_hid_report[7] = eeprom_read_byte(ACCEL_XAXIS_MAX_TRIG_ADDR);
				}
				else if(gReport[2] == ACCEL_YAXIS_LKUP_INDEX){
					howler_hid_report[6] = eeprom_read_byte(ACCEL_YAXIS_MIN_TRIG_ADDR);
					howler_hid_report[7] = eeprom_read_byte(ACCEL_YAXIS_MAX_TRIG_ADDR);
				}
				else if(gReport[2] == ACCEL_ZAXIS_LKUP_INDEX){
					howler_hid_report[6] = eeprom_read_byte(ACCEL_ZAXIS_MIN_TRIG_ADDR);
					howler_hid_report[7] = eeprom_read_byte(ACCEL_ZAXIS_MAX_TRIG_ADDR);
				}
				else{
					//Not an accelerometer control
					howler_hid_report[6] = 0x00;
					howler_hid_report[7] = 0x00;
				}
				udi_hid_generic_send_report_in(howler_hid_report);
				break;
			case CMD_SET_GLOBAL_BRIGHTNESS:
				setLEDglobalbrightness(gReport[2]);
				break;
			case CMD_SET_DEFAULT:
				//Set to Defaults
				setInputDefaults();

				//get eeprom memory to sram
				for(i = 0; i < NUM_INPUTS; i++){
					input_type_lookup[i] = eeprom_read_byte(eeprom_input_type_look_up[i]);
					input_value_lookup[i] = eeprom_read_byte(eeprom_input_value_look_up[i]);
				}
				
				howler_hid_report[0] = HOWLER_ID; //eventually make howlerid
				howler_hid_report[1] = CMD_SET_DEFAULT;
				howler_hid_report[2] = 0x01;
				howler_hid_report[3] = 0x00;
				howler_hid_report[4] = 0x00;
				howler_hid_report[5] = 0x00;
				howler_hid_report[6] = 0x00;
				howler_hid_report[7] = 0x00;
				udi_hid_generic_send_report_in(howler_hid_report);
				
				break;
			
			case CMD_GET_FW_REV:	
				//return firmware revision
				howler_hid_report[0] = HOWLER_ID; 
				howler_hid_report[1] = CMD_GET_FW_REV;
				howler_hid_report[2] = VERSION_MAJOR;
				howler_hid_report[3] = VERSION_MINOR;
				howler_hid_report[4] = 0x00;
				howler_hid_report[5] = 0x00;
				howler_hid_report[6] = 0x00;
				howler_hid_report[7] = 0x00;			
				udi_hid_generic_send_report_in(howler_hid_report);
				break;
			case CMD_SET_RGB_LED_DEFAULT:
				//set RGB LED DEFAULT
				controlSet = 0xAF;
				if(gReport[2] >= 0 && gReport[2] <= (NUM_RGB_LEDS-1)){
					
					//Set the LED
					setLED(ledr_look_up[gReport[2]] ,gReport[3]);
					setLED(ledg_look_up[gReport[2]] ,gReport[4]);
					setLED(ledb_look_up[gReport[2]] ,gReport[5]);
					
					//write to EEPROM
					eeprom_update_byte(eeprom_ledr_look_up[gReport[2]],gReport[3]);
					eeprom_update_byte(eeprom_ledg_look_up[gReport[2]],gReport[4]);
					eeprom_update_byte(eeprom_ledb_look_up[gReport[2]],gReport[5]);
					controlSet = 1;
					
				}
				
				howler_hid_report[0] = HOWLER_ID; //eventually make howlerid
				howler_hid_report[1] = CMD_SET_RGB_LED_DEFAULT;
				howler_hid_report[2] = gReport[2];
				howler_hid_report[3] = gReport[3];
				howler_hid_report[4] = gReport[4];
				howler_hid_report[5] = gReport[5]; 
				howler_hid_report[6] = controlSet;//1 if set, 0xAF if not set
				howler_hid_report[7] = 0x00;
				udi_hid_generic_send_report_in(howler_hid_report);
				
				break;
			case CMD_SET_DEVICE_ID:
				//set USB Device ID (Power cycle of device required after set)
			
				break;
			case CMD_GET_ACCEL_DATA:
				//Get Accelerometer data and send back
				howler_hid_report[0] = HOWLER_ID;
				howler_hid_report[1] = CMD_GET_ACCEL_DATA;
				howler_hid_report[2] = MMA8453_Read_Output(1); //X data
				howler_hid_report[3] = MMA8453_Read_Output(2); //Y data
				howler_hid_report[4] = MMA8453_Read_Output(3); //Z data
				howler_hid_report[5] = 0x00;
				howler_hid_report[6] = 0x00;
				howler_hid_report[7] = 0x00;
				udi_hid_generic_send_report_in(howler_hid_report);
			
				break;
			case CMD_GET_QEC:
				qdec_dirY = qdec_get_direction(&gConfig1);
				positionY = qdec_get_position(&gConfig1);
				//positionY = qdec_get_frequency(&gConfig1);
				
				qdec_dirX = qdec_get_direction(&gConfig2);
				positionX = qdec_get_position(&gConfig2);
				//positionX = qdec_get_frequency(&gConfig2);
				
				howler_hid_report[0] = HOWLER_ID;
				howler_hid_report[1] = CMD_GET_QEC;
				howler_hid_report[2] = (uint8_t)qdec_dirX;
				howler_hid_report[3] = (uint8_t)((positionX>>8)&0xff); //MSB
				howler_hid_report[4] = (uint8_t)((positionX)&0xff); //LSB
				howler_hid_report[5] = (uint8_t)qdec_dirY;
				howler_hid_report[6] = (uint8_t)((positionY>>8)&0xff); //MSB
				howler_hid_report[7] = (uint8_t)((positionY)&0xff); //LSB
				udi_hid_generic_send_report_in(howler_hid_report);
				
				break;
			case CMD_GET_ADCS:
				qdec_dirY = qdec_get_direction(&gConfig1);
				positionY = qdec_get_position(&gConfig1);
				//positionY = qdec_get_frequency(&gConfig1);
			
				qdec_dirX = qdec_get_direction(&gConfig2);
				positionX = qdec_get_position(&gConfig2);
				//positionX = qdec_get_frequency(&gConfig2);
			
				howler_hid_report[0] = HOWLER_ID;
				howler_hid_report[1] = CMD_GET_ADCS;
				howler_hid_report[2] = gJoy1_u_ana_val;
				howler_hid_report[3] = gJoy1_d_ana_val; //MSB
				howler_hid_report[4] = 0;
				howler_hid_report[5] = 0;
				howler_hid_report[6] = 0;
				howler_hid_report[7] = 0;
				udi_hid_generic_send_report_in(howler_hid_report);
			
			break;	
				
			default:
				howler_hid_report[0] = HOWLER_ID;
				howler_hid_report[1] = gReport[1]; //bad command code received
				howler_hid_report[2] = 0xBA;
				howler_hid_report[3] = 0xD0;
				howler_hid_report[4] = 0xBA;
				howler_hid_report[5] = 0xD0;
				howler_hid_report[6] = 0xBA;
				howler_hid_report[7] = 0xD0;			
				udi_hid_generic_send_report_in(howler_hid_report);
				break;
		}
		for(int i = 0; i < UDI_HID_REPORT_OUT_SIZE; i++ ){
			gReport[i] = 0x00;
		}
	}	
}


void setLeds(void){
	setLED(JOY1_LEDR, eeprom_read_byte(JOY1_LEDR_ADDR));
	setLED(JOY1_LEDG, eeprom_read_byte(JOY1_LEDG_ADDR));
	setLED(JOY1_LEDB, eeprom_read_byte(JOY1_LEDB_ADDR));	
	setLED(JOY2_LEDR, eeprom_read_byte(JOY2_LEDR_ADDR));
	setLED(JOY2_LEDG, eeprom_read_byte(JOY2_LEDG_ADDR));
	setLED(JOY2_LEDB, eeprom_read_byte(JOY2_LEDB_ADDR));	
	setLED(JOY3_LEDR, eeprom_read_byte(JOY3_LEDR_ADDR));
	setLED(JOY3_LEDG, eeprom_read_byte(JOY3_LEDG_ADDR));
	setLED(JOY3_LEDB, eeprom_read_byte(JOY3_LEDB_ADDR));	
	setLED(JOY4_LEDR, eeprom_read_byte(JOY4_LEDR_ADDR));
	setLED(JOY4_LEDG, eeprom_read_byte(JOY4_LEDG_ADDR));
	setLED(JOY4_LEDB, eeprom_read_byte(JOY4_LEDB_ADDR));

	setLED(BUTT1_LEDR, eeprom_read_byte(BUTT1_LEDR_ADDR));
	setLED(BUTT1_LEDG, eeprom_read_byte(BUTT1_LEDG_ADDR));
	setLED(BUTT1_LEDB, eeprom_read_byte(BUTT1_LEDB_ADDR));
	
	setLED(BUTT2_LEDR, eeprom_read_byte(BUTT2_LEDR_ADDR));
	setLED(BUTT2_LEDG, eeprom_read_byte(BUTT2_LEDG_ADDR));
	setLED(BUTT2_LEDB, eeprom_read_byte(BUTT2_LEDB_ADDR));
	
	setLED(BUTT3_LEDR, eeprom_read_byte(BUTT3_LEDR_ADDR));
	setLED(BUTT3_LEDG, eeprom_read_byte(BUTT3_LEDG_ADDR));
	setLED(BUTT3_LEDB, eeprom_read_byte(BUTT3_LEDB_ADDR));
	
	setLED(BUTT4_LEDR, eeprom_read_byte(BUTT4_LEDR_ADDR));
	setLED(BUTT4_LEDG, eeprom_read_byte(BUTT4_LEDG_ADDR));
	setLED(BUTT4_LEDB, eeprom_read_byte(BUTT4_LEDB_ADDR));
	
	setLED(BUTT5_LEDR, eeprom_read_byte(BUTT5_LEDR_ADDR));
	setLED(BUTT5_LEDG, eeprom_read_byte(BUTT5_LEDG_ADDR));
	setLED(BUTT5_LEDB, eeprom_read_byte(BUTT5_LEDB_ADDR));
	
	setLED(BUTT6_LEDR, eeprom_read_byte(BUTT6_LEDR_ADDR));
	setLED(BUTT6_LEDG, eeprom_read_byte(BUTT6_LEDG_ADDR));
	setLED(BUTT6_LEDB, eeprom_read_byte(BUTT6_LEDB_ADDR));
	
	setLED(BUTT7_LEDR, eeprom_read_byte(BUTT7_LEDR_ADDR));
	setLED(BUTT7_LEDG, eeprom_read_byte(BUTT7_LEDG_ADDR));
	setLED(BUTT7_LEDB, eeprom_read_byte(BUTT7_LEDB_ADDR));
	
	setLED(BUTT8_LEDR, eeprom_read_byte(BUTT8_LEDR_ADDR));
	setLED(BUTT8_LEDG, eeprom_read_byte(BUTT8_LEDG_ADDR));
	setLED(BUTT8_LEDB, eeprom_read_byte(BUTT8_LEDB_ADDR));
	
	setLED(BUTT9_LEDR, eeprom_read_byte(BUTT9_LEDR_ADDR));
	setLED(BUTT9_LEDG, eeprom_read_byte(BUTT9_LEDG_ADDR));
	setLED(BUTT9_LEDB, eeprom_read_byte(BUTT9_LEDB_ADDR));
	
	setLED(BUTT10_LEDR, eeprom_read_byte(BUTT10_LEDR_ADDR));
	setLED(BUTT10_LEDG, eeprom_read_byte(BUTT10_LEDG_ADDR));
	setLED(BUTT10_LEDB, eeprom_read_byte(BUTT10_LEDB_ADDR));
	
	setLED(BUTT11_LEDR, eeprom_read_byte(BUTT11_LEDR_ADDR));
	setLED(BUTT11_LEDG, eeprom_read_byte(BUTT11_LEDG_ADDR));
	setLED(BUTT11_LEDB, eeprom_read_byte(BUTT11_LEDB_ADDR));
	
	setLED(BUTT12_LEDR, eeprom_read_byte(BUTT12_LEDR_ADDR));
	setLED(BUTT12_LEDG, eeprom_read_byte(BUTT12_LEDG_ADDR));
	setLED(BUTT12_LEDB, eeprom_read_byte(BUTT12_LEDB_ADDR));
	
	setLED(BUTT13_LEDR, eeprom_read_byte(BUTT13_LEDR_ADDR));
	setLED(BUTT13_LEDG, eeprom_read_byte(BUTT13_LEDG_ADDR));
	setLED(BUTT13_LEDB, eeprom_read_byte(BUTT13_LEDB_ADDR));
	
	setLED(BUTT14_LEDR, eeprom_read_byte(BUTT14_LEDR_ADDR));
	setLED(BUTT14_LEDG, eeprom_read_byte(BUTT14_LEDG_ADDR));
	setLED(BUTT14_LEDB, eeprom_read_byte(BUTT14_LEDB_ADDR));
	
	setLED(BUTT15_LEDR, eeprom_read_byte(BUTT15_LEDR_ADDR));
	setLED(BUTT15_LEDG, eeprom_read_byte(BUTT15_LEDG_ADDR));
	setLED(BUTT15_LEDB, eeprom_read_byte(BUTT15_LEDB_ADDR));
	
	setLED(BUTT16_LEDR, eeprom_read_byte(BUTT16_LEDR_ADDR));
	setLED(BUTT16_LEDG, eeprom_read_byte(BUTT16_LEDG_ADDR));
	setLED(BUTT16_LEDB, eeprom_read_byte(BUTT16_LEDB_ADDR));
	
	setLED(BUTT17_LEDR, eeprom_read_byte(BUTT17_LEDR_ADDR));
	setLED(BUTT17_LEDG, eeprom_read_byte(BUTT17_LEDG_ADDR));
	setLED(BUTT17_LEDB, eeprom_read_byte(BUTT17_LEDB_ADDR));
	
	setLED(BUTT18_LEDR, eeprom_read_byte(BUTT18_LEDR_ADDR));
	setLED(BUTT18_LEDG, eeprom_read_byte(BUTT18_LEDG_ADDR));
	setLED(BUTT18_LEDB, eeprom_read_byte(BUTT18_LEDB_ADDR));
	
	setLED(BUTT19_LEDR, eeprom_read_byte(BUTT19_LEDR_ADDR));
	setLED(BUTT19_LEDG, eeprom_read_byte(BUTT19_LEDG_ADDR));
	setLED(BUTT19_LEDB, eeprom_read_byte(BUTT19_LEDB_ADDR));
	
	setLED(BUTT20_LEDR, eeprom_read_byte(BUTT20_LEDR_ADDR));
	setLED(BUTT20_LEDG, eeprom_read_byte(BUTT20_LEDG_ADDR));
	setLED(BUTT20_LEDB, eeprom_read_byte(BUTT20_LEDB_ADDR));
	
	setLED(BUTT21_LEDR, eeprom_read_byte(BUTT21_LEDR_ADDR));
	setLED(BUTT21_LEDG, eeprom_read_byte(BUTT21_LEDG_ADDR));
	setLED(BUTT21_LEDB, eeprom_read_byte(BUTT21_LEDB_ADDR));
	
	setLED(BUTT22_LEDR, eeprom_read_byte(BUTT22_LEDR_ADDR));
	setLED(BUTT22_LEDG, eeprom_read_byte(BUTT22_LEDG_ADDR));
	setLED(BUTT22_LEDB, eeprom_read_byte(BUTT22_LEDB_ADDR));
	
	setLED(BUTT23_LEDR, eeprom_read_byte(BUTT23_LEDR_ADDR));
	setLED(BUTT23_LEDG, eeprom_read_byte(BUTT23_LEDG_ADDR));
	setLED(BUTT23_LEDB, eeprom_read_byte(BUTT23_LEDB_ADDR));
	
	setLED(BUTT24_LEDR, eeprom_read_byte(BUTT24_LEDR_ADDR));
	setLED(BUTT24_LEDG, eeprom_read_byte(BUTT24_LEDG_ADDR));
	setLED(BUTT24_LEDB, eeprom_read_byte(BUTT24_LEDB_ADDR));
	
	setLED(BUTT25_LEDR, eeprom_read_byte(BUTT25_LEDR_ADDR));
	setLED(BUTT25_LEDG, eeprom_read_byte(BUTT25_LEDG_ADDR));
	setLED(BUTT25_LEDB, eeprom_read_byte(BUTT25_LEDB_ADDR));
	
	setLED(BUTT26_LEDR, eeprom_read_byte(BUTT26_LEDR_ADDR));
	setLED(BUTT26_LEDG, eeprom_read_byte(BUTT26_LEDG_ADDR));
	setLED(BUTT26_LEDB, eeprom_read_byte(BUTT26_LEDB_ADDR));
	
	setLED(HP1_LEDR, eeprom_read_byte(HP1_LEDR_ADDR));
	setLED(HP1_LEDG, eeprom_read_byte(HP1_LEDG_ADDR));
	setLED(HP1_LEDB, eeprom_read_byte(HP1_LEDB_ADDR));
	
	setLED(HP2_LEDR, eeprom_read_byte(HP2_LEDR_ADDR));
	setLED(HP2_LEDG, eeprom_read_byte(HP2_LEDG_ADDR));
	setLED(HP2_LEDB, eeprom_read_byte(HP2_LEDB_ADDR));

	
}

void setLedDefaults(void){


	eeprom_write_byte(JOY1_LEDR_ADDR, JOY1_LEDR_DEFAULT);
	eeprom_write_byte(JOY1_LEDG_ADDR, JOY1_LEDG_DEFAULT);
	eeprom_write_byte(JOY1_LEDB_ADDR, JOY1_LEDB_DEFAULT);	
	eeprom_write_byte(JOY2_LEDR_ADDR, JOY2_LEDR_DEFAULT);
	eeprom_write_byte(JOY2_LEDG_ADDR, JOY2_LEDG_DEFAULT);
	eeprom_write_byte(JOY2_LEDB_ADDR, JOY2_LEDB_DEFAULT);	
	eeprom_write_byte(JOY3_LEDR_ADDR, JOY3_LEDR_DEFAULT);
	eeprom_write_byte(JOY3_LEDG_ADDR, JOY3_LEDG_DEFAULT);
	eeprom_write_byte(JOY3_LEDB_ADDR, JOY3_LEDB_DEFAULT);	
	eeprom_write_byte(JOY4_LEDR_ADDR, JOY4_LEDR_DEFAULT);
	eeprom_write_byte(JOY4_LEDG_ADDR, JOY4_LEDG_DEFAULT);
	eeprom_write_byte(JOY4_LEDB_ADDR, JOY4_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT1_LEDR_ADDR, BUTT1_LEDR_DEFAULT);
	eeprom_write_byte(BUTT1_LEDG_ADDR, BUTT1_LEDG_DEFAULT);
	eeprom_write_byte(BUTT1_LEDB_ADDR, BUTT1_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT2_LEDR_ADDR, BUTT2_LEDR_DEFAULT);
	eeprom_write_byte(BUTT2_LEDG_ADDR, BUTT2_LEDG_DEFAULT);
	eeprom_write_byte(BUTT2_LEDB_ADDR, BUTT2_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT3_LEDR_ADDR, BUTT3_LEDR_DEFAULT);
	eeprom_write_byte(BUTT3_LEDG_ADDR, BUTT3_LEDG_DEFAULT);
	eeprom_write_byte(BUTT3_LEDB_ADDR, BUTT3_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT4_LEDR_ADDR, BUTT4_LEDR_DEFAULT);
	eeprom_write_byte(BUTT4_LEDG_ADDR, BUTT4_LEDG_DEFAULT);
	eeprom_write_byte(BUTT4_LEDB_ADDR, BUTT4_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT5_LEDR_ADDR, BUTT5_LEDR_DEFAULT);
	eeprom_write_byte(BUTT5_LEDG_ADDR, BUTT5_LEDG_DEFAULT);
	eeprom_write_byte(BUTT5_LEDB_ADDR, BUTT5_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT6_LEDR_ADDR, BUTT6_LEDR_DEFAULT);
	eeprom_write_byte(BUTT6_LEDG_ADDR, BUTT6_LEDG_DEFAULT);
	eeprom_write_byte(BUTT6_LEDB_ADDR, BUTT6_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT7_LEDR_ADDR, BUTT7_LEDR_DEFAULT);
	eeprom_write_byte(BUTT7_LEDG_ADDR, BUTT7_LEDG_DEFAULT);
	eeprom_write_byte(BUTT7_LEDB_ADDR, BUTT7_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT8_LEDR_ADDR, BUTT8_LEDR_DEFAULT);
	eeprom_write_byte(BUTT8_LEDG_ADDR, BUTT8_LEDG_DEFAULT);
	eeprom_write_byte(BUTT8_LEDB_ADDR, BUTT8_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT9_LEDR_ADDR, BUTT9_LEDR_DEFAULT);
	eeprom_write_byte(BUTT9_LEDG_ADDR, BUTT9_LEDG_DEFAULT);
	eeprom_write_byte(BUTT9_LEDB_ADDR, BUTT9_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT10_LEDR_ADDR, BUTT10_LEDR_DEFAULT);
	eeprom_write_byte(BUTT10_LEDG_ADDR, BUTT10_LEDG_DEFAULT);
	eeprom_write_byte(BUTT10_LEDB_ADDR, BUTT10_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT11_LEDR_ADDR, BUTT11_LEDR_DEFAULT);
	eeprom_write_byte(BUTT11_LEDG_ADDR, BUTT11_LEDG_DEFAULT);
	eeprom_write_byte(BUTT11_LEDB_ADDR, BUTT11_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT12_LEDR_ADDR, BUTT12_LEDR_DEFAULT);
	eeprom_write_byte(BUTT12_LEDG_ADDR, BUTT12_LEDG_DEFAULT);
	eeprom_write_byte(BUTT12_LEDB_ADDR, BUTT12_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT13_LEDR_ADDR, BUTT13_LEDR_DEFAULT);
	eeprom_write_byte(BUTT13_LEDG_ADDR, BUTT13_LEDG_DEFAULT);
	eeprom_write_byte(BUTT13_LEDB_ADDR, BUTT13_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT14_LEDR_ADDR, BUTT14_LEDR_DEFAULT);
	eeprom_write_byte(BUTT14_LEDG_ADDR, BUTT14_LEDG_DEFAULT);
	eeprom_write_byte(BUTT14_LEDB_ADDR, BUTT14_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT15_LEDR_ADDR, BUTT15_LEDR_DEFAULT);
	eeprom_write_byte(BUTT15_LEDG_ADDR, BUTT15_LEDG_DEFAULT);
	eeprom_write_byte(BUTT15_LEDB_ADDR, BUTT15_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT16_LEDR_ADDR, BUTT16_LEDR_DEFAULT);
	eeprom_write_byte(BUTT16_LEDG_ADDR, BUTT16_LEDG_DEFAULT);
	eeprom_write_byte(BUTT16_LEDB_ADDR, BUTT16_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT17_LEDR_ADDR, BUTT17_LEDR_DEFAULT);
	eeprom_write_byte(BUTT17_LEDG_ADDR, BUTT17_LEDG_DEFAULT);
	eeprom_write_byte(BUTT17_LEDB_ADDR, BUTT17_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT18_LEDR_ADDR, BUTT18_LEDR_DEFAULT);
	eeprom_write_byte(BUTT18_LEDG_ADDR, BUTT18_LEDG_DEFAULT);
	eeprom_write_byte(BUTT18_LEDB_ADDR, BUTT18_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT19_LEDR_ADDR, BUTT19_LEDR_DEFAULT);
	eeprom_write_byte(BUTT19_LEDG_ADDR, BUTT19_LEDG_DEFAULT);
	eeprom_write_byte(BUTT19_LEDB_ADDR, BUTT19_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT20_LEDR_ADDR, BUTT20_LEDR_DEFAULT);
	eeprom_write_byte(BUTT20_LEDG_ADDR, BUTT20_LEDG_DEFAULT);
	eeprom_write_byte(BUTT20_LEDB_ADDR, BUTT20_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT21_LEDR_ADDR, BUTT21_LEDR_DEFAULT);
	eeprom_write_byte(BUTT21_LEDG_ADDR, BUTT21_LEDG_DEFAULT);
	eeprom_write_byte(BUTT21_LEDB_ADDR, BUTT21_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT22_LEDR_ADDR, BUTT22_LEDR_DEFAULT);
	eeprom_write_byte(BUTT22_LEDG_ADDR, BUTT22_LEDG_DEFAULT);
	eeprom_write_byte(BUTT22_LEDB_ADDR, BUTT22_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT23_LEDR_ADDR, BUTT23_LEDR_DEFAULT);
	eeprom_write_byte(BUTT23_LEDG_ADDR, BUTT23_LEDG_DEFAULT);
	eeprom_write_byte(BUTT23_LEDB_ADDR, BUTT23_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT24_LEDR_ADDR, BUTT24_LEDR_DEFAULT);
	eeprom_write_byte(BUTT24_LEDG_ADDR, BUTT24_LEDG_DEFAULT);
	eeprom_write_byte(BUTT24_LEDB_ADDR, BUTT24_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT25_LEDR_ADDR, BUTT25_LEDR_DEFAULT);
	eeprom_write_byte(BUTT25_LEDG_ADDR, BUTT25_LEDG_DEFAULT);
	eeprom_write_byte(BUTT25_LEDB_ADDR, BUTT25_LEDB_DEFAULT);	
	eeprom_write_byte(BUTT26_LEDR_ADDR, BUTT26_LEDR_DEFAULT);
	eeprom_write_byte(BUTT26_LEDG_ADDR, BUTT26_LEDG_DEFAULT);
	eeprom_write_byte(BUTT26_LEDB_ADDR, BUTT26_LEDB_DEFAULT);	
	eeprom_write_byte(HP1_LEDR_ADDR, HP1_LEDR_DEFAULT);
	eeprom_write_byte(HP1_LEDG_ADDR, HP1_LEDG_DEFAULT);
	eeprom_write_byte(HP1_LEDB_ADDR, HP1_LEDB_DEFAULT);		
	eeprom_write_byte(HP2_LEDR_ADDR, HP2_LEDR_DEFAULT);
	eeprom_write_byte(HP2_LEDG_ADDR, HP2_LEDG_DEFAULT);
	eeprom_write_byte(HP2_LEDB_ADDR, HP2_LEDB_DEFAULT);

}
void setInputDefaults(void){
		
	//set input type defaults
	eeprom_write_byte(eeprom_input_type_look_up[JOY1_U_LKUP_INDEX], JOY1_U_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY1_D_LKUP_INDEX], JOY1_D_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY1_L_LKUP_INDEX], JOY1_L_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY1_R_LKUP_INDEX], JOY1_R_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY2_U_LKUP_INDEX], JOY2_U_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY2_D_LKUP_INDEX], JOY2_D_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY2_L_LKUP_INDEX], JOY2_L_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY2_R_LKUP_INDEX], JOY2_R_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY3_U_LKUP_INDEX], JOY3_U_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY3_D_LKUP_INDEX], JOY3_D_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY3_L_LKUP_INDEX], JOY3_L_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY3_R_LKUP_INDEX], JOY3_R_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY4_U_LKUP_INDEX], JOY4_U_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY4_D_LKUP_INDEX], JOY4_D_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY4_L_LKUP_INDEX], JOY4_L_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[JOY4_R_LKUP_INDEX], JOY4_R_TYPE_DEFAULT);
		
	eeprom_write_byte(eeprom_input_type_look_up[BUTT1_LKUP_INDEX], BUTT1_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT2_LKUP_INDEX], BUTT2_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT3_LKUP_INDEX], BUTT3_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT4_LKUP_INDEX], BUTT4_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT5_LKUP_INDEX], BUTT5_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT6_LKUP_INDEX], BUTT6_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT7_LKUP_INDEX], BUTT7_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT8_LKUP_INDEX], BUTT8_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT9_LKUP_INDEX], BUTT9_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT10_LKUP_INDEX], BUTT10_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT11_LKUP_INDEX], BUTT11_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT12_LKUP_INDEX], BUTT12_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT13_LKUP_INDEX], BUTT13_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT14_LKUP_INDEX], BUTT14_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT15_LKUP_INDEX], BUTT15_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT16_LKUP_INDEX], BUTT16_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT17_LKUP_INDEX], BUTT17_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT18_LKUP_INDEX], BUTT18_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT19_LKUP_INDEX], BUTT19_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT20_LKUP_INDEX], BUTT20_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT21_LKUP_INDEX], BUTT21_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT22_LKUP_INDEX], BUTT22_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT23_LKUP_INDEX], BUTT23_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT24_LKUP_INDEX], BUTT24_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT25_LKUP_INDEX], BUTT25_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[BUTT26_LKUP_INDEX], BUTT26_TYPE_DEFAULT);
	
	eeprom_write_byte(eeprom_input_type_look_up[ACCEL_XAXIS_LKUP_INDEX], ACCEL_XAXIS_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[ACCEL_YAXIS_LKUP_INDEX], ACCEL_YAXIS_TYPE_DEFAULT);
	eeprom_write_byte(eeprom_input_type_look_up[ACCEL_ZAXIS_LKUP_INDEX], ACCEL_ZAXIS_TYPE_DEFAULT);
		
	//set input value defaults
	eeprom_write_byte(eeprom_input_value_look_up[JOY1_U_LKUP_INDEX], JOY1_U_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY1_D_LKUP_INDEX], JOY1_D_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY1_L_LKUP_INDEX], JOY1_L_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY1_R_LKUP_INDEX], JOY1_R_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY2_U_LKUP_INDEX], JOY2_U_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY2_D_LKUP_INDEX], JOY2_D_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY2_L_LKUP_INDEX], JOY2_L_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY2_R_LKUP_INDEX], JOY2_R_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY3_U_LKUP_INDEX], JOY3_U_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY3_D_LKUP_INDEX], JOY3_D_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY3_L_LKUP_INDEX], JOY3_L_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY3_R_LKUP_INDEX], JOY3_R_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY4_U_LKUP_INDEX], JOY4_U_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY4_D_LKUP_INDEX], JOY4_D_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY4_L_LKUP_INDEX], JOY4_L_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[JOY4_R_LKUP_INDEX], JOY4_R_VALUE_DEFAULT);
		
	eeprom_write_byte(eeprom_input_value_look_up[BUTT1_LKUP_INDEX], BUTT1_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT2_LKUP_INDEX], BUTT2_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT3_LKUP_INDEX], BUTT3_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT4_LKUP_INDEX], BUTT4_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT5_LKUP_INDEX], BUTT5_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT6_LKUP_INDEX], BUTT6_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT7_LKUP_INDEX], BUTT7_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT8_LKUP_INDEX], BUTT8_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT9_LKUP_INDEX], BUTT9_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT10_LKUP_INDEX], BUTT10_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT11_LKUP_INDEX], BUTT11_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT12_LKUP_INDEX], BUTT12_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT13_LKUP_INDEX], BUTT13_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT14_LKUP_INDEX], BUTT14_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT15_LKUP_INDEX], BUTT15_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT16_LKUP_INDEX], BUTT16_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT17_LKUP_INDEX], BUTT17_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT18_LKUP_INDEX], BUTT18_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT19_LKUP_INDEX], BUTT19_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT20_LKUP_INDEX], BUTT20_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT21_LKUP_INDEX], BUTT21_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT22_LKUP_INDEX], BUTT22_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT23_LKUP_INDEX], BUTT23_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT24_LKUP_INDEX], BUTT24_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT25_LKUP_INDEX], BUTT25_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[BUTT26_LKUP_INDEX], BUTT26_VALUE_DEFAULT);
	
	eeprom_write_byte(eeprom_input_value_look_up[ACCEL_XAXIS_LKUP_INDEX], ACCEL_XAXIS_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[ACCEL_YAXIS_LKUP_INDEX], ACCEL_YAXIS_VALUE_DEFAULT);
	eeprom_write_byte(eeprom_input_value_look_up[ACCEL_ZAXIS_LKUP_INDEX], ACCEL_ZAXIS_VALUE_DEFAULT);
	
	//set input value2
	eeprom_write_byte(eeprom_input_value2_look_up[JOY1_U_LKUP_INDEX], JOY1_U_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY1_D_LKUP_INDEX], JOY1_D_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY1_L_LKUP_INDEX], JOY1_L_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY1_R_LKUP_INDEX], JOY1_R_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY2_U_LKUP_INDEX], JOY2_U_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY2_D_LKUP_INDEX], JOY2_D_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY2_L_LKUP_INDEX], JOY2_L_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY2_R_LKUP_INDEX], JOY2_R_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY3_U_LKUP_INDEX], JOY3_U_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY3_D_LKUP_INDEX], JOY3_D_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY3_L_LKUP_INDEX], JOY3_L_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY3_R_LKUP_INDEX], JOY3_R_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY4_U_LKUP_INDEX], JOY4_U_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY4_D_LKUP_INDEX], JOY4_D_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY4_L_LKUP_INDEX], JOY4_L_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[JOY4_R_LKUP_INDEX], JOY4_R_VALUE2_DEFAULT);
	
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT1_LKUP_INDEX], BUTT1_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT2_LKUP_INDEX], BUTT2_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT3_LKUP_INDEX], BUTT3_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT4_LKUP_INDEX], BUTT4_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT5_LKUP_INDEX], BUTT5_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT6_LKUP_INDEX], BUTT6_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT7_LKUP_INDEX], BUTT7_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT8_LKUP_INDEX], BUTT8_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT9_LKUP_INDEX], BUTT9_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT10_LKUP_INDEX], BUTT10_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT11_LKUP_INDEX], BUTT11_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT12_LKUP_INDEX], BUTT12_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT13_LKUP_INDEX], BUTT13_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT14_LKUP_INDEX], BUTT14_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT15_LKUP_INDEX], BUTT15_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT16_LKUP_INDEX], BUTT16_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT17_LKUP_INDEX], BUTT17_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT18_LKUP_INDEX], BUTT18_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT19_LKUP_INDEX], BUTT19_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT20_LKUP_INDEX], BUTT20_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT21_LKUP_INDEX], BUTT21_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT22_LKUP_INDEX], BUTT22_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT23_LKUP_INDEX], BUTT23_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT24_LKUP_INDEX], BUTT24_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT25_LKUP_INDEX], BUTT25_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[BUTT26_LKUP_INDEX], BUTT26_VALUE2_DEFAULT);
	
	eeprom_write_byte(eeprom_input_value2_look_up[ACCEL_XAXIS_LKUP_INDEX], ACCEL_XAXIS_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[ACCEL_YAXIS_LKUP_INDEX], ACCEL_YAXIS_VALUE2_DEFAULT);
	eeprom_write_byte(eeprom_input_value2_look_up[ACCEL_ZAXIS_LKUP_INDEX], ACCEL_ZAXIS_VALUE2_DEFAULT);
	
	eeprom_write_byte(ACCEL_XAXIS_MIN_TRIG_ADDR, ACCEL_XAXIS_MIN_TRIG_DEFAULT);
	eeprom_write_byte(ACCEL_YAXIS_MIN_TRIG_ADDR, ACCEL_YAXIS_MIN_TRIG_DEFAULT);
	eeprom_write_byte(ACCEL_ZAXIS_MIN_TRIG_ADDR, ACCEL_ZAXIS_MIN_TRIG_DEFAULT);
	eeprom_write_byte(ACCEL_XAXIS_MAX_TRIG_ADDR, ACCEL_XAXIS_MAX_TRIG_DEFAULT);
	eeprom_write_byte(ACCEL_YAXIS_MAX_TRIG_ADDR, ACCEL_YAXIS_MAX_TRIG_DEFAULT);
	eeprom_write_byte(ACCEL_ZAXIS_MAX_TRIG_ADDR, ACCEL_ZAXIS_MAX_TRIG_DEFAULT);
	
	
}

//Generic HID callbacks 
void main_hid_set_feature(uint8_t* report){
	if (report[0] == 0xAA && report[1] == 0x55
	&& report[2] == 0xAA && report[3] == 0x55) {
		// Disconnect USB Device
		udc_stop();
	}

}

void usb_generic_hid_out(uint8_t* report){
	//get report to global buffer
	
	int cnt  = 0;
	
	for(cnt = 0; cnt < UDI_HID_REPORT_OUT_SIZE; cnt++){
		gReport[cnt] = report[cnt];
	}

	gHidData = 1;	
}
