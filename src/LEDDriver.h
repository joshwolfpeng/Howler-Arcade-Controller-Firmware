// LEDDriver.h


//TWI Definitions
#define LED_SLAVE1_ADDR   0x00 
#define LED_SLAVE2_ADDR   0x01
#define LED_SLAVE3_ADDR   0x02
#define LED_SLAVE4_ADDR   0x07
#define LED_SLAVE5_ADDR   0x04
#define LED_SLAVE6_ADDR   0x05
#define LED_ALLCALL_ADDR  0xE0

#define LED_AUTO_INC_NONE			0x00
#define LED_AUTO_INC_ALL			0x80
#define LED_AUTO_INC_IND_BRT		0xA0
#define LED_AUTO_INC_GLOBAL_ONLY	0xC0
#define LED_AUTO_INC_IND_AND_GLOBAL	0xE0

//Register Definitions
#define LED_REG_MODE1		0x00
#define LED_REG_MODE2		0x01
#define LED_REG_PWM0		0x02
#define LED_REG_PWM1		0x03
#define LED_REG_PWM2		0x04
#define LED_REG_PWM3		0x05
#define LED_REG_PWM4		0x06
#define LED_REG_PWM5		0x07
#define LED_REG_PWM6		0x08
#define LED_REG_PWM7		0x09
#define LED_REG_PWM8		0x0A
#define LED_REG_PWM9		0x0B
#define LED_REG_PWM10		0x0C
#define LED_REG_PWM11		0x0D
#define LED_REG_PWM12		0x0E
#define LED_REG_PWM13		0x0F
#define LED_REG_PWM14		0x10
#define LED_REG_PWM15		0x11
#define LED_REG_GRPPWM		0x12
#define LED_REG_GRPFREQ		0x13
#define LED_REG_LEDOUT0		0x14
#define LED_REG_LEDOUT1		0x15
#define LED_REG_LEDOUT2		0x16
#define LED_REG_LEDOUT3		0x17
#define LED_REG_SUBADR1		0x18
#define LED_REG_SUBADR2		0x19
#define LED_REG_SUBADR3		0x1A
#define LED_REG_ALLCALLADR	0x1B

#define JOY1_LEDR		0x0002	//Slave Address 0, LED0
#define JOY1_LEDG		0x0202	//Slave Address 2, LED0
#define JOY1_LEDB		0x0402	//Slave Address 4, LED0
#define JOY2_LEDR		0x0011	//Slave Address 0, LED15
#define JOY2_LEDG		0x0211	//Slave Address 2, LED15
#define JOY2_LEDB		0x0411	//Slave Address 4, LED15
#define JOY3_LEDR		0x0108	//Slave Address 1, LED6
#define JOY3_LEDG		0x0708	//Slave Address 7, LED6
#define JOY3_LEDB		0x0508	//Slave Address 5, LED6
#define JOY4_LEDR		0x010B	//Slave Address 1, LED9
#define JOY4_LEDG		0x070B	//Slave Address 7, LED9
#define JOY4_LEDB		0x050B	//Slave Address 5, LED9
#define BUTT1_LEDR		0x0003	//Slave Address 0, LED1
#define BUTT1_LEDG		0x0203	//Slave Address 2, LED1
#define BUTT1_LEDB		0x0403	//Slave Address 4, LED1
#define BUTT2_LEDR		0x0004	//Slave Address 0, LED2
#define BUTT2_LEDG		0x0204	//Slave Address 2, LED2
#define BUTT2_LEDB		0x0404	//Slave Address 4, LED2
#define BUTT3_LEDR		0x0005	//Slave Address 0, LED3
#define BUTT3_LEDG		0x0205	//Slave Address 2, LED3
#define BUTT3_LEDB		0x0405	//Slave Address 4, LED3
#define BUTT4_LEDR		0x0006	//Slave Address 0, LED4
#define BUTT4_LEDG		0x0206	//Slave Address 2, LED4
#define BUTT4_LEDB		0x0406	//Slave Address 4, LED4
#define BUTT5_LEDR		0x0007	//Slave Address 0, LED5
#define BUTT5_LEDG		0x0207	//Slave Address 2, LED5
#define BUTT5_LEDB		0x0407	//Slave Address 4, LED5
#define BUTT6_LEDR		0x0008	//Slave Address 0, LED6
#define BUTT6_LEDG		0x0208	//Slave Address 2, LED6
#define BUTT6_LEDB		0x0408	//Slave Address 4, LED6
#define BUTT7_LEDR		0x0009	//Slave Address 0, LED7
#define BUTT7_LEDG		0x0209	//Slave Address 2, LED7
#define BUTT7_LEDB		0x0409	//Slave Address 4, LED7
#define BUTT8_LEDR		0x0102	//Slave Address 1, LED0
#define BUTT8_LEDG		0x0702	//Slave Address 7, LED0
#define BUTT8_LEDB		0x0502	//Slave Address 5, LED0
#define BUTT9_LEDR		0x0103	//Slave Address 1, LED1
#define BUTT9_LEDG		0x0703	//Slave Address 7, LED1
#define BUTT9_LEDB		0x0503	//Slave Address 5, LED1
#define BUTT10_LEDR		0x0104	//Slave Address 1, LED2
#define BUTT10_LEDG		0x0704	//Slave Address 7, LED2
#define BUTT10_LEDB		0x0504	//Slave Address 5, LED2
#define BUTT11_LEDR		0x0105	//Slave Address 1, LED3
#define BUTT11_LEDG		0x0705	//Slave Address 7, LED3
#define BUTT11_LEDB		0x0505	//Slave Address 5, LED3
#define BUTT12_LEDR		0x0106	//Slave Address 1, LED4
#define BUTT12_LEDG		0x0706	//Slave Address 7, LED4
#define BUTT12_LEDB		0x0506	//Slave Address 5, LED4
#define BUTT13_LEDR		0x0107	//Slave Address 1, LED5
#define BUTT13_LEDG		0x0707	//Slave Address 7, LED5
#define BUTT13_LEDB		0x0507	//Slave Address 5, LED5
#define BUTT14_LEDR		0x0010	//Slave Address 0, LED14
#define BUTT14_LEDG		0x0210	//Slave Address 2, LED14
#define BUTT14_LEDB		0x0410	//Slave Address 4, LED14
#define BUTT15_LEDR		0x000F	//Slave Address 0, LED13
#define BUTT15_LEDG		0x020F	//Slave Address 2, LED13
#define BUTT15_LEDB		0x040F	//Slave Address 4, LED13
#define BUTT16_LEDR		0x000E	//Slave Address 0, LED12
#define BUTT16_LEDG		0x020E	//Slave Address 2, LED12
#define BUTT16_LEDB		0x040E	//Slave Address 4, LED12
#define BUTT17_LEDR		0x000D	//Slave Address 0, LED11
#define BUTT17_LEDG		0x020D	//Slave Address 2, LED11
#define BUTT17_LEDB		0x040D	//Slave Address 4, LED11
#define BUTT18_LEDR		0x000C	//Slave Address 0, LED10
#define BUTT18_LEDG		0x020C	//Slave Address 2, LED10
#define BUTT18_LEDB		0x040C	//Slave Address 4, LED10
#define BUTT19_LEDR		0x000B	//Slave Address 0, LED9
#define BUTT19_LEDG		0x020B	//Slave Address 2, LED9
#define BUTT19_LEDB		0x040B	//Slave Address 4, LED9
#define BUTT20_LEDR		0x000A	//Slave Address 0, LED8
#define BUTT20_LEDG		0x020A	//Slave Address 2, LED8
#define BUTT20_LEDB		0x040A	//Slave Address 4, LED8
#define BUTT21_LEDR		0x0111	//Slave Address 1, LED15
#define BUTT21_LEDG		0x0711	//Slave Address 7, LED15
#define BUTT21_LEDB		0x0511	//Slave Address 5, LED15
#define BUTT22_LEDR		0x0110	//Slave Address 1, LED14
#define BUTT22_LEDG		0x0710	//Slave Address 7, LED14
#define BUTT22_LEDB		0x0510	//Slave Address 5, LED14
#define BUTT23_LEDR		0x010F	//Slave Address 1, LED13
#define BUTT23_LEDG		0x070F	//Slave Address 7, LED13
#define BUTT23_LEDB		0x050F	//Slave Address 5, LED13
#define BUTT24_LEDR		0x010E	//Slave Address 1, LED12
#define BUTT24_LEDG		0x070E	//Slave Address 7, LED12
#define BUTT24_LEDB		0x050E	//Slave Address 5, LED12
#define BUTT25_LEDR		0x010D	//Slave Address 1, LED11
#define BUTT25_LEDG		0x070D	//Slave Address 7, LED11
#define BUTT25_LEDB		0x050D	//Slave Address 5, LED11
#define BUTT26_LEDR		0x010C	//Slave Address 1, LED10
#define BUTT26_LEDG		0x070C	//Slave Address 7, LED10
#define BUTT26_LEDB		0x050C	//Slave Address 5, LED10
#define HP1_LEDR		0x0109	//Slave Address 1, LED7
#define HP1_LEDG		0x0709	//Slave Address 7, LED7
#define HP1_LEDB		0x0509	//Slave Address 5, LED7
#define HP2_LEDR		0x010A	//Slave Address 1, LED8
#define HP2_LEDG		0x070A	//Slave Address 7, LED8
#define HP2_LEDB		0x050A	//Slave Address 5, LED8

//RGB LED definitions
#define JOY1_LED		0x0000000202020402
#define JOY2_LED		0x0000001102110411
#define JOY3_LED		0x0000010807080508
#define JOY4_LED		0x0000010B070B050B
#define BUTT1_LED		0x0000000302030403
#define BUTT2_LED		0x0000000402040404
#define BUTT3_LED		0x0000000502050405
#define BUTT4_LED		0x0000000602060406
#define BUTT5_LED		0x0000000702070407
#define BUTT6_LED		0x0000000802080408
#define BUTT7_LED		0x0000000902090409
#define BUTT8_LED		0x0000010207020502
#define BUTT9_LED		0x0000010307030503
#define BUTT10_LED		0x0000010407040504
#define BUTT11_LED		0x0000010507050505
#define BUTT12_LED		0x0000010607060506
#define BUTT13_LED		0x0000010707070507
#define BUTT14_LED		0x0000001002100410
#define BUTT15_LED		0x0000000F020F040F
#define BUTT16_LED		0x0000000E020E040E
#define BUTT17_LED		0x0000000D020D040D
#define BUTT18_LED		0x0000000C020C040C
#define BUTT19_LED		0x0000000B020B040B
#define BUTT20_LED		0x0000000A020A040A
#define BUTT21_LED		0x0000011107110511
#define BUTT22_LED		0x0000011007100510
#define BUTT23_LED		0x0000010F070F050F
#define BUTT24_LED		0x0000010E070E050E
#define BUTT25_LED		0x0000010D070D050D
#define BUTT26_LED		0x0000010C070C050C
#define HP1_LED			0x0000010907090509
#define HP2_LED			0x0000010A070A050A

//Color definitions
#define COLOR_RED		0x00FF0000
#define COLOR_GREEN		0x0000FF00
#define COLOR_BLUE		0x000000FF
#define COLOR_BLACK		0x00000000
#define COLOR_WHITE		0x00FFFFFF
#define COLOR_YELLOW	0x00FFFF00
#define COLOR_PURPLE	0x00FF00FF
#define COLOR_CYAN		0x0000FFFF
#define COLOR_ORANGE	0x00FFA500
#define COLOR_DK_ORANGE	0x00FF5000

//function definitions
void LED_Drivers_Init(void);
void setLEDglobalbrightness(uint8_t globalBrightness);
void setLED(uint16_t ledToSet, uint8_t pwmLevel);
void setLEDRGB(uint64_t ledToSet, uint32_t rgbLevel);
void getLED(uint16_t ledToGet, uint8_t *pwmLevel);
void setLEDBank(uint16_t bankToSet, uint8_t bankValues[]);