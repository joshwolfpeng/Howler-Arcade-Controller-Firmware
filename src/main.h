/*
 * IncFile1.h
 *
 * Created: 1/28/2013 9:31:13 PM
 *  Author: arcade
 */ 


#ifndef MAIN_H_
#define MAIN_H_

static void qdec1_init(void);
static void qdec2_init(void);
static void qdec3_init(void);
static void adc_init(void);
void setInputDefaults(void);
void setLedDefaults(void);
void setLeds(void);
void checkJoystickDown(uint8_t index, uint64_t mask);
void checkJoystickUp(uint8_t index, uint64_t mask);
void checkButtonDown(uint8_t index, uint64_t mask);
void checkButtonUp(uint8_t index, uint64_t mask);
void joyButtonPressed(uint8_t button, uint8_t joystick);
void parseHidReport(void);
void main_hid_set_feature(uint8_t* report);
void usb_generic_hid_out(uint8_t* report);

#endif /* MAIN_H_ */