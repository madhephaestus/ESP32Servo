/*
 * ESP32PWM.h
 *
 *  Created on: Sep 22, 2018
 *      Author: hephaestus
 */

#ifndef LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#define LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#include "esp32-hal-ledc.h"
#define NUM_PWM 16
#define PWM_BASE_INDEX 1
#define USABLE_ESP32_PWM (NUM_PWM-PWM_BASE_INDEX)
#include <cstdint>

#include "Arduino.h"
class ESP32PWM {
private:
	void detach();
	void attach(int pin);
	int pwmChannel = 0;                         // channel number for this servo
	bool attachedState= false;
	int pin;
public:

	ESP32PWM();
	virtual ~ESP32PWM();

	//channel 0-15 resolution 1-16bits freq limits depend on resolution9
	double      setup( double freq, uint8_t resolution_bits);
	void        write( uint32_t duty);
	double      writeTone( double freq);
	double      writeNote( note_t note, uint8_t octave);
	uint32_t    read();
	double      readFreq();
	void        attachPin(uint8_t pin);
	void        detachPin(uint8_t pin);
	int getPin(){
		return pin;
	}
	int getChannel();
	bool attached(){
		return attachedState;
	}

};
ESP32PWM* pwmFactory(int pin);

#endif /* LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_ */