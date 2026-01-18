/*
 * ESP32PWM.h - Enhanced PWM Library for ESP32
 *
 * This library provides PWM functionality for ESP32 chips with support for:
 * - ESP32S3: MCPWM hardware acceleration for optimal servo performance
 * - Variable frequency mode: LEDC preferred, MCPWM fallback for flexibility
 * - Fixed frequency mode: MCPWM preferred, LEDC fallback for shared timers
 * - Automatic hardware allocation with intelligent fallbacks
 *
 * Key Features:
 * - Dual hardware support (LEDC + MCPWM on S3)
 * - 20 total PWM channels on ESP32S3 (8 LEDC + 12 MCPWM)
 * - Frequency locking for fixed-frequency applications (servos)
 * - Seamless hardware fallback when preferred hardware unavailable
 *
 * Usage:
 * - ESP32PWM pwm;                    // Variable frequency (default)
 * - ESP32PWM pwm(true);              // Variable frequency (explicit)
 * - ESP32PWM pwm(false);             // Fixed frequency
 *
 * Created on: Sep 22, 2018
 * Author: hephaestus
 * Enhanced for ESP32S3 MCPWM support
 */

#ifndef LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#define LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#include "esp32-hal-ledc.h"
#if defined(ARDUINO)
	#include "Arduino.h"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define NUM_PWM 6
#elif defined(CONFIG_IDF_TARGET_ESP32S2)   ||  defined(CONFIG_IDF_TARGET_ESP32S3)
#define NUM_PWM 8
#else
#define NUM_PWM 16
#endif

// MCPWM support for ESP32S3
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "driver/mcpwm.h"
#define MCPWM_NUM_UNITS 2
#define MCPWM_NUM_TIMERS_PER_UNIT 3
#define MCPWM_NUM_OPERATORS_PER_TIMER 2

struct MCPWMTimerInfo {
    bool initialized = false;
    long freq = -1;
    int operatorCount = 0;
    ESP32PWM* operators[MCPWM_NUM_OPERATORS_PER_TIMER] = {nullptr, nullptr};
};

#endif

#define PWM_BASE_INDEX 0
#define USABLE_ESP32_PWM (NUM_PWM-PWM_BASE_INDEX)
#include <cstdint>

class ESP32PWM {
private:

	void attach(int pin);
	int pwmChannel = 0;                         // channel number for this servo
	bool attachedState = false;
	int pin;
	uint8_t resolutionBits;
	double myFreq;
	bool useVariableFrequency = false;
	bool isMCPWM = false;
	int allocatenext(double freq);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
	mcpwm_unit_t mcpwmUnit;
	mcpwm_timer_t mcpwmTimer;
	mcpwm_operator_t mcpwmOperator;
#endif

	static double _ledcSetupTimerFreq(uint8_t pin, double freq,
		uint8_t bit_num, uint8_t channel);

	bool checkFrequencyForSideEffects(double freq);

	void adjustFrequencyLocal(double freq, double dutyScaled);
	static double mapf(double x, double in_min, double in_max, double out_min,
			double out_max) {
		if(x>in_max)
			return out_max;
		if(x<in_min)
			return out_min;
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	double setup(double freq, uint8_t resolution_bits=10);
	//channel 0-15 resolution 1-16bits freq limits depend on resolution9
	void attachPin(uint8_t pin);
	// pin allocation
	void deallocate();
public:
	// setup
	ESP32PWM(bool variableFrequency = true);  // true=variable freq (LEDC preferred), false=fixed freq (MCPWM preferred)
	virtual ~ESP32PWM();


	void detachPin(int pin);
	void attachPin(uint8_t pin, double freq, uint8_t resolution_bits=10);
	bool attached() {
		return attachedState;
	}

	// write raw duty cycle
	void write(uint32_t duty);
	// Write a duty cycle to the PWM using a unit vector from 0.0-1.0
	void writeScaled(double duty);
	//Adjust frequency (only works on variable-frequency channels)
	double writeTone(double freq);
	double writeNote(note_t note, uint8_t octave);
	void adjustFrequency(double freq, double dutyScaled=-1);

	// Read pwm data
	uint32_t read();
	double readFreq();
	double getDutyScaled();

	//Timer data
	static int timerAndIndexToChannel(int timer, int index);
	/**
	 * allocateTimer
	 * @param a timer number 0-3 indicating which timer to allocate in this library
	 * Switch to explicate allocation mode
	 *
	 */
	static void allocateTimer(int timerNumber);
	static bool explicateAllocationMode;
	int getTimer() {
		return timerNum;
	}
	int timerNum = -1;
	uint32_t myDuty = 0;
	int getChannel();
	static int PWMCount;              // the total number of attached pwm
	static int timerCount[4];
	static ESP32PWM * ChannelUsed[NUM_PWM]; // used to track whether a channel is in service
	static long timerFreqSet[4];

#if defined(CONFIG_IDF_TARGET_ESP32S3)
	static MCPWMTimerInfo mcpwmTimers[MCPWM_NUM_UNITS][MCPWM_NUM_TIMERS_PER_UNIT];
#endif

	// Helper functions
	int getPin() {
		return pin;
	}
	static bool hasPwm(int pin) {
#if defined(CONFIG_IDF_TARGET_ESP32S2)
		if ((pin >=1 && pin <= 21) || //21
				(pin == 26) || //1
				(pin >= 33 && pin <= 42)) //10
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
		if ((pin >=1 && pin <= 21) || //20
				(pin >= 35 && pin <= 45) || //11
				(pin == 47) || (pin == 48)) //2
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
		if ((pin >=0 && pin <= 10) || //11
				(pin >= 18 && pin <= 21)) //4
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
		if ((pin >=0 && pin <= 9) || //10
				(pin >= 12 && pin <= 23)) //12
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
		if ((pin >=0 && pin <= 5) || //6
				(pin >= 8 && pin <= 14) || //7
				(pin >= 22 && pin <= 27)) //6
#else
		if ((pin == 2) || //1
				(pin == 4) || //1
				(pin == 5) || //1
				((pin >= 12) && (pin <= 19)) || //8
				((pin >= 21) && (pin <= 23)) || //3
				((pin >= 25) && (pin <= 27)) || //3
				(pin == 32) || (pin == 33)) //2
#endif
			return true;
		return false;
	}
	static int channelsRemaining() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		return NUM_PWM + (MCPWM_NUM_UNITS * MCPWM_NUM_TIMERS_PER_UNIT * MCPWM_NUM_OPERATORS_PER_TIMER) - PWMCount;
#else
		return NUM_PWM - PWMCount;
#endif
	}
	static boolean DISABLE_DAC;


};

ESP32PWM* pwmFactory(int pin);

#endif /* LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_ */
