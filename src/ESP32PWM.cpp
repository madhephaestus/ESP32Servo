/*
 * ESP32PWM.cpp
 *
 *  Created on: Sep 22, 2018
 *      Author: hephaestus
 */

#include <ESP32PWM.h>
#include "esp32-hal-ledc.h"

// initialize the class variable ServoCount
int ESP32PWM::PWMCount = -1;              // the total number of attached servos
bool  ESP32PWM::explicateAllocationMode=false;
ESP32PWM * ESP32PWM::ChannelUsed[NUM_PWM]; // used to track whether a channel is in service
long ESP32PWM::timerFreqSet[4] = { -1, -1, -1, -1 };
int ESP32PWM::timerCount[4] = { 0, 0, 0, 0 };

#if defined(CONFIG_IDF_TARGET_ESP32S3)
MCPWMTimerInfo ESP32PWM::mcpwmTimers[MCPWM_NUM_UNITS][MCPWM_NUM_TIMERS_PER_UNIT];
#endif

static const char* TAG = "ESP32PWM";

// The ChannelUsed array elements are 0 if never used, 1 if in use, and -1 if used and disposed
// (i.e., available for reuse)
/**
 * allocateTimer
 * @param a timer number 0-3 indicating which timer to allocate in this library
 * Switch to explicate allocation mode
 *
 */
void ESP32PWM::allocateTimer(int timerNumber){
	if(timerNumber<0 || timerNumber>3)
		return;
	if(ESP32PWM::explicateAllocationMode==false){
		ESP32PWM::explicateAllocationMode=true;
		for(int i=0;i<4;i++)
			ESP32PWM::timerCount[i]=4;// deallocate all timers to start mode
	}
	ESP32PWM::timerCount[timerNumber]=0;
}

ESP32PWM::ESP32PWM(bool variableFrequency) : useVariableFrequency(variableFrequency) {
	resolutionBits = 8;
	pwmChannel = -1;
	pin = -1;
	myFreq = -1;
	isMCPWM = false; // Explicitly initialize to false
	if (PWMCount == -1) {
		for (int i = 0; i < NUM_PWM; i++)
			ChannelUsed[i] = NULL; // load invalid data into the storage array of pin mapping
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		// MCPWMTimerInfo struct is initialized with default values in the struct definition
		ESP_LOGI(TAG, "ESP32S3 detected - MCPWM support enabled");
#else
		ESP_LOGI(TAG, "Non-ESP32S3 target - using LEDC only");
#endif
		PWMCount = PWM_BASE_INDEX; // 0th channel does not work with the PWM system
	}
}

ESP32PWM::~ESP32PWM() {
	if (attached()) {
		if (isMCPWM) {
			// MCPWM detach not needed, deallocate handles it
		} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
			ledcDetach(pin);
#else
			ledcDetachPin(pin);
#endif
#else
			ledcDetachPin(pin);
#endif
		}
	}
	deallocate();
}

double ESP32PWM::_ledcSetupTimerFreq(uint8_t pin, double freq,
		uint8_t bit_num, uint8_t channel) {

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
	return ledcAttachChannel(pin, freq, bit_num, channel);
#else
	return ledcSetup(channel, freq, bit_num);
#endif
#else
	return ledcSetup(channel, freq, bit_num);
#endif

}

int ESP32PWM::timerAndIndexToChannel(int timerNum, int index) {
	int localIndex = 0;
	for (int j = 0; j < NUM_PWM; j++) {
		if (((j / 2) % 4) == timerNum) {
			if (localIndex == index) {
				return j;
			}
			localIndex++;
		}
	}
	return -1;
}
int ESP32PWM::allocatenext(double freq) {
	long freqlocal = (long) freq;
	if (pwmChannel < 0 && !isMCPWM) {  // not yet allocated
		if (useVariableFrequency) {
			// try LEDC first
			for (int i = 0; i < 4; i++) {
				bool freqAllocated = ((timerFreqSet[i] == freqlocal) || (timerFreqSet[i] == -1));
				if (freqAllocated && timerCount[i] < 4) {
					if (timerFreqSet[i] == -1) {
						timerFreqSet[i] = freqlocal;
					}
					timerNum = i;
					for (int index=0; index<4; ++index) {
						int myTimerNumber = timerAndIndexToChannel(timerNum,index);
						if ((myTimerNumber >= 0) && (!ChannelUsed[myTimerNumber])) {
							pwmChannel = myTimerNumber;
							ChannelUsed[pwmChannel] = this;
							timerCount[timerNum]++;
							PWMCount++;
							myFreq = freq;
							isMCPWM = false;
							return pwmChannel;
						}
					}
				}
			}
			// if no LEDC, try MCPWM (ESP32S3 only)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
			for(int u=0; u<MCPWM_NUM_UNITS; u++) {
				for(int t=0; t<MCPWM_NUM_TIMERS_PER_UNIT; t++) {
					if (mcpwmTimers[u][t].operatorCount == 0) {
						mcpwmTimers[u][t].freq = freqlocal;
						mcpwmUnit = (mcpwm_unit_t)u;
						mcpwmTimer = (mcpwm_timer_t)t;
						mcpwmOperator = MCPWM_OPR_A;
						mcpwmTimers[u][t].operators[0] = this;
						mcpwmTimers[u][t].operatorCount = 1;
						PWMCount++;
						myFreq = freq;
						isMCPWM = true;
						pwmChannel = -1; // not used
						timerNum = -1;
						return 0; // dummy
					}
				}
			}
#endif
		} else {
			// non-variable, prefer MCPWM with shared freq, fallback to LEDC
#if defined(CONFIG_IDF_TARGET_ESP32S3)
			for(int u=0; u<MCPWM_NUM_UNITS; u++) {
				for(int t=0; t<MCPWM_NUM_TIMERS_PER_UNIT; t++) {
					bool freqMatch = (mcpwmTimers[u][t].freq == freqlocal || mcpwmTimers[u][t].freq == -1);
					if (freqMatch && mcpwmTimers[u][t].operatorCount < MCPWM_NUM_OPERATORS_PER_TIMER) {
						if (mcpwmTimers[u][t].freq == -1) {
							mcpwmTimers[u][t].freq = freqlocal;
						}
						mcpwmUnit = (mcpwm_unit_t)u;
						mcpwmTimer = (mcpwm_timer_t)t;
						int opIndex = mcpwmTimers[u][t].operatorCount;
						mcpwmOperator = (opIndex == 0) ? MCPWM_OPR_A : MCPWM_OPR_B;
						mcpwmTimers[u][t].operators[opIndex] = this;
						mcpwmTimers[u][t].operatorCount++;
						PWMCount++;
						myFreq = freq;
						isMCPWM = true;
						pwmChannel = -1;
						timerNum = -1;
						return 0;
					}
				}
			}
#endif
			// if no MCPWM available, fallback to LEDC shared freq
			for (int i = 0; i < 4; i++) {
				bool freqAllocated = ((timerFreqSet[i] == freqlocal) || (timerFreqSet[i] == -1));
				if (freqAllocated && timerCount[i] < 4) {
					if (timerFreqSet[i] == -1) {
						timerFreqSet[i] = freqlocal;
					}
					timerNum = i;
					for (int index = 0; index < 4; ++index) {
						int myTimerNumber = timerAndIndexToChannel(timerNum, index);
						if ((myTimerNumber >= 0) && (!ChannelUsed[myTimerNumber])) {
							pwmChannel = myTimerNumber;
							ChannelUsed[pwmChannel] = this;
							timerCount[timerNum]++;
							PWMCount++;
							myFreq = freq;
							isMCPWM = false;
							return pwmChannel;
						}
					}
				}
			}
		}
	} else if (isMCPWM) {
		return 0; // already allocated
	} else {
		return pwmChannel;
	}
	ESP_LOGE(TAG, "ERROR All PWM timers allocated! Can't accomodate %.3f Hz\r\nHalting...", freq);
	while (1);
}
void ESP32PWM::deallocate() {
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		ESP_LOGI(TAG, "PWM deallocating MCPWM unit %d timer %d operator %d", mcpwmUnit, mcpwmTimer, mcpwmOperator);
		mcpwmTimers[mcpwmUnit][mcpwmTimer].operatorCount--;
		mcpwmTimers[mcpwmUnit][mcpwmTimer].operators[mcpwmOperator == MCPWM_OPR_A ? 0 : 1] = NULL;
		if (mcpwmTimers[mcpwmUnit][mcpwmTimer].operatorCount == 0) {
			mcpwmTimers[mcpwmUnit][mcpwmTimer].freq = -1;
		}
#else
		// This should never happen - isMCPWM should only be true on ESP32S3
		ESP_LOGE(TAG, "ERROR: MCPWM deallocate attempted on non-ESP32S3 target!");
		isMCPWM = false; // Reset to safe state
#endif
	} else if (pwmChannel >= 0) {
		ESP_LOGI(TAG, "PWM deallocating LEDc #%d",pwmChannel);
		timerCount[getTimer()]--;
		if (timerCount[getTimer()] == 0) {
			timerFreqSet[getTimer()] = -1; // last pwn closed out
		}
		ChannelUsed[pwmChannel] = NULL;
		pwmChannel = -1;
	}
	timerNum = -1;
	attachedState = false;
	isMCPWM = false;
	PWMCount--;
}

int ESP32PWM::getChannel() {
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		return (mcpwmUnit * 10 + mcpwmTimer * 2 + (mcpwmOperator == MCPWM_OPR_A ? 0 : 1)) + 100; // offset to avoid conflict
#else
		// This should never happen - isMCPWM should only be true on ESP32S3
		ESP_LOGE(TAG, "ERROR: MCPWM getChannel attempted on non-ESP32S3 target!");
		isMCPWM = false; // Reset to safe state
		return -1; // Return invalid channel as fallback
#endif
	} else if (pwmChannel < 0) {
		ESP_LOGE(TAG, "FAIL! must setup() before using get channel!");
	}
	return pwmChannel;
}

double ESP32PWM::setup(double freq, uint8_t resolution_bits) {
	if (!isMCPWM) {
		checkFrequencyForSideEffects(freq);
	}

	resolutionBits = resolution_bits;
	if (attached()) {
		if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
			// for MCPWM, re-init timer with new freq
			mcpwm_config_t pwm_config;
			pwm_config.frequency = freq;
			pwm_config.cmpr_a = 0;
			pwm_config.cmpr_b = 0;
			pwm_config.counter_mode = MCPWM_UP_COUNTER;
			pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
			mcpwm_init(mcpwmUnit, mcpwmTimer, &pwm_config);
			mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
			mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
#endif
		} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
			ledcDetach(pin);
			double val = ledcAttachChannel(getPin(), freq, resolution_bits, getChannel());
#else
			ledcDetachPin(pin);
			double val = ledcSetup(getChannel(), freq, resolution_bits);
#endif
#else
			ledcDetachPin(pin);
			double val = ledcSetup(getChannel(), freq, resolution_bits);
#endif
		}
		attachPin(pin);
		return freq;
	}
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		mcpwm_config_t pwm_config;
		pwm_config.frequency = freq;
		pwm_config.cmpr_a = 0;
		pwm_config.cmpr_b = 0;
		pwm_config.counter_mode = MCPWM_UP_COUNTER;
		pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
		mcpwm_init(mcpwmUnit, mcpwmTimer, &pwm_config);
		mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
#endif
		return freq;
	} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
		return ledcAttachChannel(getPin(), freq, resolution_bits, getChannel());
#else
		return ledcSetup(getChannel(), freq, resolution_bits);
#endif
#else
		return ledcSetup(getChannel(), freq, resolution_bits);
#endif
	}
}
double ESP32PWM::getDutyScaled() {
	return mapf((double) myDuty, 0, (double) ((1 << resolutionBits) - 1), 0.0,
			1.0);
}
void ESP32PWM::writeScaled(double duty) {
	write(mapf(duty, 0.0, 1.0, 0, (double) ((1 << resolutionBits) - 1)));
}
void ESP32PWM::write(uint32_t duty) {
	myDuty = duty;
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		float duty_percent = (float)duty / (1 << resolutionBits) * 100.0f;
		mcpwm_set_duty(mcpwmUnit, mcpwmTimer, mcpwmOperator, duty_percent);
#else
		// This should never happen - isMCPWM should only be true on ESP32S3
		ESP_LOGE(TAG, "ERROR: MCPWM operation attempted on non-ESP32S3 target!");
		isMCPWM = false; // Reset to safe state
#endif
	} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
		ledcWrite(getPin(), duty);
#else
		ledcWrite(getChannel(), duty);
#endif
#else
		ledcWrite(getChannel(), duty);
#endif
	}
}
void ESP32PWM::adjustFrequencyLocal(double freq, double dutyScaled) {
	timerFreqSet[getTimer()] = (long) freq;
	myFreq = freq;
	if (attached()) {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
		ledcDetach(pin);
		// Remove the PWM during frequency adjust
		_ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
		writeScaled(dutyScaled);
		ledcAttachChannel(getPin(), freq, resolutionBits, getChannel()); // re-attach the pin after frequency adjust
#else
		ledcDetachPin(pin);
		// Remove the PWM during frequency adjust
		_ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
		writeScaled(dutyScaled);
		ledcAttachPin(pin, getChannel()); // re-attach the pin after frequency adjust
#endif
#else
		ledcDetachPin(pin);
		// Remove the PWM during frequency adjust
		_ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
		writeScaled(dutyScaled);
		ledcAttachPin(pin, getChannel()); // re-attach the pin after frequency adjust
#endif

	} else {
		_ledcSetupTimerFreq(getPin(), freq, resolutionBits, getChannel());
		writeScaled(dutyScaled);
	}
}
void ESP32PWM::adjustFrequency(double freq, double dutyScaled) {
	if (!useVariableFrequency) {
		ESP_LOGE(TAG, "ERROR: Cannot change frequency on fixed-frequency PWM channel (pin %d). Frequency is locked for shared timer operation.", pin);
		return;
	}
	if(dutyScaled<0)
		dutyScaled=getDutyScaled();
	writeScaled(dutyScaled);
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		mcpwmTimers[mcpwmUnit][mcpwmTimer].freq = (long)freq;
		myFreq = freq;
		// re-init timer with new freq
		mcpwm_config_t pwm_config;
		pwm_config.frequency = freq;
		pwm_config.cmpr_a = 0;
		pwm_config.cmpr_b = 0;
		pwm_config.counter_mode = MCPWM_UP_COUNTER;
		pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
		mcpwm_init(mcpwmUnit, mcpwmTimer, &pwm_config);
		mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
		mcpwm_set_duty_type(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
		// update all operators on this timer
		for (int i = 0; i < mcpwmTimers[mcpwmUnit][mcpwmTimer].operatorCount; i++) {
			if (mcpwmTimers[mcpwmUnit][mcpwmTimer].operators[i] != NULL) {
				mcpwmTimers[mcpwmUnit][mcpwmTimer].operators[i]->myFreq = freq;
				mcpwmTimers[mcpwmUnit][mcpwmTimer].operators[i]->writeScaled(mcpwmTimers[mcpwmUnit][mcpwmTimer].operators[i]->getDutyScaled());
			}
		}
#endif
	} else {
		for (int i = 0; i < timerCount[getTimer()]; i++) {
			int pwm = timerAndIndexToChannel(getTimer(), i);
			if (ChannelUsed[pwm] != NULL) {
				if (ChannelUsed[pwm]->myFreq != freq) {
					ChannelUsed[pwm]->adjustFrequencyLocal(freq,
							ChannelUsed[pwm]->getDutyScaled());
				}
			}
		}
	}
}
double ESP32PWM::writeTone(double freq) {
	if (isMCPWM) {
		adjustFrequency(freq, 0.5);
	} else {
		for (int i = 0; i < timerCount[getTimer()]; i++) {
			int pwm = timerAndIndexToChannel(getTimer(), i);
			if (ChannelUsed[pwm] != NULL) {
				if (ChannelUsed[pwm]->myFreq != freq) {
					ChannelUsed[pwm]->adjustFrequencyLocal(freq,
							ChannelUsed[pwm]->getDutyScaled());
				}
				write(1 << (resolutionBits-1)); // writeScaled(0.5);
			}
		}
	}

	return 0;
}
double ESP32PWM::writeNote(note_t note, uint8_t octave) {
	const uint16_t noteFrequencyBase[12] = {
			//   C        C#       D        Eb       E        F       F#        G       G#        A       Bb        B
			4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459,
			7902 };

	if (octave > 8 || note >= NOTE_MAX) {
		return 0;
	}
	double noteFreq = (double) noteFrequencyBase[note]
			/ (double) (1 << (8 - octave));
	return writeTone(noteFreq);
}
uint32_t ESP32PWM::read() {
	if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		// MCPWM doesn't have a read function, return stored duty
		return myDuty;
#else
		// This should never happen - isMCPWM should only be true on ESP32S3
		ESP_LOGE(TAG, "ERROR: MCPWM read attempted on non-ESP32S3 target!");
		isMCPWM = false; // Reset to safe state
		return myDuty; // Return stored duty as fallback
#endif
	} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
		return ledcRead(getPin());
#else
		return ledcRead(getChannel());
#endif
#else
		return ledcRead(getChannel());
#endif
	}
}
double ESP32PWM::readFreq() {
	return myFreq;
}
void ESP32PWM::attach(int p) {
	pin = p;
	attachedState = true;
}
void ESP32PWM::attachPin(uint8_t pin) {

	if (hasPwm(pin)) {
		attach(pin);
		bool success = true;
		if (isMCPWM) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
			mcpwm_io_signals_t signal = (mcpwmOperator == MCPWM_OPR_A) ? MCPWM0A : MCPWM0B;
			if (mcpwmUnit == MCPWM_UNIT_1) signal = (mcpwmOperator == MCPWM_OPR_A) ? MCPWM1A : MCPWM1B;
			mcpwm_gpio_init(mcpwmUnit, signal, pin);
#endif
		} else {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
			success = ledcAttachChannel(pin, readFreq(), resolutionBits, getChannel());
#else
			ledcAttachPin(pin, getChannel());
#endif
#else
			ledcAttachPin(pin, getChannel());
#endif
		}
		if (success) return;
		ESP_LOGE(TAG, "ERROR PWM channel failed to configure on pin %d!", pin);
		return;
	}

#if defined(CONFIG_IDF_TARGET_ESP32S2)
	ESP_LOGE(TAG, "ERROR PWM channel unavailable on pin requested! %d PWM available on: 1-21,26,33-42", pin);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	ESP_LOGE(TAG, "ERROR PWM channel unavailable on pin requested! %d PWM available on: 1-21,35-45,47-48", pin);
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
	ESP_LOGE(TAG, "ERROR PWM channel unavailable on pin requested! %d PWM available on: 1-10,18-21", pin);
#else
	ESP_LOGE(TAG, "ERROR PWM channel unavailable on pin requested! %d PWM available on: 2,4,5,12-19,21-23,25-27,32-33", pin);
#endif

}
void ESP32PWM::attachPin(uint8_t pin, double freq, uint8_t resolution_bits) {

	if (hasPwm(pin)){
        this->pin = pin;
		int ret=setup(freq, resolution_bits);
		ESP_LOGI(TAG, "Pin Setup %d with code %d",pin,ret);
	  attachPin(pin);
	}
	else
		ESP_LOGE(TAG, "ERROR Pin Failed %d ",pin);
}
void ESP32PWM::detachPin(int pin) {
	if (!isMCPWM) {
#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
		ledcDetach(pin);
#else
		ledcDetachPin(pin);
#endif
#else
		ledcDetachPin(pin);
#endif
	}
	deallocate();
}
/* Side effects of frequency changes happen because of shared timers
 *
 * LEDC Chan to Group/Channel/Timer Mapping
 ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
 ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
 ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
 ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
 ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
 ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
 ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
 ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
 ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
 ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
 ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
 ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
 ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
 ** ledc: 13 => Group: 1, Channel: 5, Timer: 2
 ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
 ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */

bool ESP32PWM::checkFrequencyForSideEffects(double freq) {

	allocatenext(freq);
	for (int i = 0; i < timerCount[getTimer()]; i++) {
		int pwm = timerAndIndexToChannel(getTimer(), i);

		if (pwm == pwmChannel)
			continue;
		if (ChannelUsed[pwm] != NULL)
			if (ChannelUsed[pwm]->getTimer() == getTimer()) {
				double diff = abs(ChannelUsed[pwm]->myFreq - freq);
				if (abs(diff) > 0.1) {
					ESP_LOGW(TAG, 
							"\tWARNING PWM channel %d	\
							 shares a timer with channel %d\n	\
							\tchanging the frequency to %.3f		\
							Hz will ALSO change channel %d	\
							\n\tfrom its previous frequency of %.3f Hz\n "
								,pwmChannel, pwm, freq, pwm, ChannelUsed[pwm]->myFreq);
					ChannelUsed[pwm]->myFreq = freq;
				}
			}
	}
	return true;
}

ESP32PWM* pwmFactory(int pin) {
	for (int i = 0; i < NUM_PWM; i++)
		if (ESP32PWM::ChannelUsed[i] != NULL) {
			if (ESP32PWM::ChannelUsed[i]->getPin() == pin)
				return ESP32PWM::ChannelUsed[i];
		}
#if defined(CONFIG_IDF_TARGET_ESP32S3)
	for(int u=0; u<MCPWM_NUM_UNITS; u++) {
		for(int t=0; t<MCPWM_NUM_TIMERS_PER_UNIT; t++) {
			for(int o=0; o<MCPWM_NUM_OPERATORS_PER_TIMER; o++) {
				if (ESP32PWM::mcpwmTimers[u][t].operators[o] != NULL) {
					if (ESP32PWM::mcpwmTimers[u][t].operators[o]->getPin() == pin)
						return ESP32PWM::mcpwmTimers[u][t].operators[o];
				}
			}
		}
	}
#endif
	return NULL;
}
