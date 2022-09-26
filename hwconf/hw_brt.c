/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <math.h>
#include "mc_interface.h"
#include "commands.h"
#include "terminal.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "gpdrive.h"
#include "app.h"
#include "mempools.h"
#include "timeout.h"
#include "stdio.h"

//private functions
static void terminal_cmd_doublepulse(int argc, const char** argv);
void hw_setup_dac(void);

// Variables
static volatile bool i2c_running = false;
static volatile float current_sensor_gain = 0.0;
static volatile float input_current_sensor_gain = 0.0;
static volatile float input_current_sensor_offset = 1.65;
static volatile uint16_t input_current_sensor_offset_samples = 0;
static volatile uint32_t input_current_sensor_offset_sum = 0;
static volatile bool current_input_sensor_offset_start_measurement = false;

typedef enum {
	SWITCH_BOOT = 0,
	SWITCH_PARK_PRESSED,
	SWITCH_PARK,
	SWITCH_PARK_WAIT_RELEASE,
	SWITCH_TURNED_ON_PRESSED,
	SWITCH_TURNED_ON,
	SWITCH_TURNED_ON_WAIT_RELEASE,
} switch_states;

typedef enum {
	INDICATOR_OFF = 0,
	INDICATOR_ON,
	INDICATOR_BLINK
} indicator_states;

static THD_WORKING_AREA(gas_gauge_thread_wa, 128);
static THD_WORKING_AREA(smart_switch_thread_wa, 128);
static THD_WORKING_AREA(smart_indicator_thread_wa, 128);
THD_WORKING_AREA(dac_debug_thread_wa, 128);
static THD_FUNCTION(gas_gauge_thread, arg);
static THD_FUNCTION(smart_switch_thread, arg);
static THD_FUNCTION(smart_indicator_thread, arg);
static THD_FUNCTION(dac_debug_thread, arg);

static volatile switch_states switch_state = SWITCH_BOOT;
static volatile indicator_states indicator_state = INDICATOR_OFF;
volatile uint16_t inputVoltage = 0;

void LockMotor(void){
	mc_interface_set_current(0);
	mc_interface_lock();
}
void UnlockMotor(void){
	mc_interface_unlock();
}
void SetGaugePercent(uint16_t percent){
	
}
void SetIndicatorMode(uint16_t onBlink, uint16_t offBlink, uint16_t priodms){
	
}
void SetBuzzerSound(uint16_t onBlink, uint16_t offBlink){
	
}
uint16_t gaugeCounter = 0;
uint16_t gaugePeriodCount = 100;
uint16_t gaugeOnCount = 0;

uint16_t GetGaugePercentage(uint16_t voltage){
	uint16_t percentTemp = 0;
	uint16_t gaugeRange = 0;
	gaugeRange = HIGH_BATTERY_LIMIT - LOW_BATTERY_LIMIT;
	if(voltage > HIGH_BATTERY_LIMIT)
		return 100;
	if(voltage < LOW_BATTERY_LIMIT)
		return 0;
	percentTemp = (voltage - LOW_BATTERY_LIMIT) * 100 / gaugeRange;
	return percentTemp;
}
static THD_FUNCTION(gas_gauge_thread, arg){
	(void)arg;
	chRegSetThreadName("gas_gauge");
	for (;;) {
		if(inputVoltage < LOW_BATTERY_LIMIT){
			//under voltage limit
			PSW_3_ON();//turn on low voltage indicator
		}
		else if(inputVoltage > (LOW_BATTERY_LIMIT+2)){
			//normal voltage range
			PSW_3_OFF();//turn off low voltage indicator
		}
		if(gaugeCounter){
			gaugeCounter--;
			if(gaugeCounter < gaugeOnCount)
				PSW_1_ON();
			else
				PSW_1_OFF();
		}
		else
		{
			gaugeCounter = gaugePeriodCount;
		}
		chThdSleepMicroseconds(100);
	}
}
static THD_FUNCTION(smart_indicator_thread, arg) {
	(void)arg;
	chRegSetThreadName("smart_indicator");
	for (;;) {
		inputVoltage = GET_INPUT_VOLTAGE();
		gaugeOnCount = GetGaugePercentage(inputVoltage);
		switch (indicator_state)
		{
		case INDICATOR_OFF:
			PSW_2_OFF();
			//PSW_BZ_OFF();
			break;
		case INDICATOR_ON:
			PSW_2_ON();
			//PSW_BZ_ON();
			break;
		case INDICATOR_BLINK:
			PSW_2_TOGGLE();
			break;
		default:
			break;
		}
		chThdSleepMilliseconds(100);
	}
}
bool low_brake_is_pressed(void) {
	if(palReadPad(BRAKE_1_GPIO, BRAKE_1_PIN) == 0)
		return true;
	else
		return false;
}
bool high_brake_is_pressed(void) {
	if(palReadPad(H_BRAKE_1_GPIO, H_BRAKE_1_PIN) == 0)
		return true;
	else
		return false;
}
static THD_FUNCTION(smart_switch_thread, arg) {
	(void)arg;
	chRegSetThreadName("smart_switch");
	unsigned int millis_switch_pressed = 0;
	unsigned int chargeWhenRunning = 0;
	for (;;) {
		mc_state state1 = mc_interface_get_state();
		switch(switch_state){
			case SWITCH_BOOT:
				while (!mcpwm_foc_is_dccal_done()) {
					chThdSleepMilliseconds(200);
				}
				//lock motor at init, as need to press park button
				LockMotor();
				switch_state = SWITCH_PARK;
			break;
			case SWITCH_PARK:
				LockMotor();
				if(low_brake_is_pressed())
				switch_state = SWITCH_PARK_PRESSED;
			break;
			case SWITCH_PARK_PRESSED:
			//switch is pressed, wait until SMART_SWITCH_MSECS_PRESSED_ON
				if (millis_switch_pressed > SMART_SWITCH_MSECS_PRESSED_ON) {
					switch_state = SWITCH_TURNED_ON_WAIT_RELEASE;
					SetBuzzerSound(1,1);
					indicator_state = INDICATOR_ON;
					UnlockMotor();
					millis_switch_pressed = 0;
					chargeWhenRunning = 0;
				}
				if (low_brake_is_pressed()) {
					millis_switch_pressed++;
				} else {
					millis_switch_pressed = 0;
					switch_state = SWITCH_PARK;
				}
			break;
			case SWITCH_PARK_WAIT_RELEASE:
				if(!low_brake_is_pressed())
					switch_state = SWITCH_PARK;
			break;
			case SWITCH_TURNED_ON:
				if(low_brake_is_pressed())
				switch_state = SWITCH_TURNED_ON_PRESSED;

				// if(high_brake_is_pressed()){
				// 	if(state1 != MC_STATE_RUNNING){
				// 		chargeWhenRunning = 1;
				// 		LockMotor();
				// 	}
				// }
				// else {
				// 	if(chargeWhenRunning){
				// 		chargeWhenRunning = 0;
				// 		UnlockMotor();
				// 	}
				// }
			break;
			case SWITCH_TURNED_ON_PRESSED:
				//switch is pressed, wait until SMART_SWITCH_MSECS_PRESSED_ON
				if (millis_switch_pressed > SMART_SWITCH_MSECS_PRESSED_OFF) {
					switch_state = SWITCH_PARK_WAIT_RELEASE;
					SetBuzzerSound(1,1);
					indicator_state = INDICATOR_OFF;
					UnlockMotor();
					millis_switch_pressed = 0;
				}
				if (low_brake_is_pressed()) {
					millis_switch_pressed++;
				} else {
					millis_switch_pressed = 0;
					switch_state = SWITCH_TURNED_ON;
				}
			break;
			case SWITCH_TURNED_ON_WAIT_RELEASE:
				if(!low_brake_is_pressed())
					switch_state = SWITCH_TURNED_ON;
			break;
			default:
			break;
		}
		chThdSleepMilliseconds(10);
	}
}

void gas_gauge_thread_start(void) {
	chThdCreateStatic(gas_gauge_thread_wa, sizeof(gas_gauge_thread_wa),
					  LOWPRIO, gas_gauge_thread, NULL);
}
void smart_switch_thread_start(void) {
	chThdCreateStatic(smart_switch_thread_wa, sizeof(smart_switch_thread_wa),
					  LOWPRIO, smart_switch_thread, NULL);
}
void smart_indicator_thread_start(void) {
	chThdCreateStatic(smart_indicator_thread_wa, sizeof(smart_indicator_thread_wa),
					  LOWPRIO, smart_indicator_thread, NULL);
}

static THD_FUNCTION(dac_debug_thread, arg) {
	(void)arg;
	chRegSetThreadName("dac_debug");
	static uint16_t val1dac=2048;
	static uint16_t val2dac=0;
	for (;;) {
		//hw_DAC1_setdata(val1dac);
		hw_DAC2_setdata(val2dac);
		val1dac++;
		val2dac++;
		if(val1dac>0x1000)
		val1dac=0;
		if(val2dac>0x1000)
		val2dac=0;
		chThdSleepMilliseconds(1);
	}
}

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

#ifdef HW_HAS_DRV8323S
	// ENABLE_GATE
	palSetPadMode(GPIOB, 12,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);	

	palSetPad(GPIOB, 12);
	
	ENABLE_GATE();
#endif

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);		//voltage sense 1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);		//voltage sense 2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);		//voltage sense 3
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);		//mosfet temp

	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);		//dc input current
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);		//aux
	
	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);		//dc input current
	
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);		//current sense 1
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);		//current sense 2
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);		//current sense 3
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);		//dc voltage
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);		//motor temp
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);		//tps input


	//set output switch pin
	palSetPadMode(PSW_1_GPIO, PSW_1_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);//gauge meter controller
	palSetPadMode(PSW_2_GPIO, PSW_2_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);//dashboard smart indicator 
	palSetPadMode(PSW_3_GPIO, PSW_3_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);//dashboard neutral 
	palSetPadMode(PSW_BZ_GPIO, PSW_BZ_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);//buzzer


	//set input switch pin
	palSetPadMode(BRAKE_1_GPIO, BRAKE_1_PIN, PAL_MODE_INPUT_PULLUP);//parking button
	palSetPadMode(BRAKE_2_GPIO, BRAKE_2_PIN, PAL_MODE_INPUT_PULLUP);//side stand

	palSetPadMode(H_BRAKE_1_GPIO, H_BRAKE_1_PIN, PAL_MODE_INPUT_PULLUP);//high brake for charger pause


	PSW_1_OFF();
	PSW_2_OFF();
	PSW_3_OFF();
	PSW_BZ_OFF();
	gas_gauge_thread_start();
	smart_switch_thread_start();
	smart_indicator_thread_start();
#ifdef HW_HAS_DRV8323S
	drv8323s_init();
#endif
	//hw_setup_dac();

	//chThdCreateStatic(dac_debug_thread_wa, sizeof(dac_debug_thread_wa), LOWPRIO, dac_debug_thread, NULL);
#ifndef HW_HAS_DUAL_MOTORS
	//register terminal callbacks
	//double pulse not possible with dual motor setup
	// terminal_register_command_callback(
	// 	"double_pulse",
	// 	"Start a double pulse test",
	// 	0,
	// 	terminal_cmd_doublepulse);
#endif
	current_sensor_gain = DEFAULT_CURRENT_AMP_GAIN;
	input_current_sensor_gain = DEFAULT_INPUT_CURRENT_AMP_GAIN;
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);			// 0	SENS1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);			// 3	CURR1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, ADC_SampleTime_15Cycles);			// 6	THROTTLE
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);			// 9	VBAT IN

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);			// 1	SENS2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);			// 4	CURR2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 3, ADC_SampleTime_15Cycles);			// 7	EXT CURRENT
	ADC_RegularChannelConfig(ADC2, ADC_Channel_Vrefint, 4, ADC_SampleTime_15Cycles); 	// 10	VREFINT

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);			// 2	SENS3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);			// 5	CURR3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 3, ADC_SampleTime_15Cycles);			// 8	TEMP_MOTOR
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 4, ADC_SampleTime_15Cycles);			// 11	TEMP_MOSFET

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}
void hw_setup_dac(void) {
	// GPIOA clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// DAC Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	// DAC channel 1 & 2 (DAC_OUT1 = PA.4)(DAC_OUT2 = PA.5) configuration
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	//palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);	
	// Enable both DAC channels with output buffer disabled to achieve rail-to-rail output
	//DAC->CR |= DAC_CR_EN1 | DAC_CR_BOFF1 | DAC_CR_EN2 | DAC_CR_BOFF2;
	DAC->CR |= DAC_CR_EN1 | DAC_CR_BOFF1;
	// Set DAC channels at 1.65V
	hw_DAC1_setdata(0xC00);
	//hw_DAC2_setdata(0x800);
}
void hw_DAC1_setdata(uint16_t data) {
	DAC->DHR12R1 = data;
}

void hw_DAC2_setdata(uint16_t data) {
	DAC->DHR12R2 = data;
}

static void terminal_cmd_doublepulse(int argc, const char** argv)
{
	(void)argc;
	(void)argv;

	int preface, pulse1, breaktime, pulse2;
	int utick;
	int deadtime = -1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	if (argc < 5) {
		commands_printf("Usage: double_pulse <preface> <pulse1> <break> <pulse2> [deadtime]");
		commands_printf("   preface: idle time in  µs");
		commands_printf("    pulse1: high time of pulse 1 in µs");
		commands_printf("     break: break between pulses in µs\n");
		commands_printf("    pulse2: high time of pulse 2 in µs");
		commands_printf("  deadtime: overwrite deadtime, in ns");
		return;
	}
	sscanf(argv[1], "%d", &preface);
	sscanf(argv[2], "%d", &pulse1);
	sscanf(argv[3], "%d", &breaktime);
	sscanf(argv[4], "%d", &pulse2);
	if (argc == 6) {
		sscanf(argv[5], "%d", &deadtime);
	}
	timeout_configure_IWDT_slowest();

	utick = (int)(SYSTEM_CORE_CLOCK / 1000000);
	mcpwm_deinit();
	mcpwm_foc_deinit();
	gpdrive_deinit();

	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	//TIM4 als Trigger Timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / 20000);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Enable);
	TIM4->CNT = 0;

	// TIM1
	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (preface + pulse1) * utick;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = preface * utick;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);


	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	if (deadtime < 0) {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	} else {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(deadtime, SYSTEM_CORE_CLOCK);
	}
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM1->CNT = 0;
	TIM1->EGR = TIM_EGR_UG;

	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR3);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	//Timer 4 triggert Timer 1
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
	TIM1->ARR = (breaktime + pulse2) * utick;
	TIM1->CCR1 = breaktime * utick;
	while (TIM1->CNT != 0);
	TIM_Cmd(TIM4, ENABLE);

	chThdSleepMilliseconds(1);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	switch (mcconf->motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_init(mcconf);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_init(mcconf, mcconf);
		break;

	case MOTOR_TYPE_GPD:
		gpdrive_init(mcconf);
		break;

	default:
		break;
	}
	commands_printf("Done");
	return;
}

inline float hw_brt_get_current_sensor_gain() {
	return current_sensor_gain;
}

float hw_brt_read_input_current(void) {
	float ret_value = 0.0;
	if(input_current_sensor_gain > 0.0001){
		ret_value = (((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_CURR_DC]) - input_current_sensor_offset ) / input_current_sensor_gain;
		#ifdef INVERTED_INPUT_POLARITY
		ret_value = ret_value * -1;
		#endif
		}
	return ret_value;
}

void hw_brt_get_input_current_offset(void){

	if(current_input_sensor_offset_start_measurement){

		if( input_current_sensor_offset_samples == 100 ){
			current_input_sensor_offset_start_measurement = false;
			input_current_sensor_offset = ((float)input_current_sensor_offset_sum) / 100.0;
			input_current_sensor_offset *= (V_REG / 4095.0);
		}
		else{
			input_current_sensor_offset_sum += 	ADC_Value[ADC_IND_CURR_DC];
			input_current_sensor_offset_samples++;
		}
	}else{
		input_current_sensor_offset_samples++;
	}
}

void hw_brt_start_input_current_sensor_offset_measurement(void){
	current_input_sensor_offset_start_measurement = true;
	input_current_sensor_offset_samples = 0;
	input_current_sensor_offset_sum = 0;
}
