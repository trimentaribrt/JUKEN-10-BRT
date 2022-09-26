/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_BRT_H_
#define HW_BRT_H_

#define HW_NAME					"brt"
#define PCB_PROD_V2

// HW properties

#ifdef PCB_PROD_V1
#define V_REG                       3.3
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_3_SHUNTS
#define INVERTED_SHUNT_POLARITY
#define CURRENT_AMP_GAIN				hw_brt_get_current_sensor_gain()
#define HW_HAS_INPUT_CURRENT_SENSOR
#define INVERTED_INPUT_POLARITY
#define CURRENT_SHUNT_RES           1.0
#endif
#ifdef PCB_PROD_V2
#define V_REG                       3.3
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_3_SHUNTS
#define CURRENT_AMP_GAIN				hw_brt_get_current_sensor_gain()
#define HW_HAS_INPUT_CURRENT_SENSOR
//#define HW_HAS_PHASE_FILTERS
#define CURRENT_SHUNT_RES           1.0
#endif

#define LOW_BATTERY_LIMIT           60
#define HIGH_BATTERY_LIMIT          83
// #define HW_HAS_3_SHUNTS
// #define INVERTED_SHUNT_POLARITY
// #define V_REG                       3.3
// //#define HW_HAS_DRV8323S
// #endif

// #define DRV8323S_CUSTOM_SETTINGS(); drv8323s_set_current_amp_gain(CURRENT_AMP_GAIN); 
// 		drv8323s_write_reg(3,0x3ff); 
// 		drv8323s_write_reg(4,0x7ff); 
// 		drv8323s_write_reg(5,0x66d);

// Macros
//#define ENABLE_GATE()			    palSetPad(GPIOB, 5)
//#define DISABLE_GATE()			palClearPad(GPIOB, 5)

//#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 4))

#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			4
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				5

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PSW_1_GPIO			    GPIOB
#define PSW_1_PIN			    2
#define PSW_2_GPIO			    GPIOC
#define PSW_2_PIN			    9
#define PSW_3_GPIO			    GPIOD
#define PSW_3_PIN			    2

#define PSW_BZ_GPIO			    GPIOB
#define PSW_BZ_PIN			    12

#define PSW_1_ON()			    palSetPad(PSW_1_GPIO, PSW_1_PIN)
#define PSW_1_OFF()			    palClearPad(PSW_1_GPIO, PSW_1_PIN)
#define PSW_2_ON()			    palSetPad(PSW_2_GPIO, PSW_2_PIN)
#define PSW_2_OFF()			    palClearPad(PSW_2_GPIO, PSW_2_PIN)
#define PSW_3_ON()			    palSetPad(PSW_3_GPIO, PSW_3_PIN)
#define PSW_3_OFF()			    palClearPad(PSW_3_GPIO, PSW_3_PIN)
#define PSW_2_TOGGLE()          palTogglePad(PSW_2_GPIO, PSW_2_PIN);

#define PSW_BZ_ON()			    palSetPad(PSW_BZ_GPIO, PSW_BZ_PIN)
#define PSW_BZ_OFF()			palClearPad(PSW_BZ_GPIO, PSW_BZ_PIN)

#define BRAKE_1_GPIO			GPIOC
#define BRAKE_1_PIN			    13
#define BRAKE_2_GPIO			GPIOC
#define BRAKE_2_PIN			    14

#define H_BRAKE_1_GPIO			GPIOB
#define H_BRAKE_1_PIN			3
#define H_BRAKE_2_GPIO			GPIOA
#define H_BRAKE_2_PIN			15

#define SMART_SWITCH_MSECS_PRESSED_ON		5   //x10 (ms)
#define SMART_SWITCH_MSECS_PRESSED_OFF		50  //x10 (ms)
/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			12
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			4

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		9
#define ADC_IND_EXT				6
#define ADC_IND_TEMP_MOS		11
#define ADC_IND_TEMP_MOTOR		8
#define ADC_IND_VREFINT			10
#define ADC_IND_CURR_DC		    7

//#define DEFAULT_CURRENT_AMP_GAIN            0.003809524    //350A HALL CURRENT SENSOR
#define DEFAULT_CURRENT_AMP_GAIN            0.002666667    //500A HALL CURRENT SENSOR

#define DEFAULT_INPUT_CURRENT_AMP_GAIN		0.006666667    //200A HALL CURRENT SENSOR

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.44
#endif
#ifndef VIN_R1
#define VIN_R1					150000.0
#endif
#ifndef VIN_R2
#define VIN_R2					4700.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.0005 / 3.0)
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2)+1)

//Input current
#define GET_INPUT_CURRENT()				hw_brt_read_input_current()
#define GET_INPUT_CURRENT_OFFSET()		hw_brt_get_input_current_offset()
#define MEASURE_INPUT_CURRENT_OFFSET()	hw_brt_start_input_current_sensor_offset_measurement()

// NTC Termistors
#define NTC_RES(adc_val)		(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

#define DRV8323S_MOSI_GPIO		GPIOC
#define DRV8323S_MOSI_PIN		12
#define DRV8323S_MISO_GPIO		GPIOC
#define DRV8323S_MISO_PIN		11
#define DRV8323S_SCK_GPIO		GPIOC
#define DRV8323S_SCK_PIN        10
#define DRV8323S_CS_GPIO        GPIOC
#define DRV8323S_CS_PIN			9

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD1
#define HW_UART_P_GPIO_AF		GPIO_AF_USART1
#define HW_UART_P_TX_PORT		GPIOB
#define HW_UART_P_TX_PIN		6
#define HW_UART_P_RX_PORT		GPIOB
#define HW_UART_P_RX_PIN		7

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				SPID3
#define HW_SPI_GPIO_AF			GPIO_AF_SPI3
#define HW_SPI_PORT_NSS			GPIOC
#define HW_SPI_PIN_NSS			9
#define HW_SPI_PORT_SCK			GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI			12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO			11

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		1000.0

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			25.0	// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			96.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		400.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-60.0	// Input current limit in Amperes (Lower)
#endif

#define HW_LIM_CURRENT_BAT_MAX  80.0
// Setting limits
#define HW_LIM_CURRENT			-300.0, 300.0
#define HW_LIM_CURRENT_IN		-300.0, 300.0
#define HW_LIM_CURRENT_ABS		0.0, 420.0
#define HW_LIM_VIN				23.0, 97.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

#define ANGLE_TO_DAC_VALUE(angle)		( angle * 512.0 + 0x800 )//angle between -pi to pi
#define TOGGLE_DEBUG_PIN()              palTogglePad(GPIOA,5)
// HW-specific functions

float hw_brt_get_current_sensor_gain(void);
float hw_brt_read_input_current(void);
void hw_brt_get_input_current_offset(void);
void hw_brt_start_input_current_sensor_offset_measurement(void);
void hw_DAC1_setdata(uint16_t data);
void hw_DAC2_setdata(uint16_t data);

#endif /* HW_BRT_H_ */
