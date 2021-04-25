/*
 * main.c
 *
 * Created: 4/19/2021 8:59:18 PM
 *  Author: Seba
 */ 

 #define __DELAY_ROUND_CLOSEST__

/* debug session flag */
//#define S_DEBUG

#include <xc.h>
#include <util/delay.h>
#include <avr/io.h>

#include "tinudht.h"


#define ALIVE_LOOP_PERIOD		1000
#define CHECK_LOOP_COUNT		(uint8_t) 60
#define HISTERESIS		5U

#define Std_RetVal		uint8_t
#define E_OK			(uint8_t) 0U
#define E_NOT_OK		(uint8_t) 1U

#define TINUDHT_PIN		PORTB2
#define PIN_RELAY		PORTB3
#define PIN_ERROR		PORTB1
#define PIN_ALIVE		PORTB0

#ifdef S_DEBUG
#define PIN_DEBUG_VAL	PORTB1
#define PIN_DEBUG_PWM	PORTB0
#endif

#define DEBUG_1_LEN		3		/* length of 1 in debug print, in ms */
#define DEBUG_0_LEN		1		/* length of 0 in debug print, in ms */

#define SetPin(pin)		PORTB |= (1 << pin)
#define ClearPin(pin)	PORTB &= ~(1 << pin)
#define TogglePin(pin)	PORTB ^= (1 << pin);

#define SetAlivePin()		SetPin(PIN_ALIVE)
#define ClearAlivePin()		ClearPin(PIN_ALIVE)
#define ToggleAlivePin()	TogglePin(PIN_ALIVE)

#define SetErrorPin()	SetPin(PIN_ERROR)
#define ClearErrorPin()	ClearPin(PIN_ERROR)

#define SetDebugPin()	SetPin(PIN_DEBUG_VAL)
#define ClearDebugPin()	ClearPin(PIN_DEBUG_VAL)

#define SetRelayPin()	SetPin(PIN_RELAY)
#define ClearRelayPin()	ClearPin(PIN_RELAY)


typedef enum
{
	State_Undefined = 0,
	State_OFF,
	State_ON,
}States_enum;



#ifdef S_DEBUG
void print_u08(uint8_t val)
{
	for (uint8_t i = 8; i > 0; i--)
	{
		ClearDebugPin();
		_delay_ms(DEBUG_0_LEN);  
		SetDebugPin();
		if (val & (1 << (i - 1)))
		{
			_delay_ms(DEBUG_1_LEN); 
		}
		else
		{
			_delay_ms(DEBUG_0_LEN); 
			ClearDebugPin();
			_delay_ms(DEBUG_1_LEN - DEBUG_0_LEN);
		}
	}
	ClearDebugPin();
	_delay_ms(DEBUG_1_LEN);
}
#endif

void init()
{
	/* Init adc on PB4 */
	ADMUX |= (0 << REFS0) | (1 << ADLAR) | (1 << MUX1);
	ADCSRA |= 1<< ADEN;
	ADCSRA |= (1 << ADPS0) | (1 << ADPS0) | (1 << ADPS0);
	
	/* Set output pins */
	DDRB |= (1 << PIN_RELAY) | (1 << PIN_ERROR)| (1 << PIN_ALIVE);

#ifdef S_DEBUG
	DDRB |= (1 << PIN_DEBUG_VAL) | (1 << PIN_DEBUG_PWM);
	/* Setup debug PWM on PB0 */
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
	TCCR0B |= (1 << CS02);
	// OCR0A = 0x81;
#else
	SetAlivePin();
	SetErrorPin();
#endif	

}

uint8_t ScaleValue_255_2_100 (uint8_t val)
{
#ifdef S_DEBUG
	print_u08(val);
#endif
	return (uint8_t) ((val * 100U) / 255U);
}

uint8_t Adc_Read()
{
	ADCSRA |= (1 << ADSC);
	
	// Wait until the ADSC bit has been cleared
	while(ADCSRA & (1 << ADSC));
	return ADCH;
}

void SetRelay(uint8_t Humidity, uint8_t Setting)
{
	static States_enum Last_State_u08 = State_Undefined;

#ifdef S_DEBUG
	print_u08(Humidity);
#endif
	
	/* Check is setting is not to low */
	Setting = (Setting < HISTERESIS) ? HISTERESIS : Setting;

#ifdef S_DEBUG
	print_u08(Setting);
#endif


	switch (Last_State_u08)
	{
	case State_ON:
		{
			if (Humidity < Setting - HISTERESIS)
			{
				ClearRelayPin();
				Last_State_u08 = State_OFF;
			}
			break;
		}
	case State_OFF:
		{
			if (Humidity > Setting + HISTERESIS)
			{
				SetRelayPin();
				Last_State_u08 = State_ON;
			}
			break;
		}
	default:
		{
			if (Humidity > Setting)
			{
				SetRelayPin();
				Last_State_u08 = State_ON;
			}
			else
			{
				ClearRelayPin();
				Last_State_u08 = State_OFF;
			}
		}
	}
}

Std_RetVal Operate()
{
	uint8_t RetVal_u08 = E_NOT_OK;
	uint8_t AdcVal_u08;
	TinuDHT TempHum_str;

	AdcVal_u08  = Adc_Read();
	
	switch (tinudht_read(&TempHum_str, TINUDHT_PIN))
	{
		case TINUDHT_OK:
		{
			OCR0A = TempHum_str.humidity;
			SetRelay(TempHum_str.humidity, ScaleValue_255_2_100(AdcVal_u08));
			RetVal_u08 = E_OK;
#ifndef S_DEBUG
			ClearErrorPin();
#endif
			break;
		}
		case (uint8_t)TINUDHT_ERROR_CHECKSUM:
		{
			OCR0A = 254U;
#ifndef S_DEBUG
			SetErrorPin();
#endif
			break;
		}
		case (uint8_t)TINUDHT_ERROR_TIMEOUT:
		{
			OCR0A = 25U;
#ifndef S_DEBUG
			SetErrorPin();
#endif
			break;
		}
	}

	#ifdef S_DEBUG
	print_u08(OCR0A);
	print_u08(TempHum_str.humidity);
	#endif

	return RetVal_u08;

}

int main(void)
{
	uint8_t LoopCounter_u08 = CHECK_LOOP_COUNT;

	init();

    while(1)
    {

#ifndef S_DEBUG	
		ToggleAlivePin();
#endif

		if (LoopCounter_u08 < CHECK_LOOP_COUNT)
		{
			LoopCounter_u08++;
		}
		else
		{
			if (E_OK == Operate())
			{
				LoopCounter_u08 = 1U;
			}
		}
				
		_delay_ms(ALIVE_LOOP_PERIOD);
		
    }
	
}