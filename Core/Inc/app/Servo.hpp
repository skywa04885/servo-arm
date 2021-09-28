/*
 * Servo.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: lrieff
 */

#ifndef INC_APP_SERVO_HPP_
#define INC_APP_SERVO_HPP_

#include "main.h"
#include "stm32f7xx_hal.h"

template <typename T>
class SERVO
{
protected:
	TIM_HandleTypeDef 	*m_TimHandle;
	uint32_t			m_TimChannel;
	T 					m_TargetPos,
						m_MinPos, m_MaxPos,
						m_MinWidth, m_MaxWidth;
public:
	/// Constructs new SERVO class.
	SERVO (TIM_HandleTypeDef *timHandle, uint32_t channel, T minPos, T maxPos, T minWidth, T maxWidth):
		m_TimHandle (timHandle),
		m_TimChannel (channel),
		m_TargetPos (minPos),
		m_MinPos (minPos),
		m_MaxPos (maxPos),
		m_MinWidth (minWidth),
		m_MaxWidth (maxWidth)
	{}

	/// Calculates the pulse register value for the desired angle.
	inline uint16_t CalculatePulse (const T angle)
	{
		// Calculates the length of the pulse in milliseconds.
		const T PERCENTAGE = (angle - m_MinValue) / (m_MaxValue - m_MinValue);
		const T WIDTH = ((m_MaxValue - m_MinValue) * PERCENTAGE) + m_MinValue;

		// Calculates the pulse itself.
		const T PULSES_PER_MS = ((static_cast<T>(__HAL_TIM_GET_AUTORELOAD (m_TimHandle)) / 50.) / 20.);
		const uint16_t PULSE = static_cast<uint16_t>(PULSES_PER_MS * WIDTH);

		// returns the pulse.
		return PULSE;
	}

	/// Moves the SERVO to the given angle.
	void Move (const T angle) noexcept
	{
		m_TargetPos = angle;

		const uint16_t PULSE = CalculatePulse (angle);
		switch (m_Channel)
		{
		case TIM_CHANNEL_1:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_1, PULSE);
			break;
		case TIM_CHANNEL_2:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_2, PULSE);
			break;
		case TIM_CHANNEL_3:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_3, PULSE);
			break;
		case TIM_CHANNEL_4:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_4, PULSE);
			break;
		case TIM_CHANNEL_5:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_5, PULSE);
			break;
		case TIM_CHANNEL_6:
			__HAL_TIM_SET_COMPARE (m_TimHandle, TIM_CHANNEL_6, PULSE);
			break;
		default:
			Error_Handler ();
			break;
		}
	}
public:
};

typedef Servo <float> ServoF;

#endif /* INC_APP_SERVO_HPP_ */
