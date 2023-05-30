#include "PIDController.hpp"

/// @brief Construct a new PID Controller.
/// @param t_kp KP value.
/// @param t_ki KI value.
/// @param t_kd KD value.
/// @param t_frequency The PIDController's frequency.
PIDController::PIDController(float t_kp, float t_ki, float t_kd, int t_frequency)
{
	/// Initialise member variables.
	m_kp = t_kp;
	m_ki = t_ki;
	m_kd = t_kd;
};

PIDController::~PIDController()
{
};

/// @brief Get the value computed by the PID Controller on each loop.
/// @param t_currentState The system's current state vector, fed back by the sensors.
/// @param t_targetState The system's target state (reference).
/// @return The PID Controller's computed output.
float PIDController::getComputedVal(float t_currentState, float t_targetState)
{
	/// Calculate error.
	m_pidError = t_targetState - t_currentState;
	/// Calculate error integral (sum of all previous errors).
	m_pidErrorSum += m_pidError * m_frequency;
	/// Calculate error derivative (current - previous error).
	m_pidErrorDerivative = (m_pidError - m_pidErrorPrev) / (float)m_frequency;
	/// Set current error as previous before returning.
	m_pidErrorPrev = m_pidError;

	return (m_kp * m_pidError + m_ki * m_pidErrorSum + m_kd * m_pidErrorDerivative);
}

/// SECTION - Get-Set methods, to be used by Robovisor.

/// ANCHOR - Get methods.
/// @brief Get PID Controller's KP value.
/// @return The KP value.
float PIDController::getKp()
{
	return m_kp;
}

/// @brief Get PID Controller's KI value.
/// @return The KI value.
float PIDController::getKi()
{
	return m_ki;
}

/// @brief Get PID Controller's KD value.
/// @return The KD value.
float PIDController::getKd()
{
	return m_kd;
}

/// ANCHOR - Set methods.
/// @brief Set PID Controller's KP value.
/// @param t_kp
void PIDController::setKp(float t_kp)
{
	m_kp = t_kp;
}

/// @brief Set PID Controller's KI value.
/// @param t_ki
void PIDController::setKi(float t_ki)
{
	m_ki = t_ki;
}

/// @brief Set PID Controller's KD value.
/// @param t_kd
void PIDController::setKd(float t_kd)
{
	m_kd = t_kd;
}

///!SECTION