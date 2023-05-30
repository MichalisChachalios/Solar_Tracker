#pragma once

class PIDController
{
public:
	/// @brief Construct a new PID Controller.
	PIDController(float t_KP, float t_KI, float t_KD, int t_frequency);
	/// @brief Deconstruct PID Controller.
	~PIDController();

	/// @brief Get the value computed by the PID Controller on each loop.
	float getComputedVal(float t_currentState, float t_targetState);

	/// @brief Set PID Controller's KP value.
	void setKp(float t_kp);
	/// @brief Set PID Controller's KI value.
	void setKi(float t_ki);
	/// @brief Set PID Controller's KD value.
	void setKd(float t_kd);

	/// @brief Get PID Controller's KP value.
	float getKp();
	/// @brief Get PID Controller's KI value.
	float getKi();
	/// @brief Get PID Controller's KD value.
	float getKd();

private:
	/// Error passed into PID controller.
	float m_pidError = 0;
	/// Previous loop PID error (used in derivative calculation).
	float m_pidErrorPrev = 0;
	/// Sum of all previous PID errors (used in integral calculation).
	float m_pidErrorSum = 0;

	float m_pidErrorDerivative = 0;

	/// ANCHOR -  PID Control Parameters
	/// The Controller's update frequency.
	unsigned int m_frequency;
	/// Kp, Ki, Kd values.
	float m_kp = 0;
	float m_ki = 0;
	float m_kd = 0;
};
