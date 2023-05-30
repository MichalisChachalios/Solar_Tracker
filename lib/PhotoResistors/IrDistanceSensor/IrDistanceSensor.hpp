#pragma once

#include <iDistanceSensor.hpp>

#include <Poses.hpp>

/// IrDistanceSensor has information about every IR sensor separately
class IrDistanceSensor : public DistanceSensor
{
public:
	/// @brief Construct a new IR distance Sensor.
	IrDistanceSensor(int t_pinValue, int t_pinEmitter, float t_minDistance, float t_maxDistance, Pose2D t_sensorPose, Vector5D<double> t_irCalibrationValues);

	/// @brief Deconstruct IR distance Sensor.
	~IrDistanceSensor();

	/// @brief Initialise hardware.
	void initHardware();

	/// @brief Get the reading of the IR distance sensor in cm.
	/// TODO function getDistance() need to be completed. We need the Calibrator for that.
	float getDistance();

	/// @brief Get Dimension for each IR Object to calculate the transformation to the Robot's center.
	Pose2D getIrPose() const;

	/// @brief Get filtered value of IR distance sensor.
	int getValueFiltered();

	/// @brief Get IR reading with Led On.
	int getIrReadingLedOn();

	/// @brief Turn the LED on.
	void ledOn();

	/// @brief Turn the LED off.
	void ledOff();

private:
	const int m_pinSensor;	   /// Name of pin that the IR sensor is connected.
	const int m_pinLed;		   /// Name of pin that the IR LED is connected.
	const float m_minDistance; /// Min value of IR sensor.
	const float m_maxDistance; /// Max value of IR sensor.
	const Pose2D m_sensorPose; /// Pose of IR sensor.

	const Vector5D<double> m_distanceCalibrationValues;

	int m_value;		 /// Value of IR sensor.
	int m_previousValue; /// Keep filtered values.
};