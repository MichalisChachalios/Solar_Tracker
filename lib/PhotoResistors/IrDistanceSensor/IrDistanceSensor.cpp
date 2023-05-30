#include "IrDistanceSensor.hpp"
#include <Arduino.h>
#include <AriadneConfig.hpp>

/// @brief Construct a new IR distance Sensor.
/// @param t_sensorPin Analog pin that connects to Teensy.
/// @param t_pinLed LED on/off pin.
/// @param t_minDistance Minimum distance the sensor will return valid measurement for.
/// @param t_maxDistance Maximum distance the sensor will return valid measurement for.
/// @param t_sensorPose The sensor's pose on the Robot (x, y, theta).
IrDistanceSensor::IrDistanceSensor(
	int t_pinValue,
	int t_pinLed,
	float t_minDistance,
	float t_maxDistance,
	Pose2D t_sensorPose,
	Vector5D<double> t_irCalibrationValues) : m_pinSensor(t_pinValue),
						   m_pinLed(t_pinLed),
						   m_minDistance(t_minDistance),
						   m_maxDistance(t_maxDistance),
						   m_sensorPose(t_sensorPose),
						   m_distanceCalibrationValues(t_irCalibrationValues)
{
	// this->initHardware();
}

/// @brief Deconstruct IR distance Sensor.
IrDistanceSensor::~IrDistanceSensor()
{
}

/// @brief Initialise hardware.
void IrDistanceSensor::initHardware()
{
	/// Initialise sensor's pins as input or output.
	pinMode(m_pinSensor, INPUT);
	pinMode(m_pinLed, OUTPUT);
}

/// @brief Get the reading of the IR distance sensor in cm.
/// @return The distance from sensor in cm. If distance is less than the min returns -1, if the distance is greater than the maximun returns -2
float IrDistanceSensor::getDistance()
{
	/// TODO - Add these parameters to sensor's constructor. We need the Calibrator for that.
	float x = this->getIrReadingLedOn();
	// Serial.print(x);
	float a = m_distanceCalibrationValues.x1;
	// float b = m_distanceCalibrationValues.x2;
	// float c = m_distanceCalibrationValues.x3;
	// float d = m_distanceCalibrationValues.x4;
	// float e = m_distanceCalibrationValues.x5;



	// float distance = a * pow(x, 4) + b * pow(x, 3) +  c * pow(x, 2)+ d * pow(x, 1)+ e * pow(x, 0);
	// float a = 9496.87777;
	// float b = 800.4022;
	// float distance = ( 1/b)*(exp((a-x)/b));
	
	if (x > a)
	{
		/// NOTE: Fix when we calibrate IR in mm.
		return -1000000;
	}
	else if ( x<=a)
	{
		return 1000000;
	}
	/// if the value is invalid the output must be a big negative numeber becouse in the transformation to center
	/// function we add the offset, possible making it positive again
	// else if (distance < m_minDistance)
	// {
	// 	return -1000;
	// }
	// else if (distance > m_maxDistance)
	// {
	// 	return -2000;
	// }
	// else
	// {
	// 	/// Should never reach here.
	// 	return -3000;
	// }
}

///
/// @brief Get filtered value of IR distance sensor.
///
/// @return The filtered ADC value.
int IrDistanceSensor::getValueFiltered()
{
	int valLedOff = analogRead(m_pinSensor); /// Read value with LED off.
	this->ledOn();							 /// Turn LED on.
	delayMicroseconds(sensors::DELAY_EMITTER_TURN_ON);					 /// Wait for LED and transistor to rise.
	int valLedOn = analogRead(m_pinSensor);	 /// Read value with LED on.
	this->ledOff();							 /// Turn emmiter off.
	delayMicroseconds(2);					 /// Wait for emmiter to fall.
	return abs(valLedOn - valLedOff);		 /// Return the asbolute difference of the measurements.
}

int IrDistanceSensor::getIrReadingLedOn()
{
	this->ledOn();							/// Turn LED on.
	delayMicroseconds(sensors::DELAY_EMITTER_TURN_ON);					/// Wait for LED and transistor to rise.
	int valLedOn = analogRead(m_pinSensor); /// Read value with LED on.
	this->ledOff();							/// Turn emmiter off.
	delayMicroseconds(2);					/// Wait for emmiter to fall.
	return (valLedOn);						/// Return the asbolute difference of the measurements.
}

/// @brief Turn the emmiter on.
void IrDistanceSensor::ledOn()
{
	digitalWrite(m_pinLed, HIGH);
}

/// @brief Turn the LED off.
void IrDistanceSensor::ledOff()
{
	digitalWrite(m_pinLed, LOW);
}

/// @brief Get Dimension for each IR Object to calculate the transformation to the Robot's center.
/// @return Pose of IR Sensor
Pose2D IrDistanceSensor::getIrPose() const
{
	return this->m_sensorPose;
}