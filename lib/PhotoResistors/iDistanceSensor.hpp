#pragma once

/// @brief Distance Sensor Interface
class DistanceSensor
{
public:
	/// @brief Return the distance measured by the sensor.
	virtual float getDistance() = 0;
};