#include <Arduino.h>
#include <../PIDController/PIDController.hpp>

#define UpperLeftPhotoResistor 3
#define UpperRightPhotoResistor 4
#define LowerLeftPhotoResistor 5
#define LowerRightPhotoResistor 6

#define ServoVerical 7
#define ServoHorizontal 8

#define LOOP_TIME = 0.01 

PIDController VelticalController =  PIDController(4, 0, 0, 0.1);
PIDController HorizontalController =  PIDController(4, 0, 0,0.1);

void setup()
{
  delay(500);
}

void loop()
{

  int uLValue = analogRead(UpperLeftPhotoResistor); /// Read Voltage Drop from Upper Left
  int uRValue = analogRead(UpperRightPhotoResistor); /// Read Voltage Drop from Upper Right
  int lLValue = analogRead(LowerLeftPhotoResistor); /// Read Voltage Drop from Lower Left
  int lRValue = analogRead(LowerRightPhotoResistor); /// Read Voltage Drop from Lower Right

  ///SECTION Calc the mean of the values verticaly and horizonataly
  int velticalDif = (uLValue + uRValue ) / 2 - (lLValue + lRValue ) / 2 ;
  int horizontalDif = (uLValue + lLValue ) / 2 - (uRValue + lRValue ) / 2 ;

  ///SECTION Implement the P controller
  float velticalControllerOutput = VelticalController.getComputedVal(0,velticalDif);
  float horizontalControllerOutput = VelticalController.getComputedVal(0,horizontalDif);

  ///SECTION Drive the control signal
  analogWrite(ServoVerical, velticalControllerOutput);
  analogWrite(ServoHorizontal, horizontalControllerOutput);

}
