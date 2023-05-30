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
  
}

void loop()
{

  int ULValue = analogRead(UpperLeftPhotoResistor); /// Read Voltage Drop from Upper Left
  int URValue = analogRead(UpperRightPhotoResistor); /// Read Voltage Drop from Upper Right
  int LLValue = analogRead(LowerLeftPhotoResistor); /// Read Voltage Drop from Lower Left
  int LRValue = analogRead(LowerRightPhotoResistor); /// Read Voltage Drop from Lower Right

  ///SECTION Calc the mean of the values verticaly and horizonataly
  int VelticalDif = (ULValue + URValue ) / 2 - (LLValue + LRValue ) / 2 ;
  int HorizontalDif = (ULValue + LLValue ) / 2 - (URValue + LRValue ) / 2 ;

  ///SECTION Impliment the P controller
  float VelticalControllerOutput = VelticalController.getComputedVal(0,VelticalDif);
  float HorizontalControllerOutput = VelticalController.getComputedVal(0,HorizontalDif);

  ///SECTION Drive the control signal
  analogWrite(ServoVerical, VelticalControllerOutput);
  analogWrite(ServoHorizontal, HorizontalControllerOutput);

  
  


}
