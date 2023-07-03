#include <Arduino.h>
#include <../PIDController/PIDController.hpp>
#include <Servo.h>

#define UpperLeftPhotoResistor 2
#define UpperRightPhotoResistor 0
#define LowerLeftPhotoResistor 3
#define LowerRightPhotoResistor 1

#define ServoVerical 8
#define ServoHorizontal 7

#define LOOP_TIME = 0.01

Servo servoHor;
Servo servoVer;

PIDController VelticalController = PIDController(4, 0, 0, 0.1);
PIDController HorizontalController = PIDController(4, 0, 0, 0.1);

void setup()
{
  Serial.begin(9600);
  servoHor.attach(6);
  servoVer.attach(7);
  delay(500);
}

void loop()
{

  int uLValue = analogRead(UpperLeftPhotoResistor);  /// Read Voltage Drop from Upper Left
  int uRValue = analogRead(UpperRightPhotoResistor); /// Read Voltage Drop from Upper Right
  int lLValue = analogRead(LowerLeftPhotoResistor);  /// Read Voltage Drop from Lower Left
  int lRValue = analogRead(LowerRightPhotoResistor); /// Read Voltage Drop from Lower Right

  int uLValueprev = uLValue;
  int uRValueprev = uRValue;
  int lLValueprev = lLValue;
  int lRValueprev = lRValue;

  int uLValuemean = (uLValueprev + uLValue) / 2;
  int uRValuemean = (uRValueprev + uRValue) / 2;
  int lLValuemean = (lLValueprev + lLValue) / 2;
  int lRValuemean = (lRValueprev + lRValue) / 2;

  Serial.print(uLValuemean);
  Serial.print(" ");
  Serial.print(uRValuemean);
  Serial.print(" ");
  Serial.print(lLValuemean);
  Serial.print(" ");
  Serial.print(lRValuemean);
  Serial.print(" ");Serial.print(" ");Serial.print(" ");

  /// SECTION Calc the mean of the values verticaly and horizonataly
  float velticalDif = (uLValuemean + uRValuemean) / 2 - (lLValuemean + lRValuemean) / 2;
  float horizontalDif = (uLValuemean + lLValuemean) / 2 - (uRValuemean + lRValuemean) / 2;

  Serial.print(velticalDif);
  Serial.print(" ");
  Serial.print(horizontalDif);Serial.print(" ");Serial.print(" ");

  /// SECTION Implement the P controller
  float velticalControllerOutput = VelticalController.getComputedVal(0, velticalDif);
  float horizontalControllerOutput = VelticalController.getComputedVal(0, horizontalDif);

  Serial.print(velticalControllerOutput);
  Serial.print(" ");
  Serial.println(horizontalControllerOutput);

  /// SECTION Drive the control signal
  analogWrite(ServoVerical, velticalControllerOutput);
  analogWrite(ServoHorizontal, horizontalControllerOutput);
  servoHor.write(130);
  servoVer.write(130);
}
