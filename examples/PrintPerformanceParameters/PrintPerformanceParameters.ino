#include <SmartDrive.h>

SmartDrive smd = SmartDrive(SmartDrive_DefaultAddress);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  //Print on serial monitor
  Serial.print("Motor powered: " + smd.IsMotorPowered(SmartDrive_Motor_ID_1));

  delay(1000);
}
