//    FILE: AS5600_outmode_pwm_interrupt.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.
#include<AS5600.h>
#include<Wire.h>
#include<Servo.h>

AS5600 as5600;   //  use default Wire
AS5600 as56001; // will use a different wire in the ESP32
Servo my_servo;

TwoWire Wire0 = TwoWire(0);
TwoWire Wire1 = TwoWire(1);

const uint8_t SDA0 = 34;
const uint8_t SCL0 = 36;
const uint8_t SDA1 = 17;
const uint8_t SCL1 = 16;
const uint8_t servo_pin = 12;

void setup()
{
  Serial.begin(9600);

  // Second sensor PWM set up 
  Wire0.begin(SDA0, SCL0, 100000);
  Wire1.begin(SDA1, SCL1, 100000);
  my_servo.attach(servo_pin);
}


void loop()
{
  my_servo.write(70);
  Serial.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES);   //Serial output to visualize in Serial Plotter
  Serial.print(" ");
  Serial.print(as56001.rawAngle()*AS5600_RAW_TO_DEGREES);
//  Serial.print(" ");
//  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
}
//  -- END OF FILE --
