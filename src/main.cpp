#include<Arduino.h>
#include<AS5600.h>
#include<Wire.h>
#include<ESP32Servo.h>

const uint8_t SDA_0 = 25;
const uint8_t SCL_0 = 26;
const uint8_t SDA_1 = 17;
const uint8_t SCL_1 = 16;
const uint8_t servo_pin = 12;

TwoWire my_Wire0 = TwoWire(0);
TwoWire my_Wire1 = TwoWire(1);

AS5600 as5600(&my_Wire0);   //  use default Wire
AS5600 as56001(&my_Wire1); // will use a different wire in the ESP32
Servo my_servo;

void setup()
{
  Serial.begin(9600);
  // Second sensor PWM set up 
  // Wire.begin();
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	my_servo.setPeriodHertz(50);    
	my_servo.attach(servo_pin, 1000, 2000);

  // Second sensor PWM set up 
  my_Wire0.begin(SDA_0, SCL_0, 100000);
  my_Wire1.begin(SDA_1, SCL_1, 100000);
}


void loop()
{
  my_servo.write(180);
  Serial.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES);   //Serial output to visualize in Serial Plotter
  Serial.print(" ");
  Serial.println(as56001.rawAngle()*AS5600_RAW_TO_DEGREES);
//  Serial.print(" ");
//  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
}
//  -- END OF FILE --
