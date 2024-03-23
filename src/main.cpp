#include<Arduino.h>
#include<AS5600.h>
#include<Wire.h>
#include<ESP32Servo.h>

class PIDController
{
  private:
    float kp, kd, ki, umax; // constants that define the controller dynamics
    float eprev, eintegral; // previous error and integral error, utilized for the derivative and integral calculations respectively
  
  public:
  // constructor
  PIDController(): kp(1), kd(0), ki(0), umax(100){}

  // function to set parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn)
  {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }
  
  void showParameters()
  {
    Serial.println(kp);
    Serial.println(kd);
    Serial.println(ki);
    Serial.println(umax);
  }

  void evalActVar(int16_t value, int16_t target, int8_t &comm, float deltaT)
  {
    // error of the control value
    int16_t e = target - value;
    
    // derivative of the error
    float dedt = (e - eprev)/(deltaT);

    // integral of the error
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // commad for the motor
    comm = (int) fabs(u);
    if(comm > umax) comm = umax;
    if(comm < (umax * -1)) comm = -1*umax;
    
    if(e == 0) Serial.println(value);

    eprev = e;
  }
};

/*******************************************Auxiliary function prototypes****************************************/
void readAngleOnSerial(int16_t &set_point);
void my_map(uint8_t &out, int8_t &in, int8_t in_min, int8_t in_max, uint8_t out_min, uint8_t out_max);
/****************************************************************************************************************/

// variables for the connection interface of the sensors
const uint8_t SDA_0 = 25;
const uint8_t SCL_0 = 26;
const uint8_t SDA_1 = 17;
const uint8_t SCL_1 = 16;
const uint8_t servo_pin = 12;
// variables used for angle control
int16_t set_point;
int16_t angle_encoder_1;
int16_t angle_encoder_2;
float prevT;
float deltaT;
long currT;
int8_t comm;
uint8_t servo_comm;
String in_string = "";

// Initialization of the two wire objects for the ESP32
TwoWire my_Wire0 = TwoWire(0);
TwoWire my_Wire1 = TwoWire(1);
// Initialize the sensors
AS5600 as5600(&my_Wire0);   //  use default Wire
AS5600 as56001(&my_Wire1); // will use a different wire in the ESP32
// Initialize the servo object for test
Servo my_servo;
// Initialize the PID controllers for each motor
PIDController pid_motor_1;
PIDController pid_motor_2;

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
  // geting time stamp
  currT = micros();
  deltaT = (currT - prevT)/1.0e6;  
  // read angle and set point from the Serial port
  angle_encoder_1 = as5600.rawAngle()*AS5600_RAW_TO_DEGREES;   //Serial output to visualize in Serial Plotter
  readAngleOnSerial(set_point);
  pid_motor_1.evalActVar(angle_encoder_1, set_point, comm, deltaT);

  my_map(servo_comm, comm, -100, 100, 0, 180);
  my_servo.write(servo_comm);
  // Serial.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES);   //Serial output to visualize in Serial Plotter
  // Serial.print(" ");
  // Serial.println(as56001.rawAngle()*AS5600_RAW_TO_DEGREES);
//  Serial.print(" ");
//  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
}


void readAngleOnSerial(int16_t &set_point)
{
  while (Serial.available() > 0) 
  {
    int in_char = Serial.read();
    if (isDigit(in_char) || in_char == '-')
    {
      // convert the incoming byte to a char and add it to the string:
      in_string += (char)in_char;
    }
    // if you get a newline, print the string, then the string's value:
    if (in_char == '\n')
    {
      set_point = in_string.toInt(); 
      Serial.println(set_point);
      in_string = "";
      // clear the string for new input:
    }
  }
}

void my_map(uint8_t &out, int8_t &in, int8_t in_min, int8_t in_max, uint8_t out_min, uint8_t out_max)
{
  const uint8_t run = in_max - in_min;
  const uint8_t rise = out_max - out_min;
  const uint8_t delta = in - in_min;
  out = (delta * rise) /(run + out_min);
}