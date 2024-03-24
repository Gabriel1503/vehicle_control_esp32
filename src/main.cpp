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
  PIDController(): kp(5), kd(0), ki(0), umax(100){}

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
uint8_t quadrant_number;
uint8_t previous_quadrant_number;
uint8_t number_of_turns;
int32_t total_angle1;
int32_t input;
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
  as5600.resetCumulativePosition();

}


void loop()
{
  // geting time stamp
  currT = micros();
  deltaT = (currT - prevT)/1.0e6;  
  prevT = currT;
  // read angle and set point from the Serial port
  angle_encoder_1 = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;   //Serial output to visualize in Serial Plotter
  readAngleOnSerial(set_point);
  // Quadrant 1
  if (angle_encoder_1 >= 0 && angle_encoder_1 <= 90)
  {
    quadrant_number = 1;
  }
  // Quadrant 2
  if (angle_encoder_1 >= 90 && angle_encoder_1 <= 180) 
  {
    quadrant_number = 2;
  }
  // Quadrant 3
  if (angle_encoder_1 >= 180 && angle_encoder_1 <= 270) 
  {
    quadrant_number = 3;
  }
  // Quadrant 4
  if (angle_encoder_1 >= 270 && angle_encoder_1 <= 360) 
  {
     quadrant_number = 4;
  }

  if (quadrant_number != previous_quadrant_number)
  {
    // Transition from 4th to 1st quadrant
    if (quadrant_number == 1 && previous_quadrant_number == 4)
    {
      number_of_turns++;
    }
    // Transition from 1st to 4th quadrant
    if (quadrant_number == 4 && previous_quadrant_number == 1)
    {
      number_of_turns--;
    }
    previous_quadrant_number = quadrant_number;
  }
  if (total_angle1 >= 0)
  {
    total_angle1 = (number_of_turns * 360) + angle_encoder_1;
  }
  else
  {
    total_angle1 = (number_of_turns * 360) + angle_encoder_1;
  }

  // Establish Input value for PID
  input = total_angle1;

  pid_motor_1.evalActVar(input, set_point, comm, deltaT);
  my_map(servo_comm, comm, -100, 100, 0, 180);
  my_servo.write(servo_comm);
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