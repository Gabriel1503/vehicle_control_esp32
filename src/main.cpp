#include<Arduino.h>
#include<AS5600.h>
#include<Wire.h>
#include<ESP32Servo.h>


/*******************************************Auxiliary function prototypes****************************************/
void readAngleOnSerial(int16_t &set_point);
void my_map(uint8_t &out, int16_t &in, int8_t in_min, int8_t in_max, uint8_t out_min, uint8_t out_max);
/****************************************************************************************************************/

class PIDController
{
  private:
    float Kp, Kd, Ki, umax; // constants that define the controller dynamics
    float eprev, eintegral; // previous error and integral error, utilized for the derivative and integral calculations respectively
  
  public:
  // constructor
  PIDController(): Kp(2.4), Kd(0.2), umax(100){}

  // function to set parameters
  void setParams(float kpIn, float KdIn)
  {
    Kp = kpIn;
    Kd = KdIn;
  }
  
  void evalActVar(int16_t value, int16_t target, uint8_t &comm, unsigned long timers[3])
  {
    // geting timer values
    timers[0] = micros(); // current time
    timers[1] = (timers[0] - timers[2]); // delta t since previos time 
    timers[2] = timers[0]; // previous time for next iteration is the current time
    int16_t comm_raw;
    // error of the control value
    int16_t e = target - value;
    
    // derivative of the error
    float dedt = (e - eprev)/(timers[1]/1.0e6);

    // control signal
    float u = Kp*e + Kd*dedt;

    // commad for the motor
    comm_raw = (int) u;
    if(comm_raw > umax) comm_raw = umax;
    if(comm_raw < -umax) comm_raw = -1*umax;
    
    my_map(comm, comm_raw, -100, 100, 0, 180);
    if(comm > 90 && comm < 102 && e != 0) comm = 102;
    else if(comm < 90 && comm > 80 && e != 0) comm = 80;

    eprev = e;
  }
};


// variables for the connection interface of the sensors
const uint8_t SDA_0 = 25;
const uint8_t SCL_0 = 26;
const uint8_t SDA_1 = 17;
const uint8_t SCL_1 = 16;
const uint8_t servo_pin = 12;
uint8_t comm;
// variables used for angle control
int16_t set_point;
int16_t angle_encoder_1; // These 3 variables will be packed in an array later
int16_t angle_encoder_2;
unsigned long timers[3];
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
PIDController pid_motor_2; // the PID controllers will be placed in an array

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
  // read angle and set point from the Serial port
  angle_encoder_1 = as5600.getCumulativePosition() * AS5600_RAW_TO_DEGREES;   //Serial output to visualize in Serial Plotter
  readAngleOnSerial(set_point);

  pid_motor_1.evalActVar(angle_encoder_1, set_point, comm, timers);
  my_servo.write(comm);
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

void my_map(uint8_t &out, int16_t &in, int8_t in_min, int8_t in_max, uint8_t out_min, uint8_t out_max)
{
  const uint8_t run = in_max - in_min;
  const uint8_t rise = out_max - out_min;
  const uint8_t delta = in - in_min;
  out = (delta * rise) /(run + out_min);
}