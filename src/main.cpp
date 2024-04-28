#include <Arduino.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AS5600.h>
#include <ESP32Servo.h>
#include <WiFi.h>


/******************Auxiliary Functions for computing the Servo Motor commands************************/
void reconnect();
void my_map(uint8_t &out, int16_t &in, int8_t in_min, int8_t in_max, uint8_t out_min, uint8_t out_max)
{
  const uint8_t run = in_max - in_min;
  const uint8_t rise = out_max - out_min;
  const uint8_t delta = in - in_min;
  out = (delta * rise) /(run + out_min);
}

class PDController
{
  private:
    float Kp, Kd, umax; // constants that define the controller dynamics
    float eprev; // previous error and integral error, utilized for the derivative and integral calculations respectively
  
  public:
  // constructor
  PDController(): Kp(2.4), Kd(0.2), umax(100), eprev(0){}

  // function to set parameters
  void setParams(float kpIn, float KdIn)
  {
    Kp = kpIn;
    Kd = KdIn;
  }
  
  void evalActVar(int16_t value, int16_t target, uint8_t &comm, unsigned long timers[3], int16_t &eprev, uint8_t dir)
  {
    // geting timer values
    timers[0] = micros(); // current time
    timers[1] = (timers[0] - timers[2]); // delta t since previos time 
    timers[2] = timers[0]; // previous time for next iteration is the current time
    int16_t comm_raw;
    // setting the target to the opposite direction of rotation
    if(dir == 1) target *= -1;
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
    // Set the direction of rotation of the motors as they are mirroed

    my_map(comm, comm_raw, -100, 100, 0, 180);
    if(comm > 90 && comm < 102 && e != 0) comm = 102;
    else if(comm < 90 && comm > 80 && e != 0) comm = 80;

    eprev = e;
  }
};

// Pins utilized for the servos
const uint8_t servo_pins[] = {18, 19};
// Array of path Parameters
uint8_t path_params[2];
int16_t PD_controller_inputs[3];
uint8_t comms[2];
int16_t errors[2];
unsigned long timers_1[3];
unsigned long timers_2[3];
String start;
// Declaring the necessary pins for the I2C and the relay control 
const uint8_t SDA_0 = 5;
const uint8_t SCL_0 = 23;
const uint8_t SDA_1 = 21;
const uint8_t SCL_1 = 17;
const uint8_t relay_pin = 22;
// boolean to set the movement of the motor after first connection
bool start_and_stop = false;
// variables to update the dashboard for the position sensors and the power voltage and current sensors
long now = millis();
long last_encoder_measure = 0;
long last_current_and_voltage_measure = 0;
// Initializing the two I2C buses that will be used for the sensors
TwoWire I2C_buses[] = {TwoWire(0), TwoWire(1)};
AS5600 encoders[] = {AS5600(&I2C_buses[0]), AS5600(&I2C_buses[1])};
// Array with the INA219 sensors
Adafruit_INA219 current_voltage_sensors[] = {Adafruit_INA219(), Adafruit_INA219()};
// Array of servo motors
Servo servo[] = {Servo(), Servo()};
// Array with the PD controllers for each motor
PDController motor_controllers[] = {PDController(), PDController()};

// Wifi name and password where the device will be conneted
const char* ssid = "";
const char* password = "";
// MQTT broker credentials
const char* MQTT_username = NULL;
const char* MQTT_password = NULL;

// mqtt server IP address (IP address of he computer)
const char* mqtt_server = "";

WiFiClient espClient;
PubSubClient client(espClient);

void runPath(uint8_t path, uint8_t side_len, Servo servos[2])
{
  int16_t angle = 0;
  const float r = 3.5;
  switch (path)
  {
  case 3:
    angle = 150; 
    break;
  case 4:
    angle = 45;
    break;
  case 5:
    angle = 36;
    break;
  case 6:
    angle = 30;
    break; 
  default:
    return;
  }
  PD_controller_inputs[0] = (int16_t)(side_len*180)/(r*PI);
  for (uint8_t i = 0; i < path*2; i++)
  {
    encoders[0].resetCumulativePosition();
    encoders[1].resetCumulativePosition();
    uint16_t counter = 0;
    while (counter < 300)
    {
      PD_controller_inputs[1] = encoders[0].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
      motor_controllers[0].evalActVar(PD_controller_inputs[1], PD_controller_inputs[0], comms[0], timers_1, errors[0], 0);
      PD_controller_inputs[2] = encoders[1].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
      motor_controllers[1].evalActVar(PD_controller_inputs[2], PD_controller_inputs[0], comms[1], timers_2, errors[1], 1);
      servos[0].write(comms[0]);
      servos[1].write(comms[1]);
      if (errors[0] < 5 && errors[1] < 5) counter++;
      else counter = 0;
    }
    counter = 0;
    encoders[0].resetCumulativePosition();
    encoders[1].resetCumulativePosition();
    while (counter < 300)
    {
      PD_controller_inputs[1] = encoders[0].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
      motor_controllers[0].evalActVar(PD_controller_inputs[1], angle, comms[0], timers_1, errors[0], 0);
      PD_controller_inputs[2] = encoders[1].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
      motor_controllers[1].evalActVar(PD_controller_inputs[2], angle, comms[1], timers_2, errors[1], 1);
      servos[0].write(comms[0]);
      servos[1].write(comms[1]);
      if (errors[0] < 5 && errors[1] < 5) counter++;
      else counter = 0;
    }
    if(!client.connected()) reconnect();
  }
}

void goToAngle(int16_t angle, Servo servos[2])
{
  uint16_t counter = 0;
  while (counter < 300)
  {
    PD_controller_inputs[1] = encoders[0].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    motor_controllers[0].evalActVar(PD_controller_inputs[1], angle, comms[0], timers_1, errors[0], 0);
    PD_controller_inputs[2] = encoders[1].getCumulativePosition() * AS5600_RAW_TO_DEGREES;
    motor_controllers[1].evalActVar(PD_controller_inputs[2], angle, comms[1], timers_2, errors[1], 1);
    servos[0].write(comms[0]);
    servos[1].write(comms[1]);
    if (errors[0] < 5 && errors[1] < 5) counter++;
    else counter = 0;
  }
  if(!client.connected()) reconnect();
}

// Function to connet the ESP32 to the router
void setupWifi()
{
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
}

// Functions that will be executed when some device publishes a message
// to a topic that the esp8266 is subscribed to.
void callback(String topic, byte* message, uint8_t length)
{
  String temp_message;
  // Checking the topic of message so the message can be converted to the correct type
  if(topic == "side_length")
  {
    int temp_length = 0;

    for (uint8_t i = 0; i < length; i++) temp_message += (char)message[i];
    
    temp_length = temp_message.toInt();
    path_params[0] = (uint8_t)temp_length;
  }
  else if (topic == "set_path")
  {
    for (uint8_t i = 0; i < length; i++) temp_message += (char)message[i];
    int set_path = temp_message.toInt();
    path_params[1] = (uint8_t) set_path;
  }
  else if(topic == "start")
  {
    for(uint8_t i = 0; i < length; i++) temp_message += (char)message[i];
    start = temp_message;
    if(start == "true")
    {
      runPath(path_params[1], path_params[0], servo);
      client.publish("reset switch", "false");
      client.publish("reset path", "None");
      start = "";
    }
  }
  else if(topic == "set_angle")
  {
    int16_t temp_angle = 0;
    for (uint8_t i = 0; i < length; i++) temp_message += (char)message[i];
    
    temp_angle = temp_message.toInt();
    PD_controller_inputs[0] = (int16_t) temp_angle; 
  }
  else if(topic == "Move")
  {
    for(uint8_t i = 0; i < length; i++) temp_message += (char)message[i];
    start = temp_message;
    if(start == "true")
    {
      goToAngle(PD_controller_inputs[0], servo);
      client.publish("reset switch", "false");
      client.publish("reset path", "None");
      start = "";
    }
  }
}

// function reconnects the ESP8266 to the MQTT broker
void reconnect()
{
  // Loop until the connection is restablashed
  while(!client.connected())
  {
    if(client.connect("espClient", MQTT_username, MQTT_password))
    {
      // subscribe or resubscribe to topics
      client.subscribe("side_length");
      client.subscribe("set_path");
      client.subscribe("start");
      client.subscribe("Move");
      client.subscribe("set_angle");
      client.subscribe("Kp");
      client.subscribe("Kd");
    }
    else
    {
      // wait 5 seconds
      delay(5000);
    }
  }
}

void setup()
{
  // Initializing wifi connection
  setupWifi();
  // Setting MQTT server port
  client.setServer(mqtt_server, 1883);
  // Setting the callback function for comminicating with the dashboard
  client.setCallback(callback);
  // Starting the I2C buses in the correct pins
  I2C_buses[0].begin(SDA_0, SCL_0);
  I2C_buses[1].begin(SDA_1, SCL_1);
  // Attaching servos to their respective pins
  servo[0].attach(18);
  servo[1].attach(19);
  // initialize the sensors
  current_voltage_sensors[0].begin(&I2C_buses[0]);
  current_voltage_sensors[1].begin(&I2C_buses[1]);
  // Initializing and seting of the position sensors
  encoders[0].begin();
  encoders[0].setDirection(AS5600_CLOCK_WISE);
  encoders[1].begin();
  encoders[1].setDirection(AS5600_COUNTERCLOCK_WISE); 
}

void loop()
{
  // loop will ensure that the board is connected to the broker
  if(!client.connected()) reconnect();
  if(!client.loop()) client.connect("espClient", MQTT_username, MQTT_password);

  // After connecting the vehicle moves a little nad then stops to indicate all is working as desired
  if(!start_and_stop)
  {
    servo[0].write(102);
    servo[1].write(80);
    delay(500);
    servo[0].write(90);
    servo[1].write(90);
    start_and_stop = true;
  }
  now = millis();
  if(now - last_encoder_measure >= 1000)
  {
    client.publish("angle", String(encoders[0].getCumulativePosition()).c_str());
    last_encoder_measure = now;
  }
  if(now - last_current_and_voltage_measure >= 10000)
  {
    client.publish("Status Monitoring/Solar Power production", String(current_voltage_sensors[1].getPower_mW()).c_str());
    client.publish("Status Monitoring/Solar Cell Voltage", String(current_voltage_sensors[1].getBusVoltage_V()).c_str());
    client.publish("Status Monitoring/Solar Cell Current", String(current_voltage_sensors[1].getCurrent_mA()).c_str());
    client.publish("Status Monitoring/Battery Status", String(current_voltage_sensors[0].getBusVoltage_V()).c_str()); 
    last_current_and_voltage_measure = now;  
  }
}
// Have to make some proibing of the used cores and the set up the second core for reading and posting the measurements from the sensors
