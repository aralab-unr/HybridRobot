

#include <PID_v1.h>
#include <PinChangeInt.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <VL6180X.h>

#define RANGE 2

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x20
#define address1 0x22

#define Dir_pin0             8                      // PWM outputs to L298N H-Bridge motor driver module
#define En_pin0              9


double kp0 = 40 , ki0 = 10 , kd0 = 0;             // modify for optimal performance
double input0 = 0, output0 = 0, setpoint0 = 0;
long temp0;

// declare a class/function from PID library
PID myPID0(&input0, &output0, &setpoint0, kp0, ki0, kd0, DIRECT);

/* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = 2;
int enablePin1 = 3;

// set up Kalman Filter for two sensor
SimpleKalmanFilter simpleKalmanFilter0(3, 3, 0.05);
SimpleKalmanFilter simpleKalmanFilter1(3, 3, 0.05);
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;
int data_control= 70;
#define nanglen 70
#define giu 60
#define haxuong 58

float estimated_value;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// setup general configuration for both chip
void setup() 
{
  // initialize serial:
  Serial.begin(57600);
  Wire.begin();
  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);

  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW);
  int_sensor0();
  int_sensor1();
  //Serial.println("Sensors ready!");
  //delay(3000);
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetSampleTime(1);
  myPID0.SetOutputLimits(-255, 255);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

// this function set parameters for sensor 0
void int_sensor0()
{
  // Sensor0
  Serial.println("Start Sensor 0");
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
  Serial.println(sensor0.readReg(0x212),HEX); // read I2C address
  sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor0.setTimeout(500);
  sensor0.stopContinuous();
  sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(50);
  sensor0.startInterleavedContinuous(100);
  delay(50);
}

// this function set parameters for sensor 1
void int_sensor1()
{
  // Sensor1
  Serial.println("Start Sensor 1");
  digitalWrite(enablePin1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address1);
  Serial.println(sensor1.readReg(0x212),HEX);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(50);
  sensor1.startInterleavedContinuous(100);
  delay(50);
}

bool good=false;

// this loop runs forever in Arduino 
void loop() 
{
  // print the string when a newline arrives:
  if (stringComplete) 
  {
    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    Serial.println(data_control);

  }
  pwmOut0(data_control);

  // get two distance values from two sensors
  float real_value0 = sensor0.readRangeContinuousMillimeters();
  float estimated_value0 = simpleKalmanFilter0.updateEstimate(real_value0);
  float real_value1 = sensor1.readRangeContinuousMillimeters();
  float estimated_value1 = simpleKalmanFilter1.updateEstimate(real_value1);
  // take an average value to control
  estimated_value = (estimated_value0 + estimated_value1)/2;

  // set the starting point/distance
  setpoint0 = data_control;
  Serial.print(estimated_value0);
  Serial.print("   "); 
  Serial.println(data_control);
  // set the current distance
  input0 = estimated_value1;
  
  // calculate/convert the distance to round/power to send to motor
  myPID0.Compute();   
  // calculate new output
  //if(abs(estimated_value-data_control)<=1) pwmOut0(0);
  //else pwmOut0(output0);
  pwmOut0(output0);
}

// send power to motor to rotate (2 directions)
void pwmOut0(int out0) {                                // to H-Bridge board
  if (out0 > 0) {
    // Dir_pin0 = pin 8
    digitalWrite(Dir_pin0, 0);                             // drive motor CW
    // En_pin0 = pin 9
    analogWrite(En_pin0, abs(out0));
  }
  else {
    digitalWrite(Dir_pin0, 1);                             // drive motor CW
    analogWrite(En_pin0, abs(out0));                      // drive motor CCW
  }
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
 routine is run between each time loop() runs, so using delay inside loop can
 delay response. Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    //Serial.println(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '?') {
      int i = inputString.length(); 
      char data[i]; 
      strcpy(data, inputString.c_str());
      if(data[0]=='M'){
        data_control=(float)atoi(&data[1]);
        //Serial.println("SSS");
      }
      stringComplete = true;
    }
  }
}
