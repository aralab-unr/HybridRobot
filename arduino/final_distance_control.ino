#include <ros.h>
//#include <pcd_filter/magarr_controller.h>
#include <std_srvs/SetBool.h>

#include <PID_v1.h>
#include <PinChangeInt.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <VL6180X.h>

#define RANGE 2

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x20
#define address1 0x22

#define Dir_pin1             8                      // PWM outputs to L298N H-Bridge motor driver module
#define En_pin1              9


double kp0 = 40 , ki0 = 10 , kd0 = 0;             // modify for optimal performance
double input0=0, output0=0, setpoint0=0, setpoint1=0;
long temp0;

// declare a class/function from PID library
PID myPID0(&input0, &output0, &setpoint0, kp0, ki0, kd0, DIRECT);

/* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin1 = 3;

// set up Kalman Filter for two sensor
SimpleKalmanFilter simpleKalmanFilter0(3, 3, 0.05);
SimpleKalmanFilter simpleKalmanFilter1(3, 3, 0.05);
const long SERIAL_REFRESH_TIME = 100;
//long refresh_time;

/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;

bool control_trigger = false;

ros::NodeHandle nh;

//void control_signal(const pcd_filter::magarr_controller::Request& req,
//                 pcd_filter::magarr_controller::Response& res)

void control_signal(std_srvs::SetBool::Request& req,
                 std_srvs::SetBool::Response& res)
{
  // set the desired point/distance
  setpoint0 = (double)req.data;
  Serial.print("Target distance: "); 
  Serial.println(setpoint0);
  control_trigger = true;
  res.message = "d";
  
}

ros::ServiceServer<std_srvs::SetBool::Request, 
                    std_srvs::SetBool::Response> magArr("magarr_controller", &control_signal);

// setup general configuration for both chip
void setup() 
{
  // initialize serial:
  Serial.begin(57600);
  Wire.begin();
  // Reset all connected sensors
//  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);

//  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW);
//  int_sensor0();
  int_sensor1();
  //Serial.println("Sensors ready!");
  //delay(3000);
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetSampleTime(1);
  myPID0.SetOutputLimits(-255, 255);
  
  nh.initNode();
  nh.advertiseService(magArr);
  
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



// send power to motor to rotate (2 directions)
void pwmOut0(int out) {                                // to H-Bridge board
  if (out > 0) {
    digitalWrite(Dir_pin1, 1);                             // drive motor CW
    analogWrite(En_pin1, abs(out));
  }
  else {
    digitalWrite(Dir_pin1, 0);                             // drive motor CW
    analogWrite(En_pin1, abs(out));                      // drive motor CCW
  }
}

// this loop runs forever in Arduino 
void loop() 
{
  if (control_trigger)
  {
    float real_value1 = sensor1.readRangeContinuousMillimeters();
    float estimated_value1 = simpleKalmanFilter1.updateEstimate(real_value1);

    Serial.print(" Current distance: ");
    Serial.print(estimated_value1);
    // set the current distance
    input0 = estimated_value1;
    if ((setpoint0-input0)<0.01)
    {
      control_trigger = false;
    }
    else { 
      // calculate/convert the distance to round/power to send to motor
      myPID0.Compute();   
      pwmOut0(output0);
    }
  }
  nh.spinOnce();
  delay(1);
  
}
