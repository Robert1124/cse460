/**
 * BICOPTER with altitude and yaw control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Robot
Robot* myRobot = nullptr;

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput cmd;
ControlInput outputs;
ReceivedData rcv; 

// Data storage for the sensors 
float senses[myRobot->MAX_SENSORS];

bool updateParams = true;
const int TIME_STEP_MICRO = 4000;
float groundAltitude = 0;
int dt = 1000;
unsigned long clockTime;
unsigned long printTime;


typedef struct feedback_s {
    bool zEn, yawEn;
    float kpyaw, kdyaw, kiyaw;
    float kpz, kdz, kiz, z_int_low, z_int_high;
} feedback_t;

feedback_t PDterms;

// List of the variables that need persistent storage
float g = 9.81;
float mass = 0.012;
//float mass = 0.0811;
float frame_len = 0.12;



void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("CustomBicopter");
    paramUpdate();

    // updates the ground altitude for the ground feedback
    int numSenses = myRobot->sense(senses);
    groundAltitude = senses[1];//height
}


void loop() {

  
  // Retrieves cmd.params from ground station and checks flags
  recieveCommands();

  // Get sensor values
  int numSenses = myRobot->sense(senses);
  
  // send values to ground station
  rcv.flag = 1;
  rcv.values[0] = senses[1] - groundAltitude;  //height
  rcv.values[1] = senses[5];  //yaw
  rcv.values[2] = senses[10];  //battery
  rcv.values[3] = senses[0];  //temperature
  bool sent = baseComm->sendMeasurements(&rcv);

  // print sensor values every second
  // senses => [temperature, altitude, veloctity in altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, null, battery]
  /*if (micros() - printTime > 500000){
    for (int i = 0; i < numSenses-11; i++){
      Serial.print(senses[i]);
      Serial.print(",");
    }
    Serial.println(senses[numSenses-11]);
    printTime = micros();
  }*/

  float* controls = cmd.params;
  // When control[0] == 0, the robot stops its motors and sets servos to 90 degrees
  if (controls[0] == 0) {
    float outputs[5];
    outputs[0] = 0;
    outputs[1] = 0;
    outputs[2] = 0; // change if you are not using upwards facing motor/servos
    outputs[3] = 0; // change if you are not using upwards facing motor/servos
    outputs[4] = controls[5]; //LED controller
    
    myRobot->actuate(outputs, 5);
    fixClockRate();
    return;
    
  }

  // you can freely change how these values work from the ground station- dont feel required to use this version
  float fx = controls[1]; // Fx (foward backwards)
  float fz = controls[2]; // Fz ()
  float tx = controls[3]; // tx
  float tz = controls[4]; // tz
  float LED = controls[5]; // increase number of parameters as needed if you want more controls from ground station
  float yaw_rate_threshold = controls[6];

  float temperature = senses[0];
  float altitude = senses[1];
  float altitudeVelocity = senses[2];
  float pitch = senses[3];
  float roll = senses[4];
  float yaw = senses[5];
  float pitchrate = senses[6];
  float rollrate = senses[7];
  float yawrate = senses[8];
  float battery = senses[10];

  float z_integral = 0;
  float yaw_integral = 0;
  float angle_diff = 0;

  float desired_vz = 0.0;
  float desired_vyaw = 0.0;
  float moving_angle = 0;
  float m1 = 0;  // motor 1
  float m2 = 0;  // motor 2
  float t1 = 90;  // servo 1
  float t2 = 90;  // servo 2




  //Serial.print("Desire ANgle ");
  //Serial.println(tz); 
  // Z feedback
  // This is the solution that I use for the height feedback, you still need to convert this value into motor/servo values
  // This is an absolute height controller, which is in meters; for example if your controls
  if (PDterms.zEn) {
    z_integral += (fz - altitude) * ((float)dt)/1000000.0f * PDterms.kiz;
    z_integral = constrain(z_integral, PDterms.z_int_low, PDterms.z_int_high);
    // PID in z: the output is the force to maintain the altitude
    fz = (fz - altitude) * PDterms.kpz - altitudeVelocity * PDterms.kdz + z_integral;
    fz += mass * g;
  }

  if(PDterms.yawEn) {
    tz = radians(tz);
    //calculate angle diff
    //desire > current->positive
    //desire < current->negative
    angle_diff = atan2(sin(tz - yaw), cos(tz - yaw));
    //Integral in yas
    //yaw_integral += (angle_diff) * ((float)dt)/1000000.0f * PDterms.kiyaw;
    //yaw_integral = constrain(yaw_integral, PDterms.yaw_int_low, PDterms.yaw_int_high);
    tz = angle_diff * PDterms.kpyaw - yawrate * PDterms.kdyaw;
    //Serial.print("YAW control: ");
    //Serial.println(tz);
  }
  
  //fx = 0.0;
  Serial.print("TZ: ");
  Serial.println(tz); 
  

  //if(fz > 0){
  float i = fz/2 + tx/(2* frame_len);
  float j = fx/2 - tz/(2* frame_len);
  float m = fx/2  + tz/(2 * frame_len);
  float t = fz/2 - tx/(2 * frame_len);
  t1 = atan2(t , m) * (180 / PI);
  t2 = atan2(i , j) * (180 / PI);
  if(t1 < 0){
    t1 = 180 + t1;
  }
  if(t2 < 0){
    t2 = 180 + t2;
  }
  t2 = 180 - t2;
  m1 = sqrt(t * t + m * m);
  m2 = sqrt(i * i + j * j);

  Serial.print("t1: ");
  Serial.println(t1);
  Serial.print("t2: ");
  Serial.println(t2);


  //}
  
  // Put your controller code here which converts ground station controls and sensor feedback into motor/servo values
  // feel free to use the PDterms set from the ground station which are picked up in the paramUpdate function for easy tuning
  
  

  /******** INSERT YOUR CODE HERE: Use the variable yaw and yawrate for yaw control ******************/


  outputs.params[0] = m1;
  outputs.params[1] = m2;
  outputs.params[2] = t2;
  outputs.params[3] = t1;

  // Send command to the actuators
  myRobot->actuate( outputs.params, 5);

  // makes the clock rate of the loop consistent.
  fixClockRate();
}

void recieveCommands(){
  if (baseComm->isNewMsgCmd()){
    // New command received
    cmd = baseComm->receiveMsgCmd();
    if (int(cmd.params[11]) == 1 && updateParams){
      paramUpdate();
      updateParams = false;
    } else {
      updateParams = true;
    }
    // Print command
    //Serial.print("Cmd arrived: ");
    //printControlInput(cmd);
  }
}

void paramUpdate(){
    Preferences preferences; //initialize the preferences 
    preferences.begin("params", true); //true means read-only

    PDterms.zEn = preferences.getBool("zEn", false);
    PDterms.yawEn = preferences.getBool("yawEn", false);
    PDterms.kpz = preferences.getFloat("kpz", 0.5);
    PDterms.kdz = preferences.getFloat("kdz", 0.5);
    PDterms.kiz = preferences.getFloat("kiz", 0);
    PDterms.z_int_low = preferences.getFloat("z_int_low", 0);
    PDterms.z_int_high = preferences.getFloat("z_int_high", 0.2);
    PDterms.kpyaw = preferences.getFloat("kpyaw", 0.1);
    PDterms.kdyaw = preferences.getFloat("kdyaw", 0.1);// same thing as if I said kpyawrate
    PDterms.kiyaw = preferences.getFloat("kiyaw", 0);

    preferences.end();

    myRobot->getPreferences();
    baseComm->setMainBaseStation();
}

void fixClockRate() {

  dt = (int) (micros()-clockTime);
  while (TIME_STEP_MICRO - dt > 0){
    dt = (int) (micros()-clockTime);
  }
  clockTime = micros();
}
