/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/Barometer.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// MAC of the base station
uint8_t base_mac[6] = {0xC0, 0x49, 0xEF, 0xE3, 0x34, 0x78};  // fixme load this from memory


Barometer baro;

const float TIME_STEP = .004;
// Robot
Robot* myRobot = nullptr;
// Communication
BaseCommunicator* baseComm = nullptr;
// Control input from base station
ControlInput cmd;
ReceivedData rcv; 

float estimatedZ = 0;
float startHeight = 0;
float estimatedVZ = 0;
float kpz = 0;
float kdz = 0;

Preferences preferences; //initialize the preferences 
bool updateParams = true;

void setup() {
    Serial.begin(115200);
    Serial.println("Start!");

    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("RawBicopter");

    // Start sensor
    baro.init();
    baro.updateBarometer();
    estimatedZ = baro.getEstimatedZ();
    startHeight = baro.getEstimatedZ();
    estimatedVZ = baro.getVelocityZ();
    paramUpdate();
}


void loop() {

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
      Serial.print("Cmd arrived: ");
      printControlInput(cmd);
    }

    // Update measurements
    if (baro.updateBarometer()){
      // sense 
      float height = baro.getEstimatedZ() - startHeight;
      float height_velocity = baro.getVelocityZ();
      // estimate
      estimatedZ = estimatedZ * .6 + height * .4;
      estimatedVZ = estimatedVZ * .9 + height_velocity * .1;
      
      // Serial.print(estimatedZ);
      // Serial.print(", ");
      // Serial.println(estimatedVZ);
      rcv.flag = 1;
      rcv.values[0] = estimatedZ;
      rcv.values[1] = estimatedVZ;
      bool sent = baseComm->sendMeasurements(&rcv);
      // Serial.println(sent);
    }


    /**
     * Begin of Controller for Height:
     * Create your height PID to control m1 and m2 here.
     * kpz and kdz are changed from the ground station and are floats declared at the top
     */
    
    float desired_height = cmd.params[5];
    //Serial.println(String("Height: ") + desired_height);
    Serial.println(String("Current Height") + estimatedZ);
    float m1 = 0;  // YOUR INPUT FOR THE MOTOR 1 HERE
    float m2 = 0;  // YOUR INPUT FOR THE MOTOR 1 HERE


    //P Control
    //float control_input = P_control(desired_height, estimatedZ, kpz);

    
    //P Control
    //float control_input = P_control(desired_height, estimatedZ, kpz);

    //PD Control
    if(desired_height > 0){
      float control_input = PD_control(desired_height, estimatedZ, estimatedVZ, kpz, kdz);
      m1 = control_input;
      m2 = control_input;
    } else {
      m1 = 0;
      m2 = 0;
    }
    /**
     * End of controller
     */
    // Control input
    ControlInput actuate;
    actuate.params[0] = m1; // Motor 1
    actuate.params[1] = m2; // Motor 2
    // servo control
    actuate.params[2] = cmd.params[2]; // Servo 1
    actuate.params[3] = cmd.params[3]; // Servo 2
    actuate.params[4] = cmd.params[4]; //led
    // Send command to the actuators
    myRobot->actuate(actuate.params, 5);

    sleep(TIME_STEP);
}

float P_control(float desired_height, float estimatedZ, float kpz){
  float control_input = kpz * (desired_height - estimatedZ);
    if(control_input < 0){
      control_input = 0;
    }
    return control_input;
}


float PD_control(float desired_height, float estimatedZ, float estimatedVZ,float kpz,float kdz){
  float desiredVZ = 0.0;
  float control_input = kpz*(desired_height - estimatedZ) + kdz * (desiredVZ - estimatedVZ);
  if(control_input < 0){
      control_input = 0;
  } 
  return control_input;
}

void paramUpdate(){
    preferences.begin("params", true); //true means read-only

    kpz = preferences.getFloat("kpz", .2); //(value is an int) (default_value is manually set)
    kdz = preferences.getFloat("kdz", 0); //(value is an int) (default_value is manually set)
    baseComm->setMainBaseStation();
    

    Serial.print("Update Paramters!");
    Serial.print(kpz);
    Serial.print(", ");
    Serial.println(kdz);
    preferences.end();
}