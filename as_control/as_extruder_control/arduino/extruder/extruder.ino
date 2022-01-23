#include <ros.h>

#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

const int min_pulse=400;
const int max_pulse=2000;
volatile int pulse_per_second;

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
ros::NodeHandle nh;

void pulseMessageCb( const std_msgs::Int32& msg){
  if (msg.data>0){
    pulse_per_second=-min(max(abs(msg.data),min_pulse),max_pulse);
  }else if (msg.data<0)
  {
    pulse_per_second=min(max(abs(msg.data),min_pulse),max_pulse);
  }else{
    pulse_per_second=0;
  } 
  
}

ros::Subscriber<std_msgs::Int32> sub_pulse_rate("as_extruder/set_pulse_rate", &pulseMessageCb );

void setup() {
  stepper.setMinPulseWidth(20);
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(200);
  nh.initNode();  
  nh.subscribe(sub_pulse_rate);
  pulse_per_second=1000;
}
void loop() {  
  stepper.setSpeed(pulse_per_second);
  stepper.runSpeed();
  nh.spinOnce();
}