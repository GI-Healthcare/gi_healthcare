// Chopper code

# define MIN_STEP 0
# define MAX_STEP 5000

#include <ros.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>

//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;

// mode, pulse, dir
AccelStepper stepper(1, 7, 6);

// data incoming format
//device   stepper1    stepper2     stepper3
// S/I    , 0 0 0 0   , 0 0 0 0    ,  0  0  0  0  ,
// 0      1 2 3 4 5   6 7 8 9 10   11 12 13 14 15 16

int comma[4] = {1, 6, 11, 16};
String device; //S=Stepper, I=Induction
int stepper1, stepper2 ,stepper3; 

int steps = 0;

void splitData(String data){
  device = data.substring(0,comma[0]);
  stepper1 = data.substring(comma[0]+1,comma[1]).toInt();
  stepper2 = data.substring(comma[1]+1,comma[2]).toInt();
  stepper3 = data.substring(comma[2]+1,comma[3]).toInt();
}

void messageCb(const std_msgs::String& msg) {
  String data = msg.data;
  splitData(data);

  int stepperID = stepper1;
  
  // Validation which stepper motor to move
  if (device == "S" && stepperID!=0) {

    // validation for stepper limit
    if ((stepperID >= MIN_STEP && stepperID <= MAX_STEP) || (stepperID < -MIN_STEP && stepperID >= -MAX_STEP)) {
      steps = stepper.currentPosition();

      if ((steps >= MIN_STEP && steps <= MAX_STEP) || (steps < MIN_STEP && steps >= -MAX_STEP)) {
        stepper.moveTo(stepperID);
        stepper.runToPosition();
        steps = stepper.currentPosition();
      }
    }
  }
}

ros::Subscriber<std_msgs::String> sub("stepper_induction_command", &messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(3000);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
