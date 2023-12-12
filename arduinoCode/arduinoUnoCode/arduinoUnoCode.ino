// chopper lift stepper
# define CHOPPER_STEPPER_MIN_STEP 0
# define CHOPPER_STEPPER_MAX_STEP 5000

// gantry stepper
# define GANTRY_STEPPER_MIN_STEP 0
# define GANTRY_STEPPER_MAX_STEP 2000

// utensil shaker stepper
# define POT_STEPPER_MIN_STEP 0
# define POT_STEPPER_MAX_STEP 2000

// chopper pins
# define CHOPPER 9

// oil pump
# define OIL_PUMP 10

// water pump
# define WATER_PUMP 11

#include <ros.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

AccelStepper chopperStepper(1, 3, 4);
int chopperSteps = 0;

AccelStepper gantryStepper(1, 5, 6);
int gantrySteps = 0;

AccelStepper potStepper(1, 7, 8);
int potSteps = 0;

String deviceName;

int stepper1, stepper2 , stepper3;
int value;

void getDeviceName(String data) {
  deviceName = data.substring(0, 1);

  int stepperComma[4] = {1, 6, 11, 16};

  if (deviceName == "S") {
    stepper1 = data.substring(stepperComma[0] + 1, stepperComma[1]).toInt();
    stepper2 = data.substring(stepperComma[1] + 1, stepperComma[2]).toInt();
    stepper3 = data.substring(stepperComma[2] + 1, stepperComma[3]).toInt();
  }

  else {
    value = data.substring(2, data.length() - 1).toInt();
  }
}


void messageCb(const std_msgs::String& msg) {
  String data = msg.data;
  getDeviceName(data);

  // stepper control
  if (deviceName == "S") {
    // chooper lift stepper control
    if (stepper1 != 0) {
      if ((stepper1 >= CHOPPER_STEPPER_MIN_STEP && stepper1 <= CHOPPER_STEPPER_MAX_STEP) || (stepper1 < -CHOPPER_STEPPER_MIN_STEP && stepper1 >= -CHOPPER_STEPPER_MAX_STEP)) {
        chopperSteps = chopperStepper.currentPosition();
        if ((chopperSteps >= CHOPPER_STEPPER_MIN_STEP && chopperSteps <= CHOPPER_STEPPER_MAX_STEP) || (chopperSteps < CHOPPER_STEPPER_MIN_STEP && chopperSteps >= -CHOPPER_STEPPER_MAX_STEP)) {
          chopperStepper.moveTo(stepper1);
          chopperStepper.runToPosition();
          chopperSteps = chopperStepper.currentPosition();
        }
      }
    }

    // gantry stepper control
    if (stepper2 != 0) {
      if ((stepper2 >= GANTRY_STEPPER_MIN_STEP && stepper2 <= GANTRY_STEPPER_MAX_STEP) || (stepper2 < -GANTRY_STEPPER_MIN_STEP && stepper2 >= -GANTRY_STEPPER_MAX_STEP)) {
        gantrySteps = gantryStepper.currentPosition();
        if ((gantrySteps >= GANTRY_STEPPER_MIN_STEP && gantrySteps <= GANTRY_STEPPER_MAX_STEP) || (gantrySteps < GANTRY_STEPPER_MIN_STEP && gantrySteps >= -GANTRY_STEPPER_MAX_STEP)) {
          gantryStepper.moveTo(stepper2);
          gantryStepper.runToPosition();
          gantrySteps = gantryStepper.currentPosition();
        }
      }
    }

    // utensil shaker stepper control
    if (stepper3 != 0) {
      if ((stepper3 >= POT_STEPPER_MIN_STEP && stepper3 <= POT_STEPPER_MAX_STEP) || (stepper3 < -POT_STEPPER_MIN_STEP && stepper3 >= -POT_STEPPER_MAX_STEP)) {
        potSteps = potStepper.currentPosition();
        if ((potSteps >= POT_STEPPER_MIN_STEP && potSteps <= POT_STEPPER_MAX_STEP) || (potSteps < POT_STEPPER_MIN_STEP && potSteps >= -POT_STEPPER_MAX_STEP)) {
          potStepper.moveTo(stepper3);
          potStepper.runToPosition();
          potSteps = potStepper.currentPosition();
        }
      }
    }
  }

  else if (deviceName == "C") {
    if (value == 1) {
      digitalWrite(CHOPPER, HIGH);
      delay(2000);
      digitalWrite(CHOPPER, LOW);
    }
    else{
      digitalWrite(CHOPPER, LOW);
    }
  }

  else if (deviceName == "O") {
    if (value == 0) {
      digitalWrite(OIL_PUMP, LOW);
    }
    else{
      // calculate flow rate
      digitalWrite(OIL_PUMP, HIGH);
      //delay(flow rate);
      digitalWrite(OIL_PUMP, LOW);
    }
  }

  else if (deviceName == "W") {
    if (value == 0) {
      digitalWrite(WATER_PUMP, LOW);
    }
    else{
      // calculate flow rate
      digitalWrite(WATER_PUMP, HIGH);
      //delay(flow rate);
      digitalWrite(WATER_PUMP, LOW);
    }
  }
}

ros::Subscriber<std_msgs::String> sub("stepper_induction_command", &messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  chopperStepper.setMaxSpeed(500);
  chopperStepper.setAcceleration(1000);

  gantryStepper.setMaxSpeed(500);
  gantryStepper.setAcceleration(1000);

  potStepper.setMaxSpeed(500);
  potStepper.setAcceleration(1000);

  pinMode(CHOPPER, OUTPUT);
  pinMode(OIL_PUMP, OUTPUT);
  pinMode(WATER_PUMP, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
