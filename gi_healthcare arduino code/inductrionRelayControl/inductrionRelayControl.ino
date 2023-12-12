// induction code

#define LOCK   3
#define ON     4
#define HEAT_P 5
#define HEAT_M 6

#include <ros.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>

//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;

// mode, pulse, dir
AccelStepper stepper(1, 7, 6);

// data incoming format
// device     [lock/unlock | on/off | +temp | -temp]
// S/I     ,    0   0    ,
//  0      1    2   3    4    

int comma[2] = {1, 4};
String device; //S=Stepper, I=Induction
int lock_unlock, on_off , temp_p, temp_m;

int choice = -5;
int counter = 0;
int temp;

void splitData(String data) {
  device = data.substring(0, comma[0]);
  choice = data.substring(comma[0] + 1, comma[1]).toInt();
}

void messageCb(const std_msgs::String& msg) {
  String data = msg.data;
  splitData(data);
  if (device == "I") {
    // Temperature (0-9)
    if (choice >= -9 && choice <= 9) {
      if (choice > 0 && choice <= 9) {
        temp = abs(choice - counter);
        for (int i = 0; i < temp; i++) {
          digitalWrite(HEAT_P, HIGH);
          delay(100);
          digitalWrite(HEAT_P, LOW);
          delay(200);
          counter++;
        }
      }

      if ((choice >= -9 && choice <= 0) && counter != 0) {
        temp = counter - abs(choice);
        for (int i = 0; i < temp; i++) {
          digitalWrite(HEAT_M, HIGH);
          delay(100);
          digitalWrite(HEAT_M, LOW);
          delay(200);
          counter--;
        }
      }
      choice = -5;
    }

    // ON-OFF & LOCK-UNLOCK
    else if (choice == 10 || choice == 11) {
      switch (choice) {
        case 10: // LOCK-UNLOCK
          digitalWrite(LOCK, HIGH);
          delay(2000);
          digitalWrite(LOCK, LOW);
          choice = -5;
          break;

        case 11: // ON-OFF
          digitalWrite(ON, HIGH);
          delay(100);
          digitalWrite(ON, LOW);
          choice = -5;
          break;
      }
    }
  }
}

ros::Subscriber<std_msgs::String> sub("stepper_induction_command", &messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  pinMode(LOCK, OUTPUT);
  pinMode(ON, OUTPUT);
  pinMode(HEAT_P, OUTPUT);
  pinMode(HEAT_M, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
