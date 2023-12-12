// induction
# define LOCK     3
# define ON       4
# define HEAT_P   5
# define HEAT_M   6
 
// extraction fan
# define E_FAN    5

// washing pump
# define W_PUMP   6

// deliver actuator
# define DELIVERY 7

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

String deviceName;
int value;

int counter = 0;
int temp;

void getDeviceName(String data) {
  deviceName = data.substring(0, 1);
  value = data.substring(2, data.length() - 1).toInt();
}

void messageCb(const std_msgs::String& msg) {
  String data = msg.data;
  getDeviceName(data);
  
  // induction control
  if (deviceName == "I") {
    // Temperature (0-9)
    if (value >= -9 && value <= 9) {
      if (value > 0 && value <= 9) {
        temp = abs(value - counter);
        for (int i = 0; i < temp; i++) {
          digitalWrite(HEAT_P, HIGH);
          delay(100);
          digitalWrite(HEAT_P, LOW);
          delay(200);
          counter++;
        }
      }
      if ((value >= -9 && value <= 0) && counter != 0) {
        temp = counter - abs(value);
        for (int i = 0; i < temp; i++) {
          digitalWrite(HEAT_M, HIGH);
          delay(100);
          digitalWrite(HEAT_M, LOW);
          delay(200);
          counter--;
        }
      }
      value = -5;
    }
    // ON-OFF & LOCK-UNLOCK
    else if (value == 10 || value == 11) {
      switch (value) {
        case 10: // LOCK-UNLOCK
          digitalWrite(LOCK, HIGH);
          delay(2000);
          digitalWrite(LOCK, LOW);
          value = -5;
          break;

        case 11: // ON-OFF
          digitalWrite(ON, HIGH);
          delay(100);
          digitalWrite(ON, LOW);
          value = -5;
          break;
      }
    }
  }

  // extraction fan control
  else if(deviceName == "F"){
    if(value == 1){
      digitalWrite(E_FAN, HIGH);
    }
    else{
      digitalWrite(E_FAN, LOW);
    }
  }

  // washing pump control
  else if(deviceName == "P"){
    if(value == 1){
      digitalWrite(W_PUMP, HIGH);
    }
    else{
      digitalWrite(W_PUMP, LOW);
    }
  }

  // food delivery control
  else if(deviceName == "d"){
    if(value == 1){
      digitalWrite(DELIVERY, HIGH);
      delay(1000);
    }
    else{
      digitalWrite(DELIVERY, LOW);
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

  pinMode(E_FAN, OUTPUT);

  pinMode(W_PUMP, OUTPUT);

  pinMode(DELIVERY, OUTPUT);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
