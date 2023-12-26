// Base height
#include <math.h>
#include <Servo.h>
// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#include <RemoteXY.h> 
#include <Keypad.h>

#include <main.h>


// keypad setup
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
// this structure defines all the variables and events of your control interface 

struct {

    // input variables
  int8_t Base_slider; // =0..100 slider position 
  int8_t Arm1_slider; // =0..100 slider position 
  int8_t Arm2_slider; // =0..100 slider position 
  uint8_t gripper_button; // =1 if state is ON, else =0 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)
// Link lengths (replace with your specific values in cm)
float l1 = 12.0;  // Height of the first link
float l2 = 15.0;  // Height of the second link

// Base height
float base_height = 9;  // Base origin is at +5 z-axis


void forwardKinematics(int theta1, int theta2, int theta3, float &x, float &y, float &z) {
  x = l1 * cos(theta1) + l2 * cos(theta1 + theta2) + theta3;
  y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
  z = base_height;  // Base height
}

void inverseKinematics(int x, int y, int z, int &theta1, int &theta2, int &theta3) {
  theta3 = z - base_height;  // Third link is directly related to z

  float c = sqrt(x*x + y*y);
  theta1 = atan2(y, x);

  float alpha = atan2(z - base_height, c);
  float beta = acos((l1*l1 + c*c - l2*l2) / (2*l1*c));
  
  theta2 = M_PI - beta;  // Second link angle
}

void initial_position() {
  servo1.write(0);
  servo2.write(45);
  servo3.write(0);
  release();
}

// go to position with theta values
void go(u8 &theta1, u8 &theta2, u8 &theta3) {
  servo1.write(theta1);
  delay(500);
  servo2.write(theta2);
  delay(500);
  servo3.write(theta3);
  delay(500);

}
void move_to_box(u8 theta1, u8 theta2, u8 theta3) {
  inverseKinematics(10, 10, 10, theta1, theta2, theta3);
  go(theta1, theta2, theta3);
}

void grab() {
  if(IRSensor > 150){
    // Set gripper to closed position
    servo4.write(0);
  }else{
    initial_position();
  }
}

void release() {
  servo4.write(75);  // Set gripper to open position
}


void pick(float xi, float yi, float zi, float xf, float yf, float zf) {
  initial_position();
  inverseKinematics(xi, yi, zi, theta1, theta2, theta3);
  grab();
  go(theta1, theta2, theta3);
  move_to_box(theta1, theta2, theta3);
}

void setup() {
  Serial.begin(9600);
  u16 IRsensor = analogRead(IRSensor);
  // Attach servos to pins
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  RemoteXY_Init (); 
}

void loop() {
  // Loop can remain empty as our operations are done in the pick function
  RemoteXY_Handler ();
   char key = keypad.getKey();
  if (key) {
    if (key == '0') {
      controlState = 0;
    } else if (key == '1') {
      controlState = 1;
    }
  }
 if (controlState == 1) {
   //CONTROL STATE = 1
    float theta1 = map(RemoteXY.Base_slider, 0, 100, 0, 180);
    float theta2 = map(RemoteXY.Arm1_slider, 0, 100, 0, 180);
    float theta3 = map(RemoteXY.Arm2_slider, 0, 100, 0, 180);
    if (RemoteXY.gripper_button == 1) {
      grab();
    } else {
      release();
    
    }

    servo1.write(theta1);
    servo2.write(theta2);
    servo3.write(theta3);
    
    if (RemoteXY.gripper_button == 1) {
      grab();
    } else {
      release();
    }

  } else {
   //CONTROL STATE = 0 
    initial_position();
    grab();
  }
  
}


