// Base height
#include <math.h>
#include <Servo.h>
// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#include <RemoteXY.h> 
#include <Keypad.h>
// RemoteXY connection settings 
#define REMOTEXY_SERIAL_RX 52
#define REMOTEXY_SERIAL_TX 53
#define REMOTEXY_SERIAL_SPEED 9600
bool controlState = 0;
const byte ROWS = 4; 
const byte COLS = 3; 
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {12, 11, 10, 9}; 
byte colPins[COLS] = {8,7,6};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 51 bytes
  { 255,4,0,0,0,44,0,16,200,1,4,0,13,15,5,63,2,26,4,0,
  7,15,5,63,2,26,4,0,19,15,5,63,2,26,10,48,39,39,15,15,
  4,26,31,79,78,0,31,79,70,70,0 };
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

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void forwardKinematics(float theta1, float theta2, float theta3, float &x, float &y, float &z) {
  x = l1 * cos(theta1) + l2 * cos(theta1 + theta2) + theta3;
  y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
  z = base_height;  // Base height
}

void inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3) {
  theta3 = z - base_height;  // Third link is directly related to z

  float c = sqrt(x*x + y*y);
  theta1 = atan2(y, x);

  float alpha = atan2(z - base_height, c);
  float beta = acos((l1*l1 + c*c - l2*l2) / (2*l1*c));
  
  theta2 = M_PI - beta;  // Second link angle
}

void grab() {
  servo4.write(9);  // Set gripper to closed position
}

void release() {
  servo4.write(135);  // Set gripper to open position
}

void pick(float xi, float yi, float zi, float xf, float yf, float zf) {
  // Initial Position
  float theta1_i, theta2_i, theta3_i;
  inverseKinematics(xi, yi, zi, theta1_i, theta2_i, theta3_i);
  
  // Final Position
  float theta1_f, theta2_f, theta3_f;
  inverseKinematics(xf, yf, zf, theta1_f, theta2_f, theta3_f);
  
  // Set initial position
  servo1.write(theta1_i * 180.0 / M_PI);
  servo2.write(theta2_i * 180.0 / M_PI);
  servo3.write(theta3_i * 180.0 / M_PI);
  delay(1000);
 // Grab object
  grab();
  delay(2000);  // Wait for 2 seconds

  // Set final position
  servo1.write(theta1_f * 180.0 / M_PI);
  servo2.write(theta2_f * 180.0 / M_PI);
  servo3.write(theta3_f * 180.0 / M_PI);

  // Release object
  release();
}

void setup() {
  Serial.begin(9600);

  // Attach servos to pins
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);  // Assuming servo3 controls the third link
  servo4.attach(5);
  RemoteXY_Init (); 
  // TODO you setup code
  

  // Example: pick an object from (5, 5, 5) and place it at (-5, -5, -5)

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
    servo1.write(0);
    servo2.write(45);
    servo3.write(0);
    grab();

    // If controlState is not 1, do nothing
    // Optionally, you can add some code here if you want to do something specific when controlState is 0
  }
  
  // ... [Your other loop code]
}