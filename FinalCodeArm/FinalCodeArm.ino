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
int irPin = A0;
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
  // Transformation matrices
  float T1[4][4] = {
    {cos(theta1), -sin(theta1), 0, 0},
    {sin(theta1), cos(theta1), 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  float T2[4][4] = {
    {1, 0, 0, 2},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  float T3[4][4] = {
    {1, 0, 0, 12},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  // Compute end-effector position
  float end_effector[4][1] = { {0}, {0}, {0}, {1} };
  float temp[4][1];

  // Multiply T1 with end_effector
  matrixMultiply(T1, end_effector, temp);
  // Multiply T2 with temp to get new end_effector
  matrixMultiply(T2, temp, end_effector);
  // Multiply T3 with end_effector to get final position
  matrixMultiply(T3, end_effector, temp);

  x = temp[0][0];
  y = temp[1][0];
  z = temp[2][0]; // Add base height
}

// Define matrix multiplication function (simplified for 4x4 matrices)
void matrixMultiply(float A[][4], float B[][1], float result[][1]) {
  // Compute matrix multiplication and store result in 'result' matrix
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
  servo4.write(50);  // Set gripper to open position
}

void pick(float xi, float yi, float zi, float xf, float yf, float zf) {
  // Initial Position
  
  float theta1_i, theta2_i, theta3_i;
  inverseKinematics(xi, yi, zi, theta1_i, theta2_i, theta3_i);
  
  // Final Position
  float theta1_f, theta2_f, theta3_f;
  inverseKinematics(xf, yf, zf, theta1_f, theta2_f, theta3_f);
  
  // Set initial position
  servo1.write(theta1_i);
  delay(500);
  servo2.write(theta2_i);
  delay(500);
  servo3.write(theta3_i);
  delay(1500);
 // Grab object
  if (irValue >110){
  grab();
  delay(2000);  // Wait for 2 seconds
  // Set final position
  servo1.write(theta1_f);
  delay(500);
  servo2.write(theta2_f);
  delay(500);
  servo3.write(theta3_f);
  delay(1500);
  // Release object
  release();
  }else{
    return
  }

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
  irValue = analogRead(irPin);
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
    delay(500);
    float theta2 = map(RemoteXY.Arm1_slider, 0, 100, 0, 180);
    delay(500);
    float theta3 = map(RemoteXY.Arm2_slider, 0, 100, 0, 180);
    delay(500);
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
  servo2.write(90);
  servo3.write(90);
  servo4.write(50);
    // If controlState is not 1, do nothing
    // Optionally, you can add some code here if you want to do something specific when controlState is 0
  }
  
  // ... [Your other loop code]
}