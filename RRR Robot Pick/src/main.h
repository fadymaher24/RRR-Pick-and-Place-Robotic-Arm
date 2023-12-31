#ifndef MAIN.H
#define MAIN.H


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


int IRSensor = A0; // connect ir sensor module to Arduino pin 9


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 51 bytes
  { 255,4,0,0,0,44,0,16,200,1,4,0,13,15,5,63,2,26,4,0,
  7,15,5,63,2,26,4,0,19,15,5,63,2,26,10,48,39,39,15,15,
  4,26,31,79,78,0,31,79,70,70,0 };

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


// init theta values
u8 theta1 = 0;
u8 theta2 = 0;
u8 theta3 = 0;



void forwardKinematics(u8 theta1, u8 theta2, u8 theta3, u8 &x, u8 &y, u8 &z);
void inverseKinematics(float x, float y, float z, u8 &theta1, u8 &theta2, u8 &theta3);
void initial_position(void);
void move_to_box(u8 theta1, u8 theta2, u8 theta3);
void grab(void);
void release(void);
void pick(float xi, float yi, float zi, float xf, float yf, float zf);
void go(u8 &theta1, u8 &theta2, u8 &theta3);
void setup(void);
void loop(void);
#endif