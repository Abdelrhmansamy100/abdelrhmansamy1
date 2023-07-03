
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 199 bytes
  { 255, 7, 0, 11, 0, 192, 0, 16, 31, 1, 130, 0, 0, 23, 63, 19, 180, 130, 0, 22,
    0, 19, 68, 80, 130, 0, 0, 68, 63, 16, 13, 130, 0, 0, 85, 63, 15, 29, 1, 0,
    23, 5, 17, 17, 119, 31, 70, 87, 68, 0, 1, 0, 3, 24, 17, 17, 28, 31, 76, 69,
    70, 84, 0, 1, 0, 43, 24, 17, 17, 28, 31, 82, 73, 71, 72, 84, 0, 1, 0, 23,
    45, 17, 17, 1, 31, 66, 87, 68, 0, 4, 128, 4, 90, 56, 7, 2, 26, 67, 5, 22,
    34, 19, 8, 2, 26, 11, 129, 0, 23, 27, 17, 5, 25, 83, 80, 69, 69, 68, 0, 4,
    176, 1, 75, 62, 6, 2, 26, 129, 0, 22, 87, 17, 5, 25, 83, 80, 69, 69, 68, 0,
    129, 0, 15, 70, 33, 5, 25, 83, 101, 114, 118, 111, 32, 67, 111, 110, 116, 114, 111, 108,
    0, 129, 0, 50, 71, 9, 4, 8, 62, 62, 62, 62, 0, 129, 0, 4, 71, 9, 4, 8,
    60, 60, 60, 60, 0, 10, 48, 6, 8, 11, 11, 4, 26, 31, 49, 0, 31, 50, 0 };

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  uint8_t button_Forward;   // =1 if button pressed, else =0
  uint8_t button_Left;      // =1 if button pressed, else =0
  uint8_t button_Right;     // =1 if button pressed, else =0
  uint8_t button_Backward;  // =1 if button pressed, else =0
  int8_t slider_1;          // =0..100 slider position
  int8_t slider_2;          // =-100..100 slider position
  uint8_t pushSwitch_1;     // =1 if state is ON, else =0

  // output variables
  char text_Speed[11];  // string UTF8 end zero

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
#include <Servo.h>    //Servo motor library. This is standard library
#include <NewPing.h>  //Ultrasonic sensor function library. You must install this library
const int LeftMotorForward = 9;
const int LeftMotorBackward = 8;
const int RightMotorForward = 7;
const int RightMotorBackward = 6;
#define enB 5        //Enable2 L298 Pin enB
#define enA 12       //Enable1 L298 Pin enA
#define trig_pin A1  //analog input 1
#define echo_pin A2  //analog input 2

#define maximum_distance 200

boolean goesForward = false;

int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance);  //sensor function
Servo servo_motor;                                    //our servo name
void setup() {
  RemoteXY_Init();
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  analogWrite(enA, 150);   // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed
  analogWrite(enB, 150);   // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
  servo_motor.attach(A5);  //our servo pin

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  RemoteXY_Handler();
  if (RemoteXY.pushSwitch_1 == 1) {
    CIR();
  } else {
    Bluetoothcontrol();
  }
}
void Bluetoothcontrol() {
  if (RemoteXY.slider_2 > 5) {
    int val = map(RemoteXY.slider_2, 6, 100, 0, 90);
    servo_motor.write(val);


  } else if (RemoteXY.slider_2 < 2) {
    int val2 = map(RemoteXY.slider_2, 2, -100, 90, 180);
    servo_motor.write(val2);
  } else {
    servo_motor.write(0);
  }

  if (RemoteXY.button_Forward == 1) {
    moveForward();
  } else if (RemoteXY.button_Backward == 1) {
    moveBackward();
  } else if (RemoteXY.button_Left == 1) {
    turnLeft();
  } else if (RemoteXY.button_Right == 1) {
    turnRight();
  } else {
    moveStop();
  }
}
void CIR() {
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 35) {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft) {
      turnRight();
      moveStop();
    } else {
      turnLeft();
      moveStop();
    }
  } else {
    moveForward();
  }
  distance = readPing();
}
int lookRight() {
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft() {
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward() {
  goesForward = false;
  if (!goesForward) {

    goesForward = true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
  }
}

void moveBackward() {

  goesForward = false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
}

void turnRight() {

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  delay(250);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void turnLeft() {

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(250);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
