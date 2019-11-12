#include <Servo.h>

float Kp = 10, Ki = 0.5, Kd = 0;                    //PID control parameters
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//Initial the PID inputs and output 
float previous_error = 0, previous_I = 0;           //The error of PID input.
static int initial_motor_speed = 150;  
int sensor[5] = {0, 0, 0, 0, 0}; 
int leftMotor1 = 14;
int leftMotor2 = 15;
int rightMotor1 = 16;
int rightMotor2 = 17;
int turnspeed = 150;
int forwardspeed = 150;
int trac1 = 8; //The sensors sort from right side of the front car
int trac2 = 7;
int trac3 = 9;
int trac4 = 2;
int trac5 = 10;

int leftPWM = 3;
int rightPWM = 5;


void tracing(void);
void calc_pid(void);
void motor_control(void);
  
void setup() {
  // put your setup code here, to run once:
  //initial the serial port
  Serial.begin(9600);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  //initial the IR sensor port
  pinMode(trac1, INPUT);
  pinMode(trac2, INPUT);
  pinMode(trac3, INPUT);
  pinMode(trac4, INPUT);
  pinMode(trac5, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  tracing();
  calc_pid();
  motor_control();

}

void motorRun(int cmd, int value)
{
  analogWrite(leftPWM, value);  // set the PWM output (the speed of robot)
  analogWrite(rightPWM, value);
  switch (cmd) {
    case FORWARD:
      Serial.println("FORWARD"); //the state of the output
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
    case BACKWARD:
      Serial.println("BACKWARD"); //the state of the output
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
    case TURNLEFT:
      Serial.println("TURN  LEFT"); //the state of the output
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
    case TURNRIGHT:
      Serial.println("TURN  RIGHT"); //the state of the output
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
    default:
      Serial.println("STOP"); //the state of the output
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
  }
}

void motor_control()
{
  //int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;
  if(right_motor_speed < -255){
    right_motor_speed = -255;
  }
  if(right_motor_speed > 255){
    right_motor_speed = 255;
  }
  
  if (error = 0)
  {
    motorRun(FORWARD,right_motor_speed);
  } else if (error > 0)
  {
    motorRun(TURNRIGHT,right_motor_speed);
  } else if (error < 0)
  {
    motorRun(TURNLEFT,right_motor_speed);
  }
}

void tracing()
{
  sensor[0] = digitalRead(trac5);
  sensor[1] = digitalRead(trac4);
  sensor[2] = digitalRead(trac3);
  sensor[3] = digitalRead(trac2);
  sensor[4] = digitalRead(trac1);
  
    if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3
] == 1) && (sensor[4] == 1)) {
      error = 0;// corner 1 1 1 1 1   forward
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (
sensor[3] == 1) && (sensor[4] == 0)) {
      error = 0;//small corner 0 1 1 1 0   forward
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (
sensor[3] == 1) && (sensor[4] == 1)) {
      error = 2;//right 0 0 1 1 1    turn right
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (
sensor[3] == 1) && (sensor[4] == 0)) {
      error = 1;//right 0 0 1 1 0    turn right 
    } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (
sensor[3] == 0) && (sensor[4] == 0)) {
      error = -2;//left 1 1 1 0 0    turn left 
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (
sensor[3] == 0) && (sensor[4] == 0)) {
      error = -1;//left 0 1 1 0 0    turn left 
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[4] == 1)) {
      error = 2;//          0 0 0 0 1  turn right
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (
sensor[3] == 1) && (sensor[4] == 0)) {
      error = 1;//          0 0 0 1 0  turn right
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (
sensor[3] == 0) && (sensor[4] == 0)) {
      error = 0;//          0 0 1 0 0  forward
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (
sensor[3] == 0) && (sensor[4] == 0)) {
      error = -1;//         0 1 0 0 0  turn left
    } else if ((sensor[0] == 1) && (sensor[2] == 0) && ( sensor[3] == 0) && (sensor[4] == 0)) {
      error = -2;//         1 0 0 0 0  turn left
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (
sensor[3] == 0) && (sensor[4] == 0)) {
      motorRun(STOP, 0); //  0 0 0 0 0 Stop
    }

  Serial.print(sensor[0]);
  Serial.print("---");
  Serial.print(sensor[1]);
  Serial.print("---");
  Serial.print(sensor[2]);
  Serial.print("---");
  Serial.print(sensor[3]);
  Serial.print("---");
  Serial.print(sensor[4]);
  Serial.print("---");
  Serial.println();
}

void calc_pid()  //PID algorithm
  P = error;
  I = I + error;
  D = error - previous_error;
 
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
 
  previous_error = error;
}
