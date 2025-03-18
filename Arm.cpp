#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <geometry_msgs/msg/vector3.h>  // Changed from String to Vector3
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include "pid.h"
#include <ESP32Servo.h>
// #include <PCF8574.h>

// PCF8574 Encoder(0x24);

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
// geometry_msgs__msg__Vector3 msg;  // Vector3 message type
// std_msgs__msg__Float32MultiArray msg;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

hw_timer_t *My_timer = NULL;

//motor1
const int motor1a =12;
const int motor1b =14;
const int motor1V =15;

//motor2
const int motor2a =26;
const int motor2b =27;
const int motor2V =2;
//motor3
const int motor3a =33;
const int motor3b =25;
const int motor3V =0;

//encoder
const int motorencode1A = 4;
const int motorencode1B = 16;
const int motorencode2A = 17;
const int motorencode2B = 5;
const int motorencode3A = 18;
const int motorencode3B = 19;

const int Hallsensor1 = 32;
const int Hallsensor2 = 35;
const int Hallsensor3 = 34;

Servo servo1;  //180
Servo servo2;  //360

double pos[] = {0,0,0,0,0};  //out

double setpoint[] = {0,0,0,0};   //in J1-4
double Jend = 0.0;  //in end eff


double U[] = {0,0,0};
PID M1pid(-255,255, 1.2, 0.5,0.0);
PID M2pid(-255,255, 1.2, 0.5,0.0);
PID M3pid(-255,255, 1.2, 0.5,0.0);

void Drive_Motor(int motorA,int motorB,int motorPWM,int motorvalue){
  motorvalue = constrain(motorvalue, -255, 255);
  if(motorvalue > 0){
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,LOW);
    analogWrite(motorPWM,abs(motorvalue));
  }else if(motorvalue < 0){
    digitalWrite(motorA,LOW);
    digitalWrite(motorB,HIGH);
    analogWrite(motorPWM,abs(motorvalue));
  }else{
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,HIGH);
    analogWrite(motorPWM,0);
  }
}

void IRAM_ATTR read_encoder1(){
  int a = digitalRead(motorencode1B);
  if(a>0){pos[0]+=1;}else{pos[0]-=1;}
}

void IRAM_ATTR read_encoder2(){
  int a = digitalRead(motorencode2B);
  if(a>0){pos[1]+=1;}else{pos[1]-=1;}
}

void IRAM_ATTR read_encoder3(){
  int a = digitalRead(motorencode3B);
  if(a>0){pos[2]+=1;}else{pos[2]-=1;}
}

double joint1_deg_to_pos(double J){
  J = constrain(J , 0 , 360);
  double pos = ( J / 360 ) * 5 * 103 * 11 ;
  return pos ; 
}

double joint2_deg_to_pos(double J){
  J = constrain(J , 0 , 270);
  double pos = ( J / 360 ) * 16 * 103 * 11 ;
  return pos ; 
}

double joint3_deg_to_pos(double J){
  J = constrain(J , 0 , 300);
  double pos = ( J / 360 ) * 12.25 * 103 * 11 ;
  return pos ; 
}

double joint4_deg_to_pos(double J){
  J = constrain(J , 0 , 180);
  return J ; 
}

void SetZero(){
  //motordrive -> HE got activate -> pos[] = 0
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void error_loop() {
    while(1) {
        digitalWrite(2, !digitalRead(2));
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
      msg.linear.x = pos[0];  // Motor 1 Position
      msg.linear.y = pos[1];  // Motor 2 Position
      msg.linear.z = pos[2];  // Motor 3 Position
      msg.angular.x = pos[3]; // Servo1 position
      msg.angular.y = pos[4]; // Servo2 position (gripper)
      msg.angular.z = 0.0;    // Not used, set to 0

      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}


unsigned long servo2StartTime = 0;
bool servo2Moving = false;
int servo2Target = 90;  // Neutral position
String gripperstate = "";

void checkServo2Timeout() {
  if (servo2Moving && (millis() - servo2StartTime >= 1000)) {  // 1 second elapsed
      servo2.write(90);  // Stop the servo
      servo2Moving = false;
  }
}

void subscription_callback(const void *msgin) {

  const geometry_msgs__msg__Twist *incoming_msg = (const geometry_msgs__msg__Twist *)msgin;


  // Process data if needed (e.g., modify and republish)

  setpoint[0] = incoming_msg->linear.x;
  setpoint[1] = incoming_msg->linear.y;
  setpoint[2] = incoming_msg->linear.z;
  setpoint[3] = incoming_msg->angular.x;
  setpoint[4] = incoming_msg->angular.y;


  // Publish to check incomingdata
  // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));


  U[0] = M1pid.compute(setpoint[0],pos[0]);
  U[1] = M2pid.compute(setpoint[1],pos[1]);
  U[2] = M3pid.compute(setpoint[2],pos[2]);

  // analog write  here
  Drive_Motor(motor1a,motor1b,motor1V,U[0]);
  Drive_Motor(motor2a,motor2b,motor2V,U[1]);
  Drive_Motor(motor3a,motor3b,motor3V,U[2]);

  servo1.write(setpoint[3]);


  if (!servo2Moving) {
    if (setpoint[4] == 0 and gripperstate == "close") {  // Open
        servo2Target = 0;
        servo2Moving = true;
        gripperstate = "open";
        servo2StartTime = millis();
    } else if (setpoint[4] == 1 and gripperstate == "open") {  // Close
        servo2Target = 180;
        servo2Moving = true;
        gripperstate = "close";
        servo2StartTime = millis();
    }
    servo2.write(servo2Target);
  }

  checkServo2Timeout();
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize message
  // std_msgs__msg__Float32MultiArray__init(&msg);
  geometry_msgs__msg__Twist__init(&msg);
  

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "arm_node", "", &support));

  // Create publisher (Vector3)
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "arm_pos"));

  // Create subscriber (Vector3)
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cnt_arm"));

  // Create timer (publishes every ??? second)
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


  servo1.attach(22);
  servo2.attach(23);

  pinMode(motor1a,OUTPUT);
  pinMode(motor1b,OUTPUT);
  pinMode(motor2a,OUTPUT);
  pinMode(motor2b,OUTPUT);
  pinMode(motor3a,OUTPUT);
  pinMode(motor3b,OUTPUT);

  pinMode(motor1V,OUTPUT);
  pinMode(motor2V,OUTPUT);
  pinMode(motor3V,OUTPUT);

  // Encoder.pinMode(P0,INPUT_PULLDOWN);
  // Encoder.pinMode(P1,INPUT_PULLDOWN);
  // Encoder.pinMode(P2,INPUT_PULLDOWN);
  // Encoder.pinMode(P3,INPUT_PULLDOWN);
  // Encoder.pinMode(P4,INPUT_PULLDOWN);
  // Encoder.pinMode(P5,INPUT_PULLDOWN);

  pinMode(motorencode1A,INPUT_PULLDOWN);
  pinMode(motorencode1B,INPUT_PULLDOWN);
  pinMode(motorencode2A,INPUT_PULLDOWN);
  pinMode(motorencode2B,INPUT_PULLDOWN);
  pinMode(motorencode3A,INPUT_PULLDOWN);
  pinMode(motorencode3B,INPUT_PULLDOWN);

  pinMode(Hallsensor1,INPUT_PULLUP);
  pinMode(Hallsensor2,INPUT_PULLUP);
  pinMode(Hallsensor3,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motorencode1A),&read_encoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(motorencode2A),&read_encoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(motorencode3A),&read_encoder3,RISING);
}


void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
}
