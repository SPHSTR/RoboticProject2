#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>  // Changed from String to Vector3
#include "pid.h"
#include <ESP32Servo.h>
// #include <PCF8574.h>

// PCF8574 Encoder(0x24);

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;  // Vector3 message type
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

hw_timer_t *My_timer = NULL;

//motor1
const int motor1a =12;
const int motor1b =14;
const int motor1V =0;

//motor2
const int motor2a =26;
const int motor2b =27;
const int motor2V =0;
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

const int Hallsensor1 = 0;
const int Hallsensor2 = 0;
const int Hallsensor3 = 0;

Servo servo1;  //108
Servo servo2;  //360

double pos[] = {0,0,0};
double oldpos[] = {0,0,0};


double setpoint[] = {0,0,0};
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

void SetZero(){
  //motordrive -> HE got activate -> pos[] = 0
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void error_loop() {
    while(1) {
        // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        analogWrite(LED_PIN,60);
        delay(50);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

      // publish variable (enc)
      msg.x = 0;
      msg.y = 0;
      msg.z = 0;
      
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}


void subscription_callback(const void *msgin) {  
  const geometry_msgs__msg__Vector3 *incoming_msg = (const geometry_msgs__msg__Vector3 *)msgin;
  // Process data if needed (e.g., modify and republish)
  msg.x = incoming_msg->x;
  msg.y = incoming_msg->y;
  msg.z = incoming_msg->z;

  // Publish to check incomingdata
  // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));


  //setpoint 
  setpoint[0] = msg.x;
  setpoint[1] = msg.y;
  setpoint[2] = msg.z;


  U[0] = M1pid.compute(setpoint[0],pos[0]);
  U[1] = M2pid.compute(setpoint[1],pos[1]);
  U[2] = M3pid.compute(setpoint[2],pos[2]);

  // analog write  here
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,LOW);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize message
  geometry_msgs__msg__Vector3__init(&msg);

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "wheel_node", "", &support));

  // Create publisher (Vector3)
  RCCHECK(rclc_publisher_init_default(&publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),"enc_vel"));

  // Create subscriber (Vector3)
  RCCHECK(rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),"cnt_vel"));

  // Create timer (publishes every ??? second)
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


  servo1.attach(0);
  servo2.attach(0);

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
