#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>  // Changed from String to Vector3
#include "pid.h"



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

//encoder
const int motorencode1A = 4;
const int motorencode1B = 16;
const int motorencode2A = 17;
const int motorencode2B = 5;
const int motorencode3A = 18;
const int motorencode3B = 19;

double pos[] = {0,0,0};
double oldpos[] = {0,0,0};
double rev[] = {0,0,0};

double setpoint[] = {0,0,0};
double U[] = {0,0,0};


PID M1pid(-255,255, 1.2, 0.5,0.0);
PID M2pid(-255,255, 1.2, 0.5,0.0);
PID M3pid(-255,255, 1.2, 0.5,0.0);



//motor1
const int motor1a =12;
const int motor1b =14;
//motor2
const int motor2a =26;
const int motor2b =27;
//motor3
const int motor3a =33;
const int motor3b =25;

void Drive_Motor(int motorA,int motorB,int motorvalue){
    motorvalue = constrain(motorvalue, -255, 255);
    if(motorvalue > 0){
        analogWrite(motorA,abs(motorvalue));
        analogWrite(motorB,0);
    }else if(motorvalue < 0){
        analogWrite(motorA,0);
        analogWrite(motorB,abs(motorvalue));
    }else{
        analogWrite(motorA,0);
        analogWrite(motorB,0);
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

void IRAM_ATTR cal_encoder(){
    rev[0] = (pos[0] - oldpos[0])/(90*11);
    oldpos[0] = pos[0];
    rev[0]*=(10 * 60) ;   //   r/0.1s   ->  r/s

    rev[1] = (pos[1] - oldpos[1])/(90*11);
    oldpos[1] = pos[1];
    rev[1]*=(10 * 60) ;

    rev[2] = (pos[2] - oldpos[2])/(90*11);
    oldpos[2] = pos[2];
    rev[2]*=(10 * 60) ;

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

// Timer callback to publish a Vector3 message
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {  
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

        // publish variable (enc)
        msg.x = rev[0];
        msg.y = rev[1];
        msg.z = rev[2];
        
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        Serial.print("Publishing: x="); Serial.print(msg.x);
        Serial.print(" y="); Serial.print(msg.y);
        Serial.print(" z="); Serial.println(msg.z);
    }
}

// Subscription callback for "cnt_vel" topic
void subscription_callback(const void *msgin) {  
    const geometry_msgs__msg__Vector3 *incoming_msg = (const geometry_msgs__msg__Vector3 *)msgin;
    
    Serial.print("Received cnt_vel: x="); Serial.print(incoming_msg->x);
    Serial.print(" y="); Serial.print(incoming_msg->y);
    Serial.print(" z="); Serial.println(incoming_msg->z);

    // Process data if needed (e.g., modify and republish)
    msg.x = incoming_msg->x;
    msg.y = incoming_msg->y;
    msg.z = incoming_msg->z;

    // Publish to check incomingdata
    // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    // setpoint[0] = (-msg.y + (-msg.z * 0.100 * 3)) ;
    // setpoint[1] = ((msg.y * sin(PI/6)) + (msg.x * cos(PI/6)) + (-msg.z * 0.100 * 3)); 
    // setpoint[2] = ((msg.y * sin(PI/6)) - (msg.x * cos(PI/6)) + (-msg.z * 0.100 * 3)); 

    //setpoint (rad/s)
    setpoint[0] = (1/0.0425)* ( -msg.y - (msg.z * 0.100) ) ;
    setpoint[1] = (1/0.0425)* ( (msg.y*sin(PI/6)) + (msg.x * cos(PI/6)) - (msg.z * 0.100) );
    setpoint[2] = (1/0.0425)* ( (msg.y*sin(PI/6)) - (msg.x * cos(PI/6)) - (msg.z * 0.100) );

    //setpoint (rev/s)
    setpoint[0]/=(2*PI);
    setpoint[1]/=(2*PI);
    setpoint[2]/=(2*PI);
    //(rev/min)
    setpoint[0]*=60;
    setpoint[1]*=60;
    setpoint[2]*=60;

    U[0] = M1pid.compute(setpoint[0],rev[0]);
    U[1] = M2pid.compute(setpoint[1],rev[1]);
    U[2] = M3pid.compute(setpoint[2],rev[2]);
    // analog write  here

    Drive_Motor(motor1a,motor1b,U[0]);
    Drive_Motor(motor2a,motor2b,U[1]);
    Drive_Motor(motor3a,motor3b,U[2]);
    // delay(2);
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

    pinMode(motor1a,OUTPUT);
    pinMode(motor1b,OUTPUT);
    pinMode(motor2a,OUTPUT);
    pinMode(motor2b,OUTPUT);
    pinMode(motor3a,OUTPUT);
    pinMode(motor3b,OUTPUT);

    pinMode(motorencode1A,INPUT_PULLDOWN);
    pinMode(motorencode1B,INPUT_PULLDOWN);
    pinMode(motorencode2A,INPUT_PULLDOWN);
    pinMode(motorencode2B,INPUT_PULLDOWN);
    pinMode(motorencode3A,INPUT_PULLDOWN);
    pinMode(motorencode3B,INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(motorencode1A),&read_encoder1,RISING);
    attachInterrupt(digitalPinToInterrupt(motorencode2A),&read_encoder2,RISING);
    attachInterrupt(digitalPinToInterrupt(motorencode3A),&read_encoder3,RISING);

    My_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(My_timer, &cal_encoder, true);
    timerAlarmWrite(My_timer, 100000 , true);
    timerAlarmEnable(My_timer); 


    delay(1500);
    Drive_Motor(motor1a,motor1b,80);
    Drive_Motor(motor2a,motor2b,80);
    Drive_Motor(motor3a,motor3b,0);
    delay(300);
    Drive_Motor(motor1a,motor1b,0);
    Drive_Motor(motor2a,motor2b,0);
    Drive_Motor(motor3a,motor3b,0);

}

void loop() {
    // delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
}
