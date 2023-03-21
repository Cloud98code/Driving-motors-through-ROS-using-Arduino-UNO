/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <arduino_motor_control/Motors.h> // Replace header

ros::NodeHandle  nh;

Servo servo;
int16_t current_servo_position = 0;
const uint8_t DC_motor_EN1_pin = 3; 
const uint8_t DC_motor_IN1_pin = 2; // Choose digital PIN for IN1
const uint8_t DC_motor_IN2_pin = 4; // Choose digital PIN for IN2, pins for DC motor spin direction

void motors_cb( const arduino_motor_control::Motors& motor_msg){ 
    /* SERVO MOTOR POSITION UPDATE */
    if (motor_msg.servo_type == 0){ // Absolute position command received
	      current_servo_position = motor_msg.servo_command;//motor_msg.servo_command; // Update position
        servo.write(motor_msg.servo_command); //set servo angle, should be from 0-180
        
    }
    else if (motor_msg.servo_type == 1){ // Incremental position command received
        // Verify boundaries are not exceeded
        if (current_servo_position + motor_msg.servo_command >= 175) {
            servo.write(180);
            current_servo_position = 180;
         }
        else if (current_servo_position + motor_msg.servo_command <= 0) {
            servo.write(0);
            current_servo_position = 0;
        }
        //If no boundary is overcome, increase/decrease servo position
        else {
            servo.write(current_servo_position + motor_msg.servo_command);
            current_servo_position = current_servo_position + motor_msg.servo_command;
        }
    }


    /* DC MOTOR UPDATE */
    if (motor_msg.DC_motor_command > 0){ // Spin the motor counter clock-wise
            digitalWrite(DC_motor_IN1_pin,LOW) ;
            digitalWrite(DC_motor_IN2_pin,HIGH) ;
            analogWrite(DC_motor_EN1_pin, abs(motor_msg.DC_motor_command)) ;                             
    }
    else if (motor_msg.DC_motor_command < 0){ // Spin the motor clock-wise
            digitalWrite(DC_motor_IN1_pin,HIGH) ;
            digitalWrite(DC_motor_IN2_pin,LOW) ;
            analogWrite(DC_motor_EN1_pin, -motor_msg.DC_motor_command) ;                             
    }

  // Toggle LED when reading from topic
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<arduino_motor_control::Motors> sub("motor_commands", motors_cb); 

void setup(){
  /* Initialize the LED */
  pinMode(13, OUTPUT);

  /* Initialize ROS node and subscribe to "motor_commands" topic */
  nh.initNode();
  nh.subscribe(sub);
  
  /* Setup servo motor */
  servo.attach(9); //attach it to pin 9
  servo.write(200); // Send the servo motor to 0 position

  /* Setup DC motor */
  pinMode(DC_motor_EN1_pin,OUTPUT) ;    //we have to set PWM pin as output
  pinMode(DC_motor_IN1_pin,OUTPUT) ;  //Logic pins are also set as output
  pinMode(DC_motor_IN2_pin,OUTPUT) ;
}

void loop(){
  nh.spinOnce(); // Looks into topic to see if anything has been published at 100Hz
  delay(1); // Wait 1 ms
}
