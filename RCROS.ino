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

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>

int R_ADELANTE = 5;
int R_ATRAS = 6;
int L_ADELANTE = 11;
int L_ATRAS = 10;
/*carlos1*/
ros::NodeHandle  nh;

void servo_cb( const geometry_msgs::Twist& cmd_msg){
  if (cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ){
      // parar
  } else{
/*carlos2*/  
  if (cmd_msg.angular.z < 0){
  int a=cmd_msg.angular.z * 255;
   derecha(a);
 } else if(cmd_msg.angular.z > 0){
   izquierda(255)
  }else if(cmd_msg.linear.x < 0){
    // atras
  }else if(cmd_msg.linear.x > 0){
    // adelante
  }
  }
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}
/*carlos3*/

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", servo_cb);
/*rodrigo1*/
void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}
/*rodrigo2*/
void loop(){
  nh.spinOnce();
  delay(1);
}

void adelante(int velocidad){
analogWrite(R_ADELANTE, velocidad);
digitalWrite(R_ATRAS, LOW);
analogWrite(L_ADELANTE, velocidad);
digitalWrite(L_ATRAS, LOW);
}
/*rodrigo3*/
void atras(int velocidad){
digitalWrite(R_ADELANTE, LOW);
analogWrite(R_ATRAS, velocidad);
digitalWrite(L_ADELANTE, LOW);
analogWrite(L_ATRAS, velocidad);
}
/*lesmo1*/
void izquierda(int velocidad){
analogWrite(R_ADELANTE, velocidad);
digitalWrite(R_ATRAS, LOW);
digitalWrite(L_ADELANTE, LOW);
analogWrite(L_ATRAS, velocidad);
}
/*lesmo2*/
void derecha(int velocidad){
digitalWrite(R_ADELANTE, LOW);
analogWrite(R_ATRAS, velocidad);
analogWrite(L_ADELANTE, velocidad);
digitalWrite(L_ATRAS, LOW);
}
/*lesmo3*/
