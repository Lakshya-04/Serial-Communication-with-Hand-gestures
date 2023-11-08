#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>  
ros::NodeHandle nh;
int buzzerPin = 7;
Servo myServo; 
int servoPin = 8;   
void messageCallback(const std_msgs::String& msg) {
    if (strcmp(msg.data, "One") == 0) { 
        digitalWrite(2, HIGH);
        digitalWrite(3, LOW);
        digitalWrite(4, HIGH); 
        digitalWrite(5, LOW);} 
    else if(strcmp(msg.data, "Two") == 0) {
        digitalWrite(3, HIGH);
        digitalWrite(2, LOW); 
        digitalWrite(5, HIGH);  
        digitalWrite(4, LOW);}
    else if(strcmp(msg.data, "Three") == 0) {
        digitalWrite(2, HIGH); 
        digitalWrite(3, LOW); 
        digitalWrite(5, HIGH);
        digitalWrite(4, LOW); }
    else if(strcmp(msg.data, "Four") == 0) {
        digitalWrite(3, HIGH); 
        digitalWrite(2, LOW); 
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW); }
    else if(strcmp(msg.data, "Five") == 0) {
        digitalWrite(3, HIGH); 
        digitalWrite(2, LOW); 
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW); }
    else if(strcmp(msg.data, "Noo") == 0) {
      tone(buzzerPin, 2000);
    }
    else if(strcmp(msg.data, "Yo") == 0) {
        digitalWrite(3, HIGH); 
        digitalWrite(2, LOW); 
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
        myServo.attach(servoPin);
        myServo.write(90);
        tone(buzzerPin, 500);
        delay(200);
        digitalWrite(2, HIGH); 
        digitalWrite(3, LOW); 
        digitalWrite(5, HIGH);
        digitalWrite(4, LOW);
        myServo.write(0);
        tone(buzzerPin, 1000);
        delay(200);
        myServo.detach(); 
       }
      
    else{
      digitalWrite(2, LOW); 
        digitalWrite(3, LOW);  
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        noTone(buzzerPin);
    }
}

ros::Subscriber<std_msgs::String> sub("gesture", &messageCallback);

void setup() {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
     pinMode(buzzerPin, OUTPUT);  
    
    nh.getHardware()->setBaud(9600);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
} 
