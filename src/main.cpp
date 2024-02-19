#include <ArduinoJson-v7.0.3.h>
#include <rhComms.h>
#include "WiFi.h"

const char* ssid = "Nizar";
const char* password = "nizar2004";

CarController controller;

WiFiServer server(80);
#include <L298N.h>
#include <ESP32Servo.h>
#define ENA 27
#define IN1 14
#define IN2 12
#define ENB 0
#define IN3 4
#define IN4 2

Servo servoMotor;
#define servo_pin 26
Servo servoMotor1;
#define servo_pin1 25

L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN4, IN3);

void setup() {
    // initialize serial communication
  Serial.begin(9600);

  // set ESP32 LED pin to LOW
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // start the wifi access point with the specified name and password
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  // set the CarController object's server reference to the server object
  controller.setServer(&server);

  // print the IP address of the ESP32 to the serial monitor
  Serial.print("IP Address of ESP32: ");
  Serial.println(WiFi.softAPIP());

  // start the server on port 80
  server.begin();
  motor1.setSpeed(200);
  motor2.setSpeed(200);

  servoMotor.attach(26);
  servoMotor1.attach(25);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (controller.hasClient()) { //
    int read = controller.run();
    if (read == -1) {
      Serial.println("Error reading");
    } else {
      int w_key = controller.getValue("keys", "W");
      int s_key = controller.getValue("keys", "S");
      int a_key = controller.getValue("keys", "A");
      int d_key = controller.getValue("keys", "D");
      int up_arrow = controller.getValue("keys", "UP");
      int down_arrow = controller.getValue("keys", "DOWN");
      int right_arrow = controller.getValue("keys", "RIGHT");
      int left_arrow = controller.getValue("keys", "LEFT");

      // Forward
      if (w_key == 1) {
        motor1.forward();
        motor2.forward();
      }else if (s_key == 1) {
        motor1.backward();
        motor2.backward();
      }else if (d_key == 1) {
        motor1.forward();
        motor2.backward();
      }else if (a_key == 1) {
        motor1.backward();
        motor2.forward();
      }else{
        motor1.stop();
        motor2.stop();
      }

      // Adjust motor speed with UP/DOWN arrows
      if (up_arrow == 1) {
        motor1.setSpeed(motor1.getSpeed() + 30);  // Increase speed
        motor2.setSpeed(motor2.getSpeed() + 30);
      }

      if (down_arrow == 1) {
        motor1.setSpeed(motor1.getSpeed() - 30);  // Decrease speed
        motor2.setSpeed(motor2.getSpeed() - 30);
      }

      if (right_arrow == 1) {
        rotateServo(10);
      }

      if (left_arrow == 1) {
        rotateServo(-10);
      }

    }
  } else {
  Serial.println("Waiting for client...");
  while (controller.connectClient()) {}
  Serial.println("Connected to client!");
  }
  delay(50);
}
void rotateServo(int degrees) {
  int currentPos = servoMotor.read();
  int newPos = constrain(currentPos + degrees, 0, 180);  
    int newPos1 = constrain(servoMotor1.read() - degrees, 0, 180);  // Constrain within servo's range
// Constrain within servo's range
  servoMotor.write(newPos);
    servoMotor1.write(newPos1);

  delay(15);  // Delay to allow the servo to reach the desired position
}
