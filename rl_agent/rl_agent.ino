// Required Libraries
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>

// Pin Definitions
const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200  // Maximum distance to check in cm
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

class RLRobot {
private:
  // Hardware
  Servo servoDown;
  Servo servoUp;
  NewPing sonar;
  LiquidCrystal_I2C lcd;
  int servoDownAngle;
  int servoUpAngle;

  // Agent

public:
  RLRobot()
    : sonar(triggerPin, echoPin, MAX_DISTANCE),
      lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS),
      servoDownAngle(0),
      servoUpAngle(180) {}

  void setupLCD() {
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();

    printOnLCD("LCD Ready!");
  }

  void setupServos() {
    servoDown.attach(control_servo_down, 600, 2400);
    servoUp.attach(control_servo_up, 600, 2400);
    servoDown.write(servoDownAngle);
    servoUp.write(servoUpAngle);
  }

  void printOnLCD(const char* message) {
    lcd.clear();
    lcd.print(message);
    delay(2000);
    lcd.clear();
  }

  int getDistance() {
    int distance = sonar.ping_cm();  // Get distance in cm

    if (distance == 0) {
      Serial.println("Warning: No echo received from SRF module.");
      return -1;
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    return distance;
  }

  void printDistance(int distance) {
    lcd.clear();

    if (distance < 0) {
      lcd.print("Dist: SRF Error");
    } else {
      lcd.print("Dist: ");
      lcd.print(distance);
      lcd.print(" cm");
    }

    delay(1000);
  }

  void moveServoDown(int toAngle, int stepDelay = 10) {
    if (servoDownAngle < toAngle) {
      for (int angle = servoDownAngle; angle <= toAngle; angle += 2) {
        servoDown.write(angle);
        delay(stepDelay);
      }
    } else {
      for (int angle = servoDownAngle; angle >= toAngle; angle -= 2) {
        servoDown.write(angle);
        delay(stepDelay);
      }
    }

    servoDownAngle = toAngle;
  }

  void moveServoUp(int toAngle, int stepDelay = 10) {
    if (servoUpAngle < toAngle) {
      for (int angle = servoUpAngle; angle <= toAngle; angle += 2) {
        servoUp.write(angle);
        delay(stepDelay);
      }
    } else {
      for (int angle = servoUpAngle; angle >= toAngle; angle -= 2) {
        servoUp.write(angle);
        delay(stepDelay);
      }
    }

    servoUpAngle = toAngle;
  }

  void healthCheck() {
    // 1. Print Health Check on LCD
    printOnLCD("Health Check Start");

    // 2. Get distance and print it
    int dist = getDistance();
    printDistance(dist);

    // 3. Move servos to check
    moveServoDown(90);
    moveServoUp(90);
    delay(1000);

    // 4. Move servos back to initial position
    moveServoDown(0);
    moveServoUp(180);
    printOnLCD("Servos Reset");

    // 5. Print completion message
    printOnLCD("Health Check Done");
  }

  void doTraining() {
    // TODO: Add your training logic here
  }

  void doLearnedBehavior() {
    // TODO: Add your learned behavior logic here
  }
};

RLRobot robot;

void setup() {

  robot.setupLCD();

  // Notify setup running
  robot.printOnLCD("RL Robot Setup");

  robot.setupServos();

  // Notify setup completion
  robot.printOnLCD("Setup Completed");

  robot.healthCheck();
}

void loop() {
  robot.printOnLCD("Main Loop Running");
  int dist = robot.getDistance();
  robot.printDistance(dist);
  // TODO: Implement your main loop logic here
}