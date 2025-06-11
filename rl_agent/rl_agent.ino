// Required Libraries
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

// EEPROM settings
#define EEPROM_SIZE 1     // 1 byte to store robot number (1-8)
#define ROBOT_NUM_ADDR 0  // EEPROM address for robot number

// Base names for AP and OTA
const char *base_ssid = "ESP32-AP-";
const char *base_ota_hostname = "ESP32-OTA-";
const char *ap_password = "12345678";  // Common password for all APs

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
  // Dynamic network and OTA identifiers
  uint8_t robot_number;
  char ssid[32];
  char ota_hostname[32];
  TaskHandle_t otaTaskHandle;

  RLRobot()
    : sonar(triggerPin, echoPin, MAX_DISTANCE),
      lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS),
      servoDownAngle(0),
      servoUpAngle(180),
      robot_number(0),
      otaTaskHandle(NULL) {}

  void saveRobotNumber(uint8_t number) {
    EEPROM.write(ROBOT_NUM_ADDR, number);
    EEPROM.commit();
    Serial.print("Saved robot number: ");
    Serial.println(number);
  }

  uint8_t readRobotNumber() {
    uint8_t number = EEPROM.read(ROBOT_NUM_ADDR);
    if (number < 1 || number > 8) {
      Serial.println("Invalid or uninitialized robot number. Please set number (1-8).");
      lcd.clear();
      lcd.print("Set Robot Num: 1-8");
      while (!Serial.available()) {
        delay(100);  // Wait for Serial input
      }
      number = Serial.parseInt();
      if (number >= 1 && number <= 8) {
        saveRobotNumber(number);
        lcd.clear();
        lcd.print("Robot Num Set: ");
        lcd.print(number);
        delay(2000);
      } else {
        Serial.println("Invalid input. Defaulting to robot number 1.");
        number = 1;
        saveRobotNumber(number);
      }
    }
    return number;
  }

  void setup_ap() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, ap_password);
    Serial.println("AP Started");
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
  }

  void setup_ota() {
    ArduinoOTA.setHostname(ota_hostname);
    ArduinoOTA.onStart([this]() {
      Serial.println("OTA Start");
      lcd.clear();
      lcd.print("OTA Update Start");
    });
    ArduinoOTA.onEnd([this]() {
      Serial.println("\nOTA End");
      lcd.clear();
      lcd.print("OTA Update Done");
    });
    ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      lcd.clear();
      lcd.print("OTA Progress: ");
      lcd.setCursor(0, 1);
      lcd.print((progress / (total / 100)));
      lcd.print("%");
    });
    ArduinoOTA.onError([this](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      lcd.clear();
      lcd.print("OTA Error");
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    Serial.print("OTA Hostname: ");
    Serial.println(ota_hostname);
  }

  // OTA task to run asynchronously
  static void otaTask(void *parameter) {
    for (;;) {
      ArduinoOTA.handle();                  // Handle OTA updates
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield for 10ms
    }
  }

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

  void printOnLCD(const char *message) {
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

  void moveServos(int targetDown, int targetUp, int stepDelay = 7, int stepSize = 2) {
    targetDown = constrain(targetDown, 0, 90);
    targetUp = constrain(targetUp, 0, 180);

    int deltaDown = targetDown - servoDownAngle;
    int deltaUp = targetUp - servoUpAngle;

    int stepsDown = abs(deltaDown) / stepSize;
    int stepsUp = abs(deltaUp) / stepSize;
    int totalSteps = max(stepsDown, stepsUp);

    bool downDone = false;
    bool upDone = false;

    for (int i = 1; i <= totalSteps; i++) {
      if (i <= stepsDown) {
        int newDown = servoDownAngle + (deltaDown > 0 ? i * stepSize : -i * stepSize);
        servoDown.write(constrain(newDown, 0, 90));
      } else if (!downDone) {
        servoDown.write(targetDown);
        downDone = true;
      }

      if (i <= stepsUp) {
        int newUp = servoUpAngle + (deltaUp > 0 ? i * stepSize : -i * stepSize);
        servoUp.write(constrain(newUp, 0, 180));
      } else if (!upDone) {
        servoUp.write(targetUp);
        upDone = true;
      }

      delay(stepDelay);
    }

    servoDownAngle = targetDown;
    servoUpAngle = targetUp;
  }

  void healthCheck() {
    // 1. Print Health Check on LCD
    printOnLCD("Health Check Start");

    // 2. Get distance and print it
    int dist = getDistance();
    printDistance(dist);

    // 3. Move servos to check
    moveServos(90, 90);
    delay(1000);

    // 4. Move servos back to initial position
    moveServos(0, 180);
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

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Read or set robot number
  robot.robot_number = robot.readRobotNumber();
  snprintf(robot.ssid, sizeof(robot.ssid), "%s%d", base_ssid, robot.robot_number);
  snprintf(robot.ota_hostname, sizeof(robot.ota_hostname), "%s%d", base_ota_hostname, robot.robot_number);

  // Initialize AP and OTA
  robot.setup_ap();
  robot.setup_ota();

  // Start OTA task
  xTaskCreatePinnedToCore(
    RLRobot::otaTask,
    "OTATask",
    4096,
    &robot,
    1,
    &robot.otaTaskHandle,
    1);

  // Notify setup running
  char buf[32];
  snprintf(buf, sizeof(buf), "RL Robot %d Setup", robot.robot_number);
  robot.printOnLCD(buf);

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