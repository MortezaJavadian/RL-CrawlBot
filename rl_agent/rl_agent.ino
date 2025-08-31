// Required Libraries
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <vector>

using namespace std;

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

  const int SERVO_DOWN_MIN_ANGLE = 0;
  const int SERVO_DOWN_MAX_ANGLE = 80;
  const int SERVO_UP_MIN_ANGLE = 10;
  const int SERVO_UP_MAX_ANGLE = 180;

  int servoDownAngle;
  int servoUpAngle;

  int currentDistance;

  // Agent
  const int ANGLE_STEP = 20;

  const int NUM_STATES_DOWN = (SERVO_DOWN_MAX_ANGLE - SERVO_DOWN_MIN_ANGLE) / ANGLE_STEP + 1;
  const int NUM_STATES_UP = (SERVO_UP_MAX_ANGLE - SERVO_UP_MIN_ANGLE) / ANGLE_STEP + 1;
  const int TOTAL_STATES = NUM_STATES_DOWN * NUM_STATES_UP;

  const int NUM_ACTION_STEPS = 3;
  const int ACTION_MULTIPLIERS[5] = { -4, -3, 0, 3, 4 };
  const int NUM_ACTIONS = NUM_ACTION_STEPS * NUM_ACTION_STEPS;

  vector<vector<float>> qTable{ TOTAL_STATES, vector<float>(NUM_ACTIONS, 0.0) };

  float alpha = 0.4;
  float gamma = 0.9;
  float beta = -0.06;
  float epsilon_prob = 1;
  float epsilon_prob_decay = 0.965;
  float min_epsilon_prob = 0.2;

  int learning_cycles = 50;
  int steps_per_cycle = 100;

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

  // RL Learning methods

  int getStateIndex() {
    int down_step = (servoDownAngle - SERVO_DOWN_MIN_ANGLE) / ANGLE_STEP;
    int up_step = (servoUpAngle - SERVO_UP_MIN_ANGLE) / ANGLE_STEP;
    return down_step * NUM_STATES_UP + up_step;
  }

  int getBestAction(int stateIndex) {
    int bestAction = 0;
    float maxQ = -1e9;
    for (int i = 0; i < NUM_ACTIONS; ++i) {
      if (qTable[stateIndex][i] > maxQ) {
        maxQ = qTable[stateIndex][i];
        bestAction = i;
      }
    }
    return bestAction;
  }

  int chooseAction(int stateIndex) {
    if (((float)random(100) / 100.0) < epsilon_prob) {
      return random(NUM_ACTIONS);
    } else {
      return getBestAction(stateIndex);
    }
  }

  void takeAction(int actionIndex) {
    int actionDown_idx = actionIndex / NUM_ACTION_STEPS;
    int actionUp_idx = actionIndex % NUM_ACTION_STEPS;

    int deltaDown = ACTION_MULTIPLIERS[actionDown_idx] * ANGLE_STEP;
    int deltaUp = ACTION_MULTIPLIERS[actionUp_idx] * ANGLE_STEP;

    int targetDown = servoDownAngle + deltaDown;
    int targetUp = servoUpAngle + deltaUp;

    moveServos(targetDown, targetUp);
  }

  float getReward(int final_dist) {
    return (float)(final_dist - currentDistance) + beta;
  }

  void doTraining() {
    printOnLCD("Training Started...");

    for (int c = 0; c < learning_cycles; ++c) {
      lcd.clear();
      lcd.print("Cycle: ");
      lcd.print(c + 1);
      lcd.print("/");
      lcd.print(learning_cycles);
      Serial.printf("Cycle %d\n", c + 1);

      float total_reward = 0;
      currentDistance = getDistance();

      for (int step = 0; step < steps_per_cycle; ++step) {
        int currentState_idx = getStateIndex();

        int action = chooseAction(currentState_idx);
        takeAction(action);
        delay(100);

        int final_dist = getDistance();
        if (final_dist < 0)
          final_dist = currentDistance;

        float reward = getReward(final_dist);

        total_reward += reward;
        currentDistance = final_dist;

        int nextState = getStateIndex();

        float old_q = qTable[currentState_idx][action];
        float next_max_q = qTable[nextState][getBestAction(nextState)];

        float new_q = old_q + alpha * (reward + gamma * next_max_q - old_q);
        qTable[currentState_idx][action] = new_q;

        if (step % 20 == 0) {
          lcd.setCursor(0, 1);
          lcd.print("Step: ");
          lcd.print(step);
        }
      }

      if (epsilon_prob > min_epsilon_prob)
        epsilon_prob *= epsilon_prob_decay;

      lcd.clear();
      lcd.print("Total Reward: ");
      lcd.print(total_reward);
      Serial.printf("Total Reward: %.2f\n", c + 1, total_reward);
      delay(3000);
    }

    printOnLCD("Training Complete!");
  }

  void doLearnedBehavior() {
    printOnLCD("Running Learned Behavior...");

    while (true) {
      int currentState_idx = getStateIndex();
      int action = getBestAction(currentState_idx);
      takeAction(action);
    }
  }

  // Setup and Hardwere methods

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
  }

  int getDistance() {
    const int num_readings = 5;
    int readings[num_readings];
    long total = 0;

    for (int i = 0; i < num_readings; i++) {
      readings[i] = sonar.ping_cm();
      delay(20); // Small delay between pings
    }

    // Simple average, ignoring errors (0)
    int count = 0;
    for (int i = 0; i < num_readings; i++) {
      if (readings[i] > 0) {
        total += readings[i];
        count++;
      }
    }

    if (count == 0) {
      Serial.println("Warning: No echo received from SRF module.");
      return -1; [cite: 59]
    }

    int distance = total / count;

    Serial.print("Avg Distance: ");
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

  void moveServos(int targetDown, int targetUp, int stepDelay = 10, int stepSize = 2) {
    targetDown = constrain(targetDown, SERVO_DOWN_MIN_ANGLE, SERVO_DOWN_MAX_ANGLE);
    targetUp = constrain(targetUp, SERVO_UP_MIN_ANGLE, SERVO_UP_MAX_ANGLE);

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
  vTaskDelay(1);
  robot.doTraining();
  robot.doLearnedBehavior();
}