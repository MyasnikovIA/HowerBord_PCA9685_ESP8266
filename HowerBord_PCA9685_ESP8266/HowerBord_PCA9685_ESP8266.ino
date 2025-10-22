#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// WiFi credentials
const char* ssid = "XXXXXXXX";
const char* password = "XXXXXXXX";
String newHostname = "HowerBord_Hom_001";

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Константы для PWM
const uint16_t PWM_FREQ = 1000;
const uint16_t PWM_MIN = 0;
const uint16_t PWM_MAX = 4095;

// Номера каналов для двигателей
const uint8_t LEFT_SPEED_CH = 0;
const uint8_t LEFT_REVERSE_CH = 1;
const uint8_t RIGHT_SPEED_CH = 2;
const uint8_t RIGHT_REVERSE_CH = 3;
const uint8_t REEL_SPEED_CH = 4;
const uint8_t REEL_REVERSE_CH = 5;

// Структура для хранения состояния двигателя
struct MotorState {
  int speed;
  bool isReversed;
  uint8_t speed_ch;
  uint8_t reverse_ch;
  String name;
};

// Состояния двигателей
MotorState leftMotor = {0, false, LEFT_SPEED_CH, LEFT_REVERSE_CH, "LEFT"};
MotorState rightMotor = {0, false, RIGHT_SPEED_CH, RIGHT_REVERSE_CH, "RIGHT"};
MotorState reelMotor = {0, false, REEL_SPEED_CH, REEL_REVERSE_CH, "REEL"};

// Настройки движения
int currentSpeed = 20;
int provodSpeed = 150;

// Переменные для реле
bool relay1 = false;
bool relay2 = false;
bool relay3 = false;

// Sensor data
float pitch = 0, roll = 0, yaw = 0;
float lastSentPitch = 0, lastSentRoll = 0, lastSentYaw = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
bool calibrated = false;
unsigned long lastTime = 0;

// Относительный ноль
float zeroPitch = 0, zeroRoll = 0, zeroYaw = 0;
bool zeroSet = false;

// Накопленные углы (без ограничений)
double accumulatedPitch = 0, accumulatedRoll = 0, accumulatedYaw = 0;
float prevPitch = 0, prevRoll = 0, prevYaw = 0;
bool firstMeasurement = true;

// WebSocket connection management
bool clientConnected = false;
unsigned long lastDataSend = 0;
const unsigned long SEND_INTERVAL = 50;
const float CHANGE_THRESHOLD = 1.0;

// Функции управления двигателями
void setupMotorController() {
  Serial.println("Initializing motor controller...");
  
  Wire.begin();
  
  if (!pwm.begin()) {
    Serial.println("Could not find PCA9685 on default address 0x40");
    Serial.println("Trying alternative address 0x41...");
    pwm = Adafruit_PWMServoDriver(0x41);
    if (!pwm.begin()) {
      Serial.println("Could not find PCA9685 on address 0x41 either!");
      Serial.println("Check I2C connections!");
      while (1);
    }
  }
  
  pwm.setPWMFreq(PWM_FREQ);
  stopAllMotors();
  
  Serial.println("Motor controller ready");
}

void setMotorSpeed(MotorState &motor, int speedPercent) {
  speedPercent = constrain(speedPercent, 0, 100);
  uint16_t pwmValue = map(speedPercent, 0, 100, PWM_MIN, PWM_MAX);
  pwm.setPWM(motor.speed_ch, 0, pwmValue);
  motor.speed = speedPercent;
  
  Serial.print(motor.name);
  Serial.print(" motor speed: ");
  Serial.print(speedPercent);
  Serial.println("%");
}

void setMotorDirection(MotorState &motor, bool reverse) {
  motor.isReversed = reverse;
  uint16_t pwmValue = reverse ? PWM_MAX : PWM_MIN;
  pwm.setPWM(motor.reverse_ch, 0, pwmValue);
  
  Serial.print(motor.name);
  Serial.print(" motor direction: ");
  Serial.println(reverse ? "REVERSE" : "FORWARD");
}

void stopMotor(MotorState &motor) {
  pwm.setPWM(motor.speed_ch, 0, PWM_MIN);
  pwm.setPWM(motor.reverse_ch, 0, PWM_MIN);
  motor.speed = 0;
  motor.isReversed = false;
}

void stopAllMotors() {
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 0);
  }
  
  leftMotor.speed = 0;
  leftMotor.isReversed = false;
  rightMotor.speed = 0;
  rightMotor.isReversed = false;
  reelMotor.speed = 0;
  reelMotor.isReversed = false;
}

// Функции движения
void moveForward() {
  Serial.println("Moving FORWARD");
  setMotorDirection(leftMotor, true);
  setMotorDirection(rightMotor, false);
  setMotorSpeed(leftMotor, currentSpeed);
  setMotorSpeed(rightMotor, currentSpeed);
  String message = "MOVING:FORWARD";
  webSocket.broadcastTXT(message);
}

void moveBackward() {
  Serial.println("Moving BACKWARD");
  setMotorDirection(leftMotor, false);
  setMotorDirection(rightMotor, true);
  setMotorSpeed(leftMotor, currentSpeed);
  setMotorSpeed(rightMotor, currentSpeed);
  String message = "MOVING:BACKWARD";
  webSocket.broadcastTXT(message);
}

void moveLeft() {
  Serial.println("Turning LEFT");
  setMotorDirection(leftMotor, false);
  setMotorDirection(rightMotor, false);
  setMotorSpeed(leftMotor, currentSpeed);
  setMotorSpeed(rightMotor, currentSpeed);
  String message = "MOVING:LEFT";
  webSocket.broadcastTXT(message);
}

void moveRight() {
  Serial.println("Turning RIGHT");
  setMotorDirection(leftMotor, true);
  setMotorDirection(rightMotor, true);
  setMotorSpeed(leftMotor, currentSpeed);
  setMotorSpeed(rightMotor, currentSpeed);
  String message = "MOVING:RIGHT";
  webSocket.broadcastTXT(message);
}

void stopMovement() {
  Serial.println("STOPPING");
  stopAllMotors();
  String message = "MOVING:STOP";
  webSocket.broadcastTXT(message);
}

void setSpeed(int speedPercent) {
  currentSpeed = constrain(speedPercent, 0, 100);
  Serial.print("Speed set to: ");
  Serial.print(currentSpeed);
  Serial.println("%");
  
  if (leftMotor.speed > 0) setMotorSpeed(leftMotor, currentSpeed);
  if (rightMotor.speed > 0) setMotorSpeed(rightMotor, currentSpeed);
  
  String message = "SPEED:" + String(currentSpeed);
  webSocket.broadcastTXT(message);
}

// Установка относительного нуля
void setZeroPoint() {
  zeroPitch = pitch;
  zeroRoll = roll;
  zeroYaw = yaw;
  zeroSet = true;
  
  // Сбрасываем накопленные углы при установке нуля
  accumulatedPitch = 0;
  accumulatedRoll = 0;
  accumulatedYaw = 0;
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
  
  Serial.println("Zero point set");
  Serial.print("Zero Pitch: "); Serial.print(zeroPitch);
  Serial.print(" Roll: "); Serial.print(zeroRoll);
  Serial.print(" Yaw: "); Serial.println(zeroYaw);
  
  String message = "ZERO_SET:PITCH:" + String(zeroPitch, 2) + 
                   ",ROLL:" + String(zeroRoll, 2) + 
                   ",YAW:" + String(zeroYaw, 2);
  webSocket.broadcastTXT(message);
}

// Сброс относительного нуля
void resetZeroPoint() {
  zeroPitch = 0;
  zeroRoll = 0;
  zeroYaw = 0;
  zeroSet = false;
  
  accumulatedPitch = 0;
  accumulatedRoll = 0;
  accumulatedYaw = 0;
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
  
  Serial.println("Zero point reset");
  webSocket.broadcastTXT("ZERO_RESET");
}

// Расчет накопленных углов (без ограничений)
void updateAccumulatedAngles() {
  if (firstMeasurement) {
    prevPitch = pitch;
    prevRoll = roll;
    prevYaw = yaw;
    firstMeasurement = false;
    return;
  }
  
  // Вычисляем разницу углов с учетом переходов через 180/-180
  float deltaPitch = pitch - prevPitch;
  float deltaRoll = roll - prevRoll;
  float deltaYaw = yaw - prevYaw;
  
  // Корректируем разницу для переходов через границу ±180
  if (deltaPitch > 180) deltaPitch -= 360;
  else if (deltaPitch < -180) deltaPitch += 360;
  
  if (deltaRoll > 180) deltaRoll -= 360;
  else if (deltaRoll < -180) deltaRoll += 360;
  
  if (deltaYaw > 180) deltaYaw -= 360;
  else if (deltaYaw < -180) deltaYaw += 360;
  
  // Накопление углов
  accumulatedPitch += deltaPitch;
  accumulatedRoll += deltaRoll;
  accumulatedYaw += deltaYaw;
  
  prevPitch = pitch;
  prevRoll = roll;
  prevYaw = yaw;
}

// Получение относительных углов (без ограничений)
double getRelativePitch() {
  if (!zeroSet) return accumulatedPitch;
  return accumulatedPitch - zeroPitch;
}

double getRelativeRoll() {
  if (!zeroSet) return accumulatedRoll;
  return accumulatedRoll - zeroRoll;
}

double getRelativeYaw() {
  if (!zeroSet) return accumulatedYaw;
  return accumulatedYaw - zeroYaw;
}

// Управление двигателем смотки
void setReelMotor(int speed, int direction) {
  if (direction == 1) {
    setMotorDirection(reelMotor, false);
  } else if (direction == -1) {
    setMotorDirection(reelMotor, true);
  }
  
  int speedPercent = map(speed, 0, 1023, 0, 100);
  setMotorSpeed(reelMotor, speedPercent);
  
  Serial.print("Reel motor: speed=");
  Serial.print(speedPercent);
  Serial.print("%, direction=");
  Serial.println(direction == 1 ? "FORWARD" : "BACKWARD");
}

// Управление реле
void setRelay(int relayNum, bool state) {
  switch(relayNum) {
    case 1:
      relay1 = state;
      Serial.print("Relay 1: ");
      Serial.println(state ? "ON" : "OFF");
      break;
    case 2:
      relay2 = state;
      Serial.print("Relay 2: ");
      Serial.println(state ? "ON" : "OFF");
      break;
    case 3:
      relay3 = state;
      Serial.print("Relay 3: ");
      Serial.println(state ? "ON" : "OFF");
      break;
  }
  
  String message = "RELAY" + String(relayNum) + ":" + (state ? "ON" : "OFF");
  webSocket.broadcastTXT(message);
}

void calibrateSensor() {
  Serial.println("Calibrating...");
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(2);
  }
  
  gyroOffsetX = sumX / 500;
  gyroOffsetY = sumY / 500;
  gyroOffsetZ = sumZ / 500;
  calibrated = true;
  
  Serial.println("Calibration complete");
}

void sendSensorData() {
  // Обновляем накопленные углы
  updateAccumulatedAngles();
  
  // Получаем относительные углы
  double relPitch = getRelativePitch();
  double relRoll = getRelativeRoll();
  double relYaw = getRelativeYaw();
  
  String data = "PITCH:" + String(pitch, 1) + 
                ",ROLL:" + String(roll, 1) + 
                ",YAW:" + String(yaw, 1) +
                ",REL_PITCH:" + String(relPitch, 2) +
                ",REL_ROLL:" + String(relRoll, 2) +
                ",REL_YAW:" + String(relYaw, 2) +
                ",ACC_PITCH:" + String(accumulatedPitch, 2) +
                ",ACC_ROLL:" + String(accumulatedRoll, 2) +
                ",ACC_YAW:" + String(accumulatedYaw, 2) +
                ",ZERO_SET:" + String(zeroSet ? "true" : "false");
  
  webSocket.broadcastTXT(data);
  lastSentPitch = pitch;
  lastSentRoll = roll;
  lastSentYaw = yaw;
}

bool dataChanged() {
  return (abs(pitch - lastSentPitch) >= CHANGE_THRESHOLD ||
          abs(roll - lastSentRoll) >= CHANGE_THRESHOLD ||
          abs(yaw - lastSentYaw) >= CHANGE_THRESHOLD);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      clientConnected = (webSocket.connectedClients() > 0);
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        clientConnected = true;
        sendSensorData();
        String speedMessage = "SPEED:" + String(currentSpeed);
        webSocket.broadcastTXT(speedMessage);
      }
      break;
      
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        Serial.printf("[%u] Received: %s\n", num, message);
        
        if (message == "GET_DATA") {
          sendSensorData();
        }
        else if (message == "RECALIBRATE") {
          calibrated = false;
          calibrateSensor();
          String calMessage = "RECALIBRATION_COMPLETE";
          webSocket.broadcastTXT(calMessage);
        }
        else if (message == "RESET_ANGLES") {
          pitch = 0; roll = 0; yaw = 0;
          lastSentPitch = 0; lastSentRoll = 0; lastSentYaw = 0;
          resetZeroPoint();
          String resetMessage = "ANGLES_RESET";
          webSocket.broadcastTXT(resetMessage);
          sendSensorData();
        }
        else if (message == "MOVE_FORWARD") {
          moveForward();
        }
        else if (message == "MOVE_BACKWARD") {
          moveBackward();
        }
        else if (message == "MOVE_LEFT") {
          moveLeft();
        }
        else if (message == "MOVE_RIGHT") {
          moveRight();
        }
        else if (message == "STOP") {
          stopMovement();
        }
        else if (message.startsWith("SPEED:")) {
          int newSpeed = message.substring(6).toInt();
          setSpeed(newSpeed);
        }
        else if (message == "SET_ZERO") {
          setZeroPoint();
          webSocket.broadcastTXT("ZERO_POINT_SET");
        }
        else if (message == "RESET_ZERO") {
          resetZeroPoint();
          webSocket.broadcastTXT("ZERO_POINT_RESET");
        }
        else if (message.startsWith("GAMEPAD:")) {
          processGamepadCommand(message);
        }
      }
      break;
  }
}

void processGamepadCommand(String command) {
  if (command.startsWith("GAMEPAD:BUTTON:")) {
    String buttonData = command.substring(15);
    int colonPos = buttonData.indexOf(':');
    if (colonPos != -1) {
      String button = buttonData.substring(0, colonPos);
      bool state = buttonData.substring(colonPos + 1).toInt() == 1;
      
      if (button == "X" && state) {
        // Увеличить скорость через слайдер
        int newSpeed = min(currentSpeed + 20, 100);
        setSpeed(newSpeed);
        String message = "SLIDER_UPDATE:" + String(newSpeed);
        webSocket.broadcastTXT(message);
      }
      else if (button == "Y" && state) {
        // Уменьшить скорость через слайдер
        int newSpeed = max(currentSpeed - 20, 0);
        setSpeed(newSpeed);
        String message = "SLIDER_UPDATE:" + String(newSpeed);
        webSocket.broadcastTXT(message);
      }
      else if (button == "R" && state) {
        setReelMotor(provodSpeed, 1);
      }
      else if (button == "R" && !state) {
        setReelMotor(0, 1);
      }
      else if (button == "SELECT") {
        setRelay(1, state);
      }
      else if (button == "A") {
        setRelay(2, state);
      }
      else if (button == "B") {
        setRelay(3, state);
      }
    }
  }
  else if (command.startsWith("GAMEPAD:AXIS:")) {
    String axisData = command.substring(13);
    int colonPos = axisData.indexOf(':');
    if (colonPos != -1) {
      String axis = axisData.substring(0, colonPos);
      float value = axisData.substring(colonPos + 1).toFloat();
      
      if (axis == "0") {
        handleAxisMovement(value, 0);
      }
      else if (axis == "1") {
        handleAxisMovement(value, 1);
      }
    }
  }
  else if (command.startsWith("GAMEPAD:PROVOD_SPEED:")) {
    provodSpeed = command.substring(21).toInt();
    String message = "PROVOD_SPEED:" + String(provodSpeed);
    webSocket.broadcastTXT(message);
  }
}

void handleAxisMovement(float value, int axis) {
  int direction = 0;
  
  if (value < -0.1) {
    if (axis == 1) {
      direction = 1; // Вперед
    } else {
      direction = 3; // Влево
    }
  } 
  else if (value > 0.1) {
    if (axis == 1) {
      direction = 2; // Назад
    } else {
      direction = 4; // Вправо
    }
  }
  
  // Используем текущую скорость из слайдера
  int speedPercent = currentSpeed;
  
  switch (direction) {
    case 1:
      setMotorDirection(leftMotor, true);
      setMotorDirection(rightMotor, false);
      setMotorSpeed(leftMotor, speedPercent);
      setMotorSpeed(rightMotor, speedPercent);
      break;
    case 2:
      setMotorDirection(leftMotor, false);
      setMotorDirection(rightMotor, true);
      setMotorSpeed(leftMotor, speedPercent);
      setMotorSpeed(rightMotor, speedPercent);
      break;
    case 3:
      setMotorDirection(leftMotor, false);
      setMotorDirection(rightMotor, false);
      setMotorSpeed(leftMotor, speedPercent);
      setMotorSpeed(rightMotor, speedPercent);
      break;
    case 4:
      setMotorDirection(leftMotor, true);
      setMotorDirection(rightMotor, true);
      setMotorSpeed(leftMotor, speedPercent);
      setMotorSpeed(rightMotor, speedPercent);
      break;
    default:
      stopMovement();
      break;
  }
}

void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  server.sendHeader("Content-Type", "text/html; charset=utf-8");
}

void handleOptions() {
  addCORSHeaders();
  server.send(200, "text/plain", "");
}

void handleAPIStatus() {
  addCORSHeaders();
  String json = "{";
  json += "\"status\":\"running\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"speed\":" + String(currentSpeed) + ",";
  json += "\"provodSpeed\":" + String(provodSpeed) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"yaw\":" + String(yaw, 2) + ",";
  json += "\"relPitch\":" + String(getRelativePitch(), 2) + ",";
  json += "\"relRoll\":" + String(getRelativeRoll(), 2) + ",";
  json += "\"relYaw\":" + String(getRelativeYaw(), 2) + ",";
  json += "\"accPitch\":" + String(accumulatedPitch, 2) + ",";
  json += "\"accRoll\":" + String(accumulatedRoll, 2) + ",";
  json += "\"accYaw\":" + String(accumulatedYaw, 2) + ",";
  json += "\"zeroSet\":" + String(zeroSet ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleAPIMove() {
  addCORSHeaders();
  String direction = server.arg("direction");
  String response = "{\"status\":\"ok\",\"direction\":\"" + direction + "\"}";
  
  if (direction == "forward") {
    moveForward();
  } else if (direction == "backward") {
    moveBackward();
  } else if (direction == "left") {
    moveLeft();
  } else if (direction == "right") {
    moveRight();
  } else if (direction == "stop") {
    stopMovement();
  } else {
    response = "{\"status\":\"error\",\"message\":\"Invalid direction\"}";
    server.send(400, "application/json", response);
    return;
  }
  
  server.send(200, "application/json", response);
}

void handleAPISpeed() {
  addCORSHeaders();
  String speedStr = server.arg("speed");
  int newSpeed = speedStr.toInt();
  
  if (newSpeed >= 0 && newSpeed <= 100) {
    setSpeed(newSpeed);
    String response = "{\"status\":\"ok\",\"speed\":" + String(currentSpeed) + "}";
    server.send(200, "application/json", response);
  } else {
    String response = "{\"status\":\"error\",\"message\":\"Speed must be between 0-100\"}";
    server.send(400, "application/json", response);
  }
}

void handleSetZero() {
  addCORSHeaders();
  setZeroPoint();
  String response = "{\"status\":\"ok\",\"message\":\"Zero point set\"}";
  server.send(200, "application/json", response);
}

void handleResetZero() {
  addCORSHeaders();
  resetZeroPoint();
  String response = "{\"status\":\"ok\",\"message\":\"Zero point reset\"}";
  server.send(200, "application/json", response);
}

void handleGamepadPost() {
  addCORSHeaders();
  
  if (server.hasArg("provodSpeed") && server.hasArg("provod")) {
    int speed = server.arg("provodSpeed").toInt();
    int direction = server.arg("provod").toInt();
    setReelMotor(speed, direction);
    server.send(200, "text/plain", "OK");
    return;
  }
  
  if (server.hasArg("button_a")) {
    setRelay(2, server.arg("button_a").toInt() == 1);
  }
  if (server.hasArg("button_b")) {
    setRelay(3, server.arg("button_b").toInt() == 1);
  }
  if (server.hasArg("button_select")) {
    setRelay(1, server.arg("button_select").toInt() == 1);
  }
  
  if (server.hasArg("up") && server.hasArg("dir")) {
    int speedValue = server.arg("up").toInt();
    int direction = server.arg("dir").toInt();
    int speedPercent = map(speedValue, 0, 1023, 0, 100);
    
    switch (direction) {
      case 1:
        moveForward();
        break;
      case 2:
        moveBackward();
        break;
      case 3:
        moveLeft();
        break;
      case 4:
        moveRight();
        break;
      default:
        stopMovement();
        break;
    }
  }
  
  server.send(200, "text/plain", "OK");
}

// HTML части
const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>MPU6050 + Motor Control + Gamepad</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; }
        .data { background: #f0f0f0; padding: 15px; margin: 10px 0; border-radius: 5px; }
        .value { font-size: 24px; font-weight: bold; color: #2c3e50; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .connected { background: #d4edda; color: #155724; }
        .disconnected { background: #f8d7da; color: #721c24; }
        .controls { margin: 20px 0; }
        button { padding: 10px 15px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; }
        .btn-primary { background: #007bff; color: white; }
        .btn-warning { background: #ffc107; color: black; }
        .btn-danger { background: #dc3545; color: white; }
        .btn-success { background: #28a745; color: white; }
        .btn-info { background: #17a2b8; color: white; }
        .movement-controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 20px 0; }
        .movement-btn { padding: 20px; font-size: 18px; }
        .speed-control { margin: 20px 0; }
        .speed-slider { width: 100%; margin: 10px 0; }
        .api-section { background: #e9ecef; padding: 15px; margin: 20px 0; border-radius: 5px; }
        .gamepad-info { background: #fff3cd; padding: 15px; margin: 20px 0; border-radius: 5px; }
        .visualization { background: #2c3e50; color: white; padding: 20px; border-radius: 10px; margin: 20px 0; }
        .cube-container { width: 300px; height: 300px; margin: 20px auto; perspective: 1000px; }
        .cube { width: 100%; height: 100%; position: relative; transform-style: preserve-3d; transition: transform 0.1s ease-out; }
        .face { position: absolute; width: 300px; height: 300px; border: 3px solid #34495e; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: bold; color: white; background: rgba(52, 152, 219, 0.8); }
        .front { transform: rotateY(0deg) translateZ(150px); background: rgba(231, 76, 60, 0.8); }
        .back { transform: rotateY(180deg) translateZ(150px); background: rgba(52, 152, 219, 0.8); }
        .right { transform: rotateY(90deg) translateZ(150px); background: rgba(46, 204, 113, 0.8); }
        .left { transform: rotateY(-90deg) translateZ(150px); background: rgba(155, 89, 182, 0.8); }
        .top { transform: rotateX(90deg) translateZ(150px); background: rgba(241, 196, 15, 0.8); }
        .bottom { transform: rotateX(-90deg) translateZ(150px); background: rgba(230, 126, 34, 0.8); }
        .zero-controls { background: #e8f5e8; padding: 15px; margin: 20px 0; border-radius: 5px; border-left: 4px solid #28a745; }
    </style>
</head>
)rawliteral";

const char HTML_BODY_START[] PROGMEM = R"rawliteral(
<body>
    <h1>MPU6050 + Motor Control + Gamepad</h1>
    
    <div class="status" id="status">Disconnected</div>
    
    <div class="data">
        <div>Pitch: <span class="value" id="pitch">0</span>&deg;</div>
        <div>Roll: <span class="value" id="roll">0</span>&deg;</div>
        <div>Yaw: <span class="value" id="yaw">0</span>&deg;</div>
        <div>Relative Pitch: <span class="value" id="relPitch">0</span>&deg;</div>
        <div>Relative Roll: <span class="value" id="relRoll">0</span>&deg;</div>
        <div>Relative Yaw: <span class="value" id="relYaw">0</span>&deg;</div>
        <div>Accumulated Pitch: <span class="value" id="accPitch">0</span>&deg;</div>
        <div>Accumulated Roll: <span class="value" id="accRoll">0</span>&deg;</div>
        <div>Accumulated Yaw: <span class="value" id="accYaw">0</span>&deg;</div>
        <div>Current Speed: <span class="value" id="currentSpeed">20</span>%</div>
        <div>Provod Speed: <span class="value" id="provodSpeed">150</span></div>
        <div>Zero Point: <span class="value" id="zeroStatus">Not Set</span></div>
    </div>
)rawliteral";

const char HTML_ZERO_CONTROLS[] PROGMEM = R"rawliteral(
    <div class="zero-controls">
        <h3>Zero Point Control</h3>
        <button class="btn-success" onclick="sendCommand('SET_ZERO')">Set Zero Point</button>
        <button class="btn-warning" onclick="sendCommand('RESET_ZERO')">Reset Zero</button>
        <button class="btn-danger" onclick="sendCommand('RESET_ANGLES')">Reset All Angles</button>
        <div style="font-size: 12px; color: #666; margin-top: 10px;">
            Zero point allows you to set a reference position. Relative angles show deviation from zero point.
            Accumulated angles show total rotation without limits (can exceed 360°).
        </div>
    </div>
)rawliteral";

const char HTML_VISUALIZATION[] PROGMEM = R"rawliteral(
    <div class="visualization">
        <h3>3D Platform Visualization</h3>
        <div class="cube-container">
            <div class="cube" id="cube">
                <div class="face front">FRONT</div>
                <div class="face back">BACK</div>
                <div class="face right">RIGHT</div>
                <div class="face left">LEFT</div>
                <div class="face top">TOP</div>
                <div class="face bottom">BOTTOM</div>
            </div>
        </div>
    </div>
)rawliteral";

const char HTML_SPEED_CONTROL[] PROGMEM = R"rawliteral(
    <div class="speed-control">
        <h3>Speed Control</h3>
        <input type="range" min="0" max="100" value="20" class="speed-slider" id="speedSlider">
        <div>Speed: <span id="speedValue">20</span>%</div>
    </div>
)rawliteral";

const char HTML_MOVEMENT_CONTROLS[] PROGMEM = R"rawliteral(
    <div class="movement-controls">
        <div></div>
        <button class="btn-success movement-btn" onclick="sendMoveCommand('MOVE_FORWARD')">&uarr; Forward</button>
        <div></div>
        
        <button class="btn-info movement-btn" onclick="sendMoveCommand('MOVE_LEFT')">&larr; Left</button>
        <button class="btn-danger movement-btn" onclick="sendCommand('STOP')">Stop</button>
        <button class="btn-info movement-btn" onclick="sendMoveCommand('MOVE_RIGHT')">Right &rarr;</button>
        
        <div></div>
        <button class="btn-success movement-btn" onclick="sendMoveCommand('MOVE_BACKWARD')">&darr; Backward</button>
        <div></div>
    </div>
)rawliteral";

const char HTML_CONTROLS[] PROGMEM = R"rawliteral(
    <div class="controls">
        <button class="btn-primary" onclick="sendCommand('GET_DATA')">Get Sensor Data</button>
        <button class="btn-warning" onclick="sendCommand('RECALIBRATE')">Recalibrate</button>
    </div>
)rawliteral";

const char HTML_GAMEPAD_INFO[] PROGMEM = R"rawliteral(
    <div class="gamepad-info">
        <h3>Gamepad Control (Super Nintendo)</h3>
        <p><strong>X</strong> - Increase speed (moves slider)</p>
        <p><strong>Y</strong> - Decrease speed (moves slider)</p>
        <p><strong>R</strong> - Pull wire</p>
        <p><strong>SELECT</strong> - Relay 1</p>
        <p><strong>A</strong> - Relay 2</p>
        <p><strong>B</strong> - Relay 3</p>
        
        <div class="speed-control">
            <h4>Wire Speed Control</h4>
            <input type="range" min="0" max="1023" value="150" class="speed-slider" id="provodSpeedSlider">
            <div>Wire Speed: <span id="provodSpeedValue">150</span></div>
        </div>
    </div>
)rawliteral";

const char HTML_API_SECTION[] PROGMEM = R"rawliteral(
    <div class="api-section">
        <h3>REST API Endpoints (for cross-domain requests):</h3>
        <p><strong>GET /api/status</strong> - Get device status</p>
        <p><strong>POST /api/move?direction=forward|backward|left|right|stop</strong> - Control movement</p>
        <p><strong>POST /api/speed?speed=0-100</strong> - Set speed</p>
        <p><strong>POST /api/setZero</strong> - Set zero point</p>
        <p><strong>POST /api/resetZero</strong> - Reset zero point</p>
        <p><strong>POST /post</strong> - Gamepad commands (compatible with existing system)</p>
    </div>
    
    <div class="data">
        <h3>Last Message:</h3>
        <div id="lastMessage">No data received</div>
    </div>
)rawliteral";

const char HTML_SCRIPT_START[] PROGMEM = R"rawliteral(
    <script>
        let ws = null;
        let statusDiv = document.getElementById('status');
        let lastMessageDiv = document.getElementById('lastMessage');
        let speedSlider = document.getElementById('speedSlider');
        let speedValue = document.getElementById('speedValue');
        let currentSpeedSpan = document.getElementById('currentSpeed');
        let provodSpeedSlider = document.getElementById('provodSpeedSlider');
        let provodSpeedValue = document.getElementById('provodSpeedValue');
        let provodSpeedSpan = document.getElementById('provodSpeed');
        let cube = document.getElementById('cube');
        let zeroStatusSpan = document.getElementById('zeroStatus');
        
        // Gamepad variables
        let provodSpeed = 150;
        
        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = protocol + '//' + window.location.hostname + ':81';
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                statusDiv.textContent = 'Connected';
                statusDiv.className = 'status connected';
                startGamepadPolling();
            };
            
            ws.onmessage = function(event) {
                console.log('Received:', event.data);
                lastMessageDiv.textContent = event.data;
                
                // Parse sensor data
                if (event.data.includes('PITCH:') && event.data.includes('ROLL:') && event.data.includes('YAW:')) {
                    const data = event.data;
                    
                    // Parse all data fields
                    const pitchMatch = data.match(/PITCH:([-\d.]+)/);
                    const rollMatch = data.match(/ROLL:([-\d.]+)/);
                    const yawMatch = data.match(/YAW:([-\d.]+)/);
                    const relPitchMatch = data.match(/REL_PITCH:([-\d.]+)/);
                    const relRollMatch = data.match(/REL_ROLL:([-\d.]+)/);
                    const relYawMatch = data.match(/REL_YAW:([-\d.]+)/);
                    const accPitchMatch = data.match(/ACC_PITCH:([-\d.]+)/);
                    const accRollMatch = data.match(/ACC_ROLL:([-\d.]+)/);
                    const accYawMatch = data.match(/ACC_YAW:([-\d.]+)/);
                    const zeroSetMatch = data.match(/ZERO_SET:(true|false)/);
                    
                    if (pitchMatch) {
                        const pitch = parseFloat(pitchMatch[1]);
                        document.getElementById('pitch').textContent = pitch.toFixed(1);
                    }
                    if (rollMatch) {
                        const roll = parseFloat(rollMatch[1]);
                        document.getElementById('roll').textContent = roll.toFixed(1);
                    }
                    if (yawMatch) {
                        const yaw = parseFloat(yawMatch[1]);
                        document.getElementById('yaw').textContent = yaw.toFixed(1);
                    }
                    if (relPitchMatch) {
                        const relPitch = parseFloat(relPitchMatch[1]);
                        document.getElementById('relPitch').textContent = relPitch.toFixed(1);
                    }
                    if (relRollMatch) {
                        const relRoll = parseFloat(relRollMatch[1]);
                        document.getElementById('relRoll').textContent = relRoll.toFixed(1);
                    }
                    if (relYawMatch) {
                        const relYaw = parseFloat(relYawMatch[1]);
                        document.getElementById('relYaw').textContent = relYaw.toFixed(1);
                    }
                    if (accPitchMatch) {
                        const accPitch = parseFloat(accPitchMatch[1]);
                        document.getElementById('accPitch').textContent = accPitch.toFixed(1);
                    }
                    if (accRollMatch) {
                        const accRoll = parseFloat(accRollMatch[1]);
                        document.getElementById('accRoll').textContent = accRoll.toFixed(1);
                    }
                    if (accYawMatch) {
                        const accYaw = parseFloat(accYawMatch[1]);
                        document.getElementById('accYaw').textContent = accYaw.toFixed(1);
                    }
                    if (zeroSetMatch) {
                        const zeroSet = zeroSetMatch[1] === 'true';
                        zeroStatusSpan.textContent = zeroSet ? 'Set' : 'Not Set';
                        zeroStatusSpan.style.color = zeroSet ? '#28a745' : '#dc3545';
                    }
                    
                    // Update 3D visualization
                    update3DVisualization(pitchMatch[1], rollMatch[1], yawMatch[1]);
                }
                
                // Parse speed data
                if (event.data.startsWith('SPEED:')) {
                    const speed = event.data.substring(6);
                    currentSpeedSpan.textContent = speed;
                    speedSlider.value = speed;
                    speedValue.textContent = speed;
                }
                
                // Parse slider update
                if (event.data.startsWith('SLIDER_UPDATE:')) {
                    const speed = event.data.substring(14);
                    speedSlider.value = speed;
                    speedValue.textContent = speed;
                }
                
                // Parse provod speed data
                if (event.data.startsWith('PROVOD_SPEED:')) {
                    const speed = event.data.substring(13);
                    provodSpeed = parseInt(speed);
                    provodSpeedSlider.value = speed;
                    provodSpeedValue.textContent = speed;
                    provodSpeedSpan.textContent = speed;
                }
                
                // Parse zero point messages
                if (event.data === 'ZERO_POINT_SET') {
                    zeroStatusSpan.textContent = 'Set';
                    zeroStatusSpan.style.color = '#28a745';
                    showNotification('Zero point set successfully', 'success');
                }
                if (event.data === 'ZERO_POINT_RESET') {
                    zeroStatusSpan.textContent = 'Not Set';
                    zeroStatusSpan.style.color = '#dc3545';
                    showNotification('Zero point reset', 'info');
                }
                if (event.data.startsWith('ZERO_SET:')) {
                    zeroStatusSpan.textContent = 'Set';
                    zeroStatusSpan.style.color = '#28a745';
                }
                if (event.data === 'ZERO_RESET') {
                    zeroStatusSpan.textContent = 'Not Set';
                    zeroStatusSpan.style.color = '#dc3545';
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                statusDiv.textContent = 'Disconnected';
                statusDiv.className = 'status disconnected';
                stopGamepadPolling();
                
                setTimeout(connectWebSocket, 2000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function update3DVisualization(pitch, roll, yaw) {
            // Apply rotation to the 3D cube
            cube.style.transform = `rotateX(${roll}deg) rotateY(${yaw}deg) rotateZ(${pitch}deg)`;
        }
        
        function sendCommand(command) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(command);
                console.log('Sent command:', command);
            } else {
                console.log('WebSocket not connected');
            }
        }
        
        // Special function for movement commands that sends speed first
        function sendMoveCommand(command) {
            // Get current speed from slider
            const currentSpeed = speedSlider.value;
            // Send speed update first
            sendCommand('SPEED:' + currentSpeed);
            // Then send movement command
            sendCommand(command);
        }
        
        function showNotification(message, type = 'info') {
            // Create notification element
            const notification = document.createElement('div');
            notification.textContent = message;
            notification.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                padding: 15px 20px;
                border-radius: 5px;
                color: white;
                font-weight: bold;
                z-index: 1000;
                background: ${type === 'success' ? '#28a745' : type === 'error' ? '#dc3545' : '#17a2b8'};
            `;
            
            document.body.appendChild(notification);
            
            // Remove after 3 seconds
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 3000);
        }
)rawliteral";

const char HTML_SCRIPT_END[] PROGMEM = R"rawliteral(
        // Speed slider event
        speedSlider.addEventListener('input', function() {
            const speed = this.value;
            speedValue.textContent = speed;
            sendCommand('SPEED:' + speed);
        });
        
        // Provod speed slider event
        provodSpeedSlider.addEventListener('input', function() {
            const speed = this.value;
            provodSpeed = parseInt(speed);
            provodSpeedValue.textContent = speed;
            provodSpeedSpan.textContent = speed;
            sendCommand('GAMEPAD:PROVOD_SPEED:' + speed);
        });
        
        // Keyboard controls
        document.addEventListener('keydown', function(event) {
            switch(event.key) {
                case 'ArrowUp':
                    sendMoveCommand('MOVE_FORWARD');
                    break;
                case 'ArrowDown':
                    sendMoveCommand('MOVE_BACKWARD');
                    break;
                case 'ArrowLeft':
                    sendMoveCommand('MOVE_LEFT');
                    break;
                case 'ArrowRight':
                    sendMoveCommand('MOVE_RIGHT');
                    break;
                case ' ':
                    sendCommand('STOP');
                    break;
            }
        });
        
        // Gamepad polling
        let gamepadPollInterval;
        const previousButtonStates = {};
        const previousAxesStates = {};
        
        function startGamepadPolling() {
            gamepadPollInterval = setInterval(pollGamepads, 100);
        }
        
        function stopGamepadPolling() {
            if (gamepadPollInterval) {
                clearInterval(gamepadPollInterval);
            }
        }
        
        function pollGamepads() {
            const gamepads = navigator.getGamepads();
            
            for (const gamepad of gamepads) {
                if (!gamepad) continue;
                
                if (!previousButtonStates[gamepad.index]) {
                    previousButtonStates[gamepad.index] = [];
                }
                if (!previousAxesStates[gamepad.index]) {
                    previousAxesStates[gamepad.index] = [];
                }
                
                // Check buttons
                gamepad.buttons.forEach((button, index) => {
                    const currentValue = button.value;
                    const previousValue = previousButtonStates[gamepad.index][index];
                    
                    if (previousValue !== currentValue && currentValue > 0.5) {
                        let buttonName = '';
                        switch(index) {
                            case 0: buttonName = 'A'; break;
                            case 1: buttonName = 'B'; break;
                            case 2: buttonName = 'X'; break;
                            case 3: buttonName = 'Y'; break;
                            case 4: buttonName = 'L'; break;
                            case 5: buttonName = 'R'; break;
                            case 8: buttonName = 'SELECT'; break;
                            case 9: buttonName = 'START'; break;
                        }
                        
                        if (buttonName) {
                            sendCommand('GAMEPAD:BUTTON:' + buttonName + ':1');
                            
                            // Handle X and Y buttons for speed control
                            if (buttonName === 'X') {
                                // Increase speed
                                let newSpeed = Math.min(parseInt(speedSlider.value) + 20, 100);
                                speedSlider.value = newSpeed;
                                speedValue.textContent = newSpeed;
                                sendCommand('SPEED:' + newSpeed);
                            } else if (buttonName === 'Y') {
                                // Decrease speed
                                let newSpeed = Math.max(parseInt(speedSlider.value) - 20, 0);
                                speedSlider.value = newSpeed;
                                speedValue.textContent = newSpeed;
                                sendCommand('SPEED:' + newSpeed);
                            }
                        }
                        
                        previousButtonStates[gamepad.index][index] = currentValue;
                    } else if (previousValue !== currentValue && currentValue <= 0.5) {
                        let buttonName = '';
                        switch(index) {
                            case 0: buttonName = 'A'; break;
                            case 1: buttonName = 'B'; break;
                            case 2: buttonName = 'X'; break;
                            case 3: buttonName = 'Y'; break;
                            case 4: buttonName = 'L'; break;
                            case 5: buttonName = 'R'; break;
                            case 8: buttonName = 'SELECT'; break;
                            case 9: buttonName = 'START'; break;
                        }
                        
                        if (buttonName) {
                            sendCommand('GAMEPAD:BUTTON:' + buttonName + ':0');
                        }
                        previousButtonStates[gamepad.index][index] = currentValue;
                    }
                });
                
                // Check axes (analog joystick always enabled)
                gamepad.axes.forEach((axis, index) => {
                    const currentValue = axis;
                    const previousValue = previousAxesStates[gamepad.index][index];
                    
                    const roundedCurrent = Math.round(currentValue * 100) / 100;
                    const roundedPrevious = Math.round(previousValue * 100) / 100;
                    
                    if (roundedPrevious !== roundedCurrent) {
                        sendCommand('GAMEPAD:AXIS:' + index + ':' + roundedCurrent);
                        previousAxesStates[gamepad.index][index] = currentValue;
                    }
                });
            }
        }
        
        // Initialize when page loads
        window.addEventListener('load', function() {
            connectWebSocket();
        });
        
        window.addEventListener("gamepadconnected", function(e) {
            console.log("Gamepad connected:", e.gamepad.id);
        });
        
        window.addEventListener("gamepaddisconnected", function(e) {
            console.log("Gamepad disconnected:", e.gamepad.id);
        });
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  String html = FPSTR(HTML_HEAD);
  html += FPSTR(HTML_BODY_START);
  html += FPSTR(HTML_ZERO_CONTROLS);
  html += FPSTR(HTML_VISUALIZATION);
  html += FPSTR(HTML_SPEED_CONTROL);
  html += FPSTR(HTML_MOVEMENT_CONTROLS);
  html += FPSTR(HTML_CONTROLS);
  html += FPSTR(HTML_GAMEPAD_INFO);
  html += FPSTR(HTML_API_SECTION);
  html += FPSTR(HTML_SCRIPT_START);
  html += FPSTR(HTML_SCRIPT_END);
  
  addCORSHeaders();
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  WiFi.hostname(newHostname.c_str());
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  
  setupMotorController();
  
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  
  calibrateSensor();
  
  server.on("/", handleRoot);
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/move", HTTP_POST, handleAPIMove);
  server.on("/api/speed", HTTP_POST, handleAPISpeed);
  server.on("/api/setZero", HTTP_POST, handleSetZero);
  server.on("/api/resetZero", HTTP_POST, handleResetZero);
  server.on("/post", HTTP_POST, handleGamepadPost);
  
  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/move", HTTP_OPTIONS, handleOptions);
  server.on("/api/speed", HTTP_OPTIONS, handleOptions);
  server.on("/api/setZero", HTTP_OPTIONS, handleOptions);
  server.on("/api/resetZero", HTTP_OPTIONS, handleOptions);
  server.on("/post", HTTP_OPTIONS, handleOptions);
  
  server.enableCORS(true);
  server.begin();
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("HTTP server started on port 80");
  Serial.println("WebSocket server started on port 81");
  
  lastDataSend = millis();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  if (!calibrated) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (lastTime == 0) deltaTime = 0.01;
  lastTime = currentTime;
  
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;
  
  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  pitch += gyroX * deltaTime * 180.0 / PI;
  roll += gyroY * deltaTime * 180.0 / PI;
  yaw += gyroZ * deltaTime * 180.0 / PI;
  
  float alpha = 0.96;
  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll = alpha * roll + (1.0 - alpha) * accelRoll;
  
  if (clientConnected && (currentTime - lastDataSend >= SEND_INTERVAL)) {
    if (dataChanged() || lastDataSend == 0) {
      sendSensorData();
      lastDataSend = currentTime;
    }
  }
  
  delay(10);
}
