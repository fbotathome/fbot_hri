#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <map>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  500  // Minimum pulse width in microseconds
#define SERVOMAX  2500 // Maximum pulse width in microseconds
#define SERVO_PIN 0    // Servo connected to channel 0

// --- NOVO: Sistema de Piscagem Inteligente ---
bool blinkEnabled = false;          
unsigned long nextBlinkMillis = 0;
unsigned long blinkEndMillis = 0;
bool isBlinking = false;

// ----- PARAMETRIZAR -----------
String nameEyelidL = ""; 
String nameEyelidR = "";
int angleClosedL = 0; 
int angleClosedR = 0; 
// --------------------------------------------
int currentEyelidLeftAngle = 60;   
int currentEyelidRightAngle = 130; 

//functions declarations
std::map<String, int> configureMotors(JsonObject motors_config);
int getCommand(JsonObject doc);
JsonDocument stringToJsonObject(String str);
String write_to_motors(JsonObject motors_angles);
void clearMotors();
int angleToPulse(int angle);
void writeToAngle(int pin, int angle);

// --- NOVO: Função de controle ---
void handleBlinking();

//global variables
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();
std::map<String, int> motors;
enum commands : int {
  ERROR, CONFIG, WRITE_ANGLES, TOGGLE_BLINK = 3
};
Preferences preferences;

void setup() {
   // put your setup code here, to run once:
  Serial.begin(9600);
  pca9685.begin();
  pca9685.setPWMFreq(50); // Set frequency to 50 Hz

  preferences.begin("storage", false);
  String stored_config = preferences.getString("json_config", "");

  if(!stored_config.isEmpty()){
    clearMotors();
    JsonDocument json_doc = stringToJsonObject(stored_config);
    JsonObject json_obj = json_doc.as<JsonObject>();
    motors = configureMotors(json_obj);
  }
  nextBlinkMillis = millis() + 3000;
}

void loop() {
  handleBlinking(); // Processa piscagem se blinkEnabled e em Neutro

  if (Serial.available()==1){ //FAZER CHAVE VALOR COM COMANDO NO JSON PARA QUANDO FOR CONFIGURAÇÃO E QUANDO FOR ESCRITA DE VALORES!!!
    
      //Serial.print("LOOP!!");
    String string_json = Serial.readString();
    JsonDocument json_doc = stringToJsonObject(string_json);
    JsonObject json_obj = json_doc.as<JsonObject>();
    int cmd = getCommand(json_obj);
    String response;

    //Serial.print("command = ");
    //Serial.print(cmd);

    switch (cmd)
    {
    case ERROR:  //Command not found or Json not structured correctly
      response = "{\"response\": \"error\"}";
      break;
    
    case CONFIG: //Command for instanciating motors
      preferences.putString("json_config", string_json);
      clearMotors(); // Clear the existing motors map
      motors = configureMotors(json_obj);
      response = "{\"response\": \"success\", \"message\": {";
      for (auto it = motors.begin(); it != motors.end(); ++it) {
        response += "\"" + it->first + "\": " + String(it->second); 
        if (std::next(it) != motors.end()) {
          response += ", ";
        }
      }
      response += "}}";
      break;

    case WRITE_ANGLES:
      if (motors.empty()) {
        Serial.println("Erro: Nenhum motor configurado.");
        response = "{\"response\": \"error\", \"message\": \"No motors configured\"}";
        break;
      }

      //Serial.println(cmd);
      //Serial.println(WRITE_ANGLES);
      response = write_to_motors(json_obj);
      break;

    case TOGGLE_BLINK: 
      if (json_obj.containsKey("blink")) {
          blinkEnabled = json_obj["blink"].as<bool>();
          
          response = "{\"response\": \"success\", \"blink_enabled\": " + String(blinkEnabled ? "true" : "false") + "}";
      }
      break;

    default:
      response = "{\"response\": \"command not found\"}";
      break;
    }
    Serial.print(response);
  }
}

void handleBlinking() {
    // SEGURANÇA: Se os parâmetros não foram carregados via YAML, a função aborta silenciosamente
    if (!blinkEnabled || nameEyelidL == "" || motors.count(nameEyelidL) == 0) return;

    unsigned long currentMillis = millis();
    if (!isBlinking && currentMillis >= nextBlinkMillis) {
        isBlinking = true;
        blinkEndMillis = currentMillis + 150;
        writeToAngle(motors[nameEyelidL], angleClosedL);
        writeToAngle(motors[nameEyelidR], angleClosedR);
    }
    if (isBlinking && currentMillis >= blinkEndMillis) {
        isBlinking = false;
        nextBlinkMillis = currentMillis + random(2000, 6000);
        writeToAngle(motors[nameEyelidL], currentEyelidLeftAngle);
        writeToAngle(motors[nameEyelidR], currentEyelidRightAngle);
    }
}

int getCommand(JsonObject doc){
  if (doc.isNull()) 
  //Serial.println("JsonObject é nulo!");
  
  return ERROR; // Retorna um valor de erro
  int command_value = 0;
  if(doc["cmd"].is<JsonVariant>()){
    command_value = doc["cmd"].as<int>();
  }

  //Serial.println(command_value);
  doc.remove("cmd");
  return command_value;
}

JsonDocument stringToJsonObject(String str){
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, str);
  if (error) {
    //Serial.print("Erro no JSON: ");
    //Serial.println(error.c_str());
  } else {
    //Serial.println("JSON desserializado com sucesso!");
  }
  return doc;
}

std::map<String, int> configureMotors(JsonObject motors_config) {
  
  // std::map<String, Servo> motors_dict;
  std::map<String, int> motors_dict; // Use int to store pin numbers

  // NOVO: Captura parâmetros dinâmicos de piscagem se existirem
  if (motors_config.containsKey("blink_cfg")) {
    JsonObject bCfg = motors_config["blink_cfg"];
    nameEyelidL = bCfg["nameL"].as<String>();
    nameEyelidR = bCfg["nameR"].as<String>();
    angleClosedL = bCfg["angleL"].as<int>();
    angleClosedR = bCfg["angleR"].as<int>();
  }

  if(motors_config["cmd"].is<JsonVariant>()){
    motors_config.remove("cmd");
  }
  motors_config.remove("blink_cfg"); // Remove para não tentar criar pino para o objeto de config

  for (JsonPair key_value : motors_config) {
    const char* motor_name = key_value.key().c_str();
    int motor_pin = key_value.value().as<int>();

    //DEPRECATED -> SERVO OBJECT NOT USED ANYMORE, SAVING NAME AND PIN NUMBER INSTEAD
    // Servo* motor = new Servo(); // Aloca dinamicamente o objeto Servo
    // motor->attach(motor_pin);

    motors_dict[motor_name] = motor_pin;  //*motor; // Armazena o objeto no mapa
  }
  return motors_dict;
}

String write_to_motors(JsonObject motors_angles) {
  for (JsonPair key_value : motors_angles) {
    String motor_name = key_value.key().c_str(); 
    int motor_angle = key_value.value().as<int>();   

    Serial.print("Motor name: ");
    Serial.println(motor_name);
    Serial.print("Motor angle: ");
    Serial.println(motor_angle);

    if (motor_name == nameEyelidL) currentEyelidLeftAngle = motor_angle;
    else if (motor_name == nameEyelidR) currentEyelidRightAngle = motor_angle;

    // Verifica se o motor_name existe no mapa
    auto it = motors.find(motor_name);
    if (it == motors.end()) {
      Serial.println("Motor not found in map.");
      return "{\"response\": \"error\", \"message\": \"Motor not found: " + motor_name + "\"}";
    }

    int motor_pin = it->second;  // Obtém o pino associado ao motor
    Serial.print("Motor pin: ");
    Serial.println(motor_pin);
    writeToAngle(motor_pin, motor_angle);  // Move o motor para o ângulo desejado
  }
  return "{\"response\": \"success\", \"message\": \"Motors moved successfully\"}";
}

void clearMotors() {

  // for (auto& motor_pair : motors) {
  //   Servo* motor_ptr = &motor_pair.second; 
  //   motor_ptr->detach();
  // }

 motors.clear(); }

/**
 * @brief Converts an angle in degrees (0-180) to a pulse width in microseconds.
 * * @param angle The desired angle in degrees (0-180).
 * @return int The corresponding pulse width in microseconds.
 */

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

/**
 * @brief Writes the desired angle directly to the servo.
 * * @param pin The servo pin/channel on the PCA9685.
 * @param angle The desired angle in degrees (0-180).
 */
void writeToAngle(int pin, int angle) {
  int pulse = angleToPulse(angle);  // Convert angle to pulse width
  pca9685.writeMicroseconds(pin, pulse); // Write pulse width to servo
}