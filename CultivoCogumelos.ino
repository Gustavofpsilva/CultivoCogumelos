#include <DHT.h>
#include <PID_v1.h>

#define DHT_PIN 2
#define FAN_PIN 3
#define HEATER_PIN 5
#define HUMIDIFIER_PIN 6
#define LIGHT_PIN 9

DHT dht(DHT_PIN, DHT22);

double setpoint_temperature = 24.0; // Temperatura desejada em Celsius
double setpoint_humidity = 90.0;    // Umidade desejada em porcentagem
double setpoint_light_intensity = 500.0; // Intensidade de luz desejada em unidades arbitrárias

double input_temperature, output_temperature;
double input_humidity, output_humidity;
double input_light, output_light;

double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error_temperature, lastError_temperature;
double cumError_temperature, rateError_temperature;

double error_humidity, lastError_humidity;
double cumError_humidity, rateError_humidity;

double error_light, lastError_light;
double cumError_light, rateError_light;

double outMin = 0;
double outMax = 255;

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  digitalWrite(FAN_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);

  previousTime = millis();
}

void loop() {
  double current_temperature = dht.readTemperature();
  double current_humidity = dht.readHumidity();
  double current_light = analogRead(A0); // Substitua A0 pelo pino correto para o sensor de luz

  PID_Controller_Temperature(current_temperature);
  PID_Controller_Humidity(current_humidity);
  PID_Controller_Light(current_light);

  Serial.print("Temperatura: ");
  Serial.print(current_temperature);
  Serial.print(" °C | Umidade: ");
  Serial.print(current_humidity);
  Serial.print(" % | Luz: ");
  Serial.println(current_light);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

void PID_Controller_Temperature(double current_temperature) {
  error_temperature = setpoint_temperature - current_temperature;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_temperature;
  cumError_temperature += error_temperature * elapsedTime;
  double I = Ki * cumError_temperature;
  rateError_temperature = (error_temperature - lastError_temperature) / elapsedTime;
  double D = Kd * rateError_temperature;

  output_temperature = P + I + D;
  output_temperature = constrain(output_temperature, outMin, outMax);

  digitalWrite(HEATER_PIN, output_temperature > 50);
  lastError_temperature = error_temperature;
  previousTime = currentTime;
}

void PID_Controller_Humidity(double current_humidity) {
  error_humidity = setpoint_humidity - current_humidity;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_humidity;
  cumError_humidity += error_humidity * elapsedTime;
  double I = Ki * cumError_humidity;
  rateError_humidity = (error_humidity - lastError_humidity) / elapsedTime;
  double D = Kd * rateError_humidity;

  output_humidity = P + I + D;
  output_humidity = constrain(output_humidity, outMin, outMax);

  digitalWrite(HUMIDIFIER_PIN, output_humidity > 50);
  lastError_humidity = error_humidity;
  previousTime = currentTime;
}

void PID_Controller_Light(double current_light) {
  error_light = setpoint_light_intensity - current_light;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_light;
  cumError_light += error_light * elapsedTime;
  double I = Ki * cumError_light;
  rateError_light = (error_light - lastError_light) / elapsedTime;
  double D = Kd * rateError_light;

  output_light = P + I + D;
  output_light = constrain(output_light, outMin, outMax);

  digitalWrite(LIGHT_PIN, output_light > 50);
  lastError_light = error_light;
  previousTime = currentTime;
}
