#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

// Declarer les variables globales
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
SoftwareSerial labviewSerial(2, 3); // RX, TX (transmission et réception des données)
// Moteur A connecté aux pins 5 et 6
int motorAPin1 = 7;
int motorAPin2 = 4;

// Moteur B connecté to pins 2 et 3 
int motorBPin1 = 3;
int motorBPin2 = 2;

float accX = 0, accY = 0, accZ = 0;
float dt = 0.02; // temps d'échantillonnage en secondes (20 ms)
bool startPid = false; // démarrer ou non le PID
int start=0;
float Kp = 0, Ki = 0, Kd = 0;
float pidOutput;
float setpoint = 0; // Consigne
float pitch, pitchOutput, pitchError, pitchLastError, pitchIntegral, pitchDerivative;

void setup() {
  
  // Initialiser la communication série
  Serial.begin(9600);
  labviewSerial.begin(9600);
  // Initialiser l'accelerometre
  if(!accel.begin()) {
//   ADXL345 Non détécté
    while(1);
  }

  // definir la marge de l' accelerometre
  accel.setRange(ADXL345_RANGE_16_G);

  // fréquence de l'accelerometer 
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  // Initialiser les sorties des 2 moteurs
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
}

void loop() {
  // lire les données de l'accelerometre
  sensors_event_t event;
  accel.getEvent(&event);
  accX = event.acceleration.x;
  accY = event.acceleration.y;
  accZ = event.acceleration.z;

  // Calcul de l'angle 
  pitch = -atan2(accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI;

  // Conversion de l'angle en une chaîne de caractères
  char angleString[10];
  dtostrf(pitch, 4, 2, angleString);

  // Envoi de l'angle à LabVIEW
  Serial.println(angleString);

  // Reception des variables à partir de LabVIEW
 while (labviewSerial.available() > 0) {
    String message = labviewSerial.readStringUntil('\n');
    Serial.println(message);
    if (message.startsWith("C")) {
      setpoint = message.substring(1).toFloat();
    } else if (message.startsWith("S")) {
      start = message.substring(1).toInt();
      if (start==1){
        startPid= true;
      }else{
        startPid= false;
      }
    } else if (message.startsWith("kp")) {
      Kp = message.substring(2).toFloat();
    } else if (message.startsWith("ki:")) {
      Ki = message.substring(2).toFloat();
    } else if (message.startsWith("kd:")) {
      Kd = message.substring(2).toFloat();
    }
  }
  // controle PID 
  if(startPid) {
      // Compute PID output
  pitchError = setpoint - pitch;
  pitchIntegral += pitchError * dt;
  pitchDerivative = (pitchError - pitchLastError) / dt;
  pitchOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  pitchLastError = pitchError;

  // commander les moteurs
  if (pitchOutput > 0) {
    // Moteur A en marche
    digitalWrite(motorAPin1, HIGH);
    digitalWrite(motorAPin2, LOW);
    // Moteur B en arret
    digitalWrite(motorBPin1, LOW);
    digitalWrite(motorBPin2, LOW);
  } else if (pitchOutput < 0) {
    // Moteur A en arret
    digitalWrite(motorAPin1, LOW);
    digitalWrite(motorAPin2, LOW);
    // Moteur B en marche
    digitalWrite(motorBPin1, HIGH);
    digitalWrite(motorBPin2, LOW);
  } else {
    // Stop both motors
    digitalWrite(motorAPin1, LOW);
    digitalWrite(motorAPin2, LOW);
    digitalWrite(motorBPin1, LOW);
    digitalWrite(motorBPin2, LOW);
  }
  }
} 
