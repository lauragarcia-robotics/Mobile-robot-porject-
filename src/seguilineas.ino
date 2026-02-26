#include <Arduino.h>
#include "driver/ledc.h"

// Pines del motor
#define IN1 13
#define IN2 25
#define INB 33
#define IN3 26
#define IN4 27
#define INA 14



const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;


// Pines del sensor de distancia
const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

//Pines infrarojo
const int pinDigital = 32;  // DO del sensor al GPIO4
const int pinAnalog = 35;  // AO del sensor al GPIO34 

int cont = 0;


const int grey = 2500;
const int velocidadBase = 170;
const float k = 0.08;
const int velocidadMin = 150;
const int velocidadMax = 190;

int negro = 3700;
int blanco = 1200;

unsigned long previousMillis = 0;
const unsigned long intervaloMovimiento = 80; // 50 ms de movimiento
const unsigned long intervaloParada = 20;     // 20 ms para detener y leer
bool moviendose = true;

int velocidadIzq = 0;
int velocidadDer = 0;

void setup() {
  // put your setup code here, to run once:
 // Inicializar el puerto serial
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(pinDigital, INPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    
    
  ledcSetup(pwmChannel1, freq, resolution);     // Configura canal 1, roda izquierda
  ledcAttachPin(INA, pwmChannel1);              

  ledcSetup(pwmChannel2, freq, resolution);     // Configura canal 2, roda derecha
  ledcAttachPin(INB, pwmChannel2);  
  


}

void loop() {
  

unsigned long currentMillis = millis();

  if (moviendose) {
    // Mover con las velocidades calculadas
    moverRobot(velocidadIzq, velocidadDer);

    // Si pasaron 50ms, detener y cambiar estado
    if (currentMillis - previousMillis >= intervaloMovimiento) {
      previousMillis = currentMillis;
      moviendose = false;

      // Detener motores
      moverRobot(0, 0);
    }
  } else {
    // En estado detenido, leer sensores y calcular velocidades
    int valorAnalogico = analogRead(pinAnalog);
    int valorDigital = digitalRead(pinDigital);

    int error = grey - valorAnalogico;
    int ajuste = k * error;

    int vIzq = velocidadBase - ajuste;
    int vDer = velocidadBase + ajuste;

    // Dirección y ajuste de signos
    if (vIzq >= 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      vIzq = -vIzq;
    }

    if (vDer >= 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      vDer = -vDer;
    }

    // Limitar velocidad
    velocidadIzq = constrain(vIzq, velocidadMin, velocidadMax);
    velocidadDer = constrain(vDer, velocidadMin, velocidadMax);

    Serial.print("Valor Analógico: ");
    Serial.println(valorAnalogico);
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("Velocidad Izquierda PWM: ");
    Serial.println(velocidadIzq);
    Serial.print("Velocidad Derecha PWM: ");
    Serial.println(velocidadDer);

    // Si pasaron 20ms, cambiar a modo movimiento
    if (currentMillis - previousMillis >= intervaloParada) {
      previousMillis = currentMillis;
      moviendose = true;
    }
  }
}

void moverRobot(int vIzq, int vDer) {
  ledcWrite(pwmChannel1, vIzq);
  ledcWrite(pwmChannel2, vDer);
}
