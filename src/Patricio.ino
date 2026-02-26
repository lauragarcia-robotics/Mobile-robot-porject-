#include <Arduino.h>
#include "BluetoothSerial.h"
#include "driver/ledc.h"

// Pines del motor
#define IN1 13
#define IN2 25
#define INB 33
#define IN3 26
#define IN4 27
#define INA 14

char ultimocomando;

const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 200;

// Bluetooth
BluetoothSerial SerialBT;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

//Pines infrarojo
const int pinDigital = 32;  // DO del sensor al GPIO4
const int pinAnalog = 35;  // AO del sensor al GPIO34 

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

int cont = 0;

int velocidadIzq=0;
int velocidadDer=0;

void setup() {
    // Configuración de pines de salida
    Serial.begin(115200);
    SerialBT.begin("PATRICK");
    Serial.println("Bluetooth listo. Esperando conexión...");


    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(pinDigital, INPUT);

  ledcSetup(pwmChannel1, freq, resolution);     // Configura canal 1
  ledcAttachPin(INA, pwmChannel1);              // Asocia el pin INA al canal 1

  ledcSetup(pwmChannel2, freq, resolution);     // Configura canal 2
  ledcAttachPin(INB, pwmChannel2);  

}

void loop() {
  
    if (SerialBT.available()) {
        char received = SerialBT.read();
        ultimocomando = received;
        Serial.print("Recibido: ");
        Serial.println(ultimocomando);
        switch (received) { 
            case 'P':      parada(); break;
            case 'I':  Sizquierda(); break;
            case 'D':    Sderecha(); break;
            case 'A':    arranque(); break;
            case 'B':       atras(); break;
            case 'E':  Gizquierda(); break; //Rotar
            case 'R': Gderecha(); break;  //Rotar
            case 'S': Seguilineas(); break;
            default:  parada(); break;
        }
    }
}



void arranque() {
  do {
    Medusas();
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); //delante
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); //delante
    ledcWrite(pwmChannel1, 180);
    ledcWrite(pwmChannel2, 180);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'A');
}

void Sderecha() {
  do {
       Medusas();
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); //delante dreta
     digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); //delante
    ledcWrite(pwmChannel1, 180);
    ledcWrite(pwmChannel2, 160);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'D');
}

void Sizquierda() {
  do {
        Medusas();

    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); //delante
     digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); //delante
    ledcWrite(pwmChannel1, 160);
    ledcWrite(pwmChannel2, 180);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'I');
}

void atras() {
  do {
        Medusas();

   digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); //atras
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); //atras
    ledcWrite(pwmChannel1, 180);
    ledcWrite(pwmChannel2, 180);
     Serial.println("Atrás");

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'B');
    
}

void parada() {
  do {
        Medusas();

    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'P');
}

void Gizquierda(){
 do {
      Medusas();

    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); //atras dreta
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); //delante
    ledcWrite(pwmChannel1, 180);
    ledcWrite(pwmChannel2, 180);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
} while (ultimocomando == 'E');
}

void Gderecha(){
  do {
        Medusas();

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); //delante dreta
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); //atras
    ledcWrite(pwmChannel1, 180);
    ledcWrite(pwmChannel2, 180);

    if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); 
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'R');
}


void Seguilineas(){
  do {
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
  

 if (SerialBT.available()) {
            ultimocomando = SerialBT.read(); // Leer el nuevo comando
            Serial.print("Nuevo comando: ");
            Serial.println(ultimocomando);
  }
}while (ultimocomando == 'S');
}

void moverRobot(int vIzq, int vDer) {
  ledcWrite(pwmChannel1, vIzq);
  ledcWrite(pwmChannel2, vDer);
}

void Medusas(){
static unsigned long ultimaDeteccion = 0;
  static bool enEspera = false;
  const unsigned long tiempoEspera = 2000;  // 2 segundos

  int valorAnalogico = analogRead(pinAnalog);
  int valorDigital = digitalRead(pinDigital);

  Serial.print("Valor Analógico: ");
  Serial.println(valorAnalogico);
  /*
  Serial.print("Valor Digital: ");
  Serial.println(valorDigital);
*/
  unsigned long ahora = millis();

  if (!enEspera && (valorAnalogico < 600) && (valorAnalogico > 50)) {
    cont++;
    ultimaDeteccion = ahora;
    enEspera = true;
    Serial.print("Cont: ");
    Serial.println(cont);
    SerialBT.println("M");
  }

  // Salir de modo espera tras 2 segundos
  if (enEspera && (ahora - ultimaDeteccion >= tiempoEspera)) {
    enEspera = false;
  }
}
