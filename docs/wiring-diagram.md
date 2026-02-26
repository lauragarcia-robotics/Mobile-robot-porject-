# Wiring Diagram - ESP32 Robot 

## Motor Driver 
| ESP32 Pin | L298N Pin |       Function        |
|-----------|-----------|-----------------------|
|  GPIO 13  |    IN1    | Right motor direction |
|  GPIO 25  |    IN2    | Right motor direction |
|  GPIO 33  |    INB    |    Right motor PWM    |
|  GPIO 26  |    IN3    |  Left motor direction |
|  GPIO 27  |    IN4    |  Left motor direction |
|  GPIO 14  |    INA    |    Left motor PWM     |

## Ultrasonic Sensor (HC-SR04)
| ESP32 Pin | Sensor Pin |    Function    |
|-----------|------------|----------------|
|   GPIO 5  |    TRIG    | Trigger signal |
|  GPIO 18  |    ECHO    |   Echo signal  |

## Infrared Sensor (TCRT5000)
| ESP32 Pin | Sensor Pin |    Function    |
|-----------|------------|----------------|
|  GPIO 32  |     DO     | Digital output |
|  GPIO 35  |     AO     |  Analog output |
