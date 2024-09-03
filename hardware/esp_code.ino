#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
#include "BluetoothSerial.h"3

// Motor control pins
#define IN1 13
#define IN2 32
#define IN3 33
#define IN4 25

// Color sensor pins
#define S0 27
#define S1 26
#define S2 12
#define S3 14
#define sensorOut 35

// TF-Luna LiDAR sensor variables and object
TFMPlus tfmP;

// Bluetooth serial object
BluetoothSerial SerialBT;

// Variable for storing incoming value
char Incoming_value = 0;

// Motor control functions
void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  Serial.println("Turning right");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  Serial.println("Motors stopped");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void startMotorsBasedOnDistance() {
  while(1){
 int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

  if (tfmP.getData(tfDist, tfFlux, tfTemp)) {
    Serial.print("Dist: ");
    Serial.print(tfDist);
    // Serial.print(" cm, Flux: ");
    // Serial.print(tfFlux);
    // Serial.print(", Temp: ");
    // Serial.print(tfTemp);
    // Serial.println(" C");

    if (tfDist > 30) {
      moveForward();
    } else {
      stopMotors();
      delay(1000); // Stop and wait for 1 second
      turnLeft();
      delay(1000); // Turn left for 1 second
      moveForward();
    }
  } else {
    tfmP.printFrame(); // Display the error and HEX data
  }
  }
}



void setup() {
  // Initialize terminal serial port
  Serial.begin(115200);
  delay(20);

  // Initialize Bluetooth serial port
  SerialBT.begin("Self-driving");  // Bluetooth device name

  // Initialize TFMini Plus LiDAR sensor
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(20);
  tfmP.begin(&Serial2);

  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Color sensor pins setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Initial state: motors stopped
  stopMotors();
}

void loop() {
  if (SerialBT.available() > 0) {
    Incoming_value = SerialBT.read();
    handleBluetoothCommand(Incoming_value);
  }
}

void handleBluetoothCommand(char command) {
  switch (command) {
    case 'f':
      moveForward();
      break;
    case 'b':
      moveBackward();
      break;
    case 'l':
      turnLeft();
      break;
    case 'r':
      turnRight();
      break;
    case 't':
    Serial.println("Init Self driving mode");
      startMotorsBasedOnDistance(); 
      break;
      case 's':
      stopMotors();
      break;
    default:
      // Invalid command
      Serial.println("Invalid command received");
      break;
  }
}
