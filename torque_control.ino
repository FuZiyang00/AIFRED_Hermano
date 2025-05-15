/**
 * Torque control example of adaptive gripper with pressure feedback.
 * Based on SimpleFOC library.
 * Implementa controllo automatico di pressione con soglie e derivata
 */
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>
#include <math.h>

// SPI pins for TLE5012 sensor
#define PIN_SPI1_SS0 94
#define PIN_SPI1_MOSI 69
#define PIN_SPI1_MISO 95
#define PIN_SPI1_SCK 68

// 3-wire SPI communication
tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);


// BLDC motor configuration
BLDCMotor motor = BLDCMotor(7, 0.24, 360, 0.000133);

// Driver pins
const int U = 11;
const int V = 10;
const int W = 9;
const int EN_U = 6;
const int EN_V = 5;
const int EN_W = 3;

BLDCDriver3PWM driver = BLDCDriver3PWM(U, V, W, EN_U, EN_V, EN_W);

// Voltage control
float target_voltage = 1;

#if ENABLE_MAGNETIC_SENSOR
// 3D magnetic sensor
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
const int CALIBRATION_SAMPLES = 20;
double xOffset = 0, yOffset = 0, zOffset = 0;

// Pressure control parameters
#define PRESSURE_MAX 0.5     // Soglia massima (calibrare)
#define PRESSURE_MIN 0.0      // Soglia minima (calibrare)
#define DP_DT_THRESHOLD 1.0    // 1%/s di variazione pressione
#define TIMEOUT_MS 10000       // Timeout sicurezza
#define SMOOTHING_FACTOR 0.3   // Filtro derivata esponenziale
#define DPDT_STABLE_SAMPLES 100  // Numero di campioni per considerare la derivata stabile (valore da calibrare)
#define ANGULAR_VELOCITY_THRESHOLD 0.1 // Threshold for angular velocity
#define ANGLE_STABLE_SAMPLES 50     // Number of samples to consider angle stable

// Pressure control variables
float currentPressure = 0;
float previousPressure = 0;
float smoothedDpdt = 0;
unsigned long previousTime = 0;
unsigned long closingStartTime = 0;
bool isClosing = false;
bool prevButton1State = HIGH;
bool prevButton2State = HIGH;  // Add tracking for button 2
bool canClose = true;          // New flag to control if closing is allowed
int dpdtStableCount = 0; // Contatore per campioni con derivata bassa
float previousAngle = 0;
float smoothedAngularVelocity = 0;
int angleStableCount = 0; // Counter for stable angle samples
#endif

#if ENABLE_COMMANDER
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_voltage, cmd); }
#endif

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Inizializzazione sensore angolo
  tle5012Sensor.init();
  motor.linkSensor(&tle5012Sensor);

  // Configurazione driver
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  // Configurazione motore
  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.init();
  motor.initFOC();
  Serial.println(F("Motor ready."));

#if ENABLE_MAGNETIC_SENSOR
  // Inizializzazione sensore 3D
  dut.begin();
  calibrateSensor();
  Serial.println("Calibrazione sensore 3D completata");

  // Configurazione pulsanti
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
#endif

  Serial.println("Sistema pronto");
#if ENABLE_COMMANDER
  command.add('T', doTarget, "target voltage");
#endif
  _delay(1000);
}

void loop() {
#if ENABLE_MAGNETIC_SENSOR
  // Lettura pulsante con rilevamento fronte
  bool button1State = digitalRead(BUTTON1);
  bool button2State = digitalRead(BUTTON2);  // Add reading of button 2 state
  
  // Acquisizione dati sensore 3D
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset;
  y -= yOffset;
  z -= zOffset;


  // Calcolo pressione come magnitudine vettoriale
  currentPressure = sqrt(x*x + y*y + z*z);
  
  // Calcolo derivata filtrata
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0f;
  if(dt > 0) {
    float rawDpdt = (currentPressure - previousPressure) / dt;
    smoothedDpdt = (1.0 - SMOOTHING_FACTOR) * smoothedDpdt + SMOOTHING_FACTOR * rawDpdt;
    
    // Calculate angle change rate
    float currentAngle = tle5012Sensor.getSensorAngle();
    float rawAngularVelocity = (currentAngle - previousAngle) / dt;
    smoothedAngularVelocity = (1.0 - SMOOTHING_FACTOR) * smoothedAngularVelocity + SMOOTHING_FACTOR * abs(rawAngularVelocity);
    previousAngle = currentAngle;
  }
  previousPressure = currentPressure;
  previousTime = currentTime;

  // Check Button 2 state changes
  if(button2State == LOW && prevButton2State == HIGH) {
    // Button 2 pressed (rising edge)
    canClose = true;
    Serial.println("Reset: closing enabled");
  } 
  else if(button2State == HIGH && prevButton2State == LOW) {
    // Button 2 released (falling edge)
    isClosing = true;
    canClose = true;
    closingStartTime = currentTime;
    dpdtStableCount = 0;
    angleStableCount = 0;
    Serial.println("Button released: restarting closing");
  }
  prevButton2State = button2State;

  // Macchina a stati per controllo chiusura
  if(canClose && button1State == LOW && prevButton1State == HIGH) {
    // Fronte di discesa pulsante
    isClosing = true;
    closingStartTime = currentTime;
    dpdtStableCount = 0; // Resetta il contatore all'inizio della chiusura
    angleStableCount = 0; // Reset angle stability counter
    Serial.println("Inizio chiusura...");
  }
  prevButton1State = button1State;

  // Set isClosing to true if angle is moving regularly (only if canClose is true)
  if(canClose && !isClosing && smoothedAngularVelocity > ANGULAR_VELOCITY_THRESHOLD) {
    isClosing = true;
    closingStartTime = currentTime;
    dpdtStableCount = 0;
    angleStableCount = 0;
    Serial.println("Motion detected: started closing");
  }

  if(isClosing) {
    // Calcolo soglie dinamiche
    float dpdtThreshold = (DP_DT_THRESHOLD / 100.0) * PRESSURE_MAX;
    
    // Condizioni di stop prioritarie
    if(currentPressure >= PRESSURE_MAX) {
      Serial.println("Stop: pressione massima raggiunta");
      isClosing = false;
    }
    else if((currentTime - closingStartTime) > TIMEOUT_MS) {
      Serial.println("Stop: timeout sicurezza");
      isClosing = false;
    }
    // Controllo derivata pressione solo se non già fermato per altri motivi
    else {
        if(smoothedDpdt <= dpdtThreshold) {
            dpdtStableCount++;
            if (dpdtStableCount >= DPDT_STABLE_SAMPLES) {
                Serial.print("Stop: pressione stabilizzata (derivata bassa per ");
                Serial.print(DPDT_STABLE_SAMPLES);
                Serial.println(" campioni)");
                isClosing = false;
            }
        } else {
            dpdtStableCount = 0; // Resetta il contatore se la derivata non è più bassa
        }
        
        // Check if angle has stopped moving
        if(smoothedAngularVelocity < ANGULAR_VELOCITY_THRESHOLD * 0.5) {
            angleStableCount++;
            if(angleStableCount >= ANGLE_STABLE_SAMPLES) {
                Serial.println("Stop: angle movement stabilized");
                isClosing = false;
            }
        } else {
            angleStableCount = 0; // Reset if angle is still moving
        }
    }
    
    if(!isClosing) {
      target_voltage = -0.5;  // Tensione di mantenimento
      canClose = false;  // Disable closing until Button 2 is pressed
      Serial.println("Closing locked until Button 2 is pressed");
    } else {
      target_voltage = -3.0;  // Tensione di chiusura
    }
  } 
  else if(digitalRead(BUTTON2) == LOW) {
    target_voltage = 2.0;  // Apertura
    // Note: when button is released, the falling edge handler above will restart closing
  } 
  else {
    target_voltage = -0.3; // Mantenimento
  }

  // Invio dati diagnostici
  Serial.print("P:");
  Serial.print(currentPressure);
  Serial.print(",dP/dt:");
  Serial.print(smoothedDpdt);  
  Serial.print(",Is closing:");
  Serial.println(isClosing);
 
#endif

  // Aggiornamento sensore e controllo motore
  tle5012Sensor.update();
  motor.loopFOC();
  motor.move(target_voltage);

#if ENABLE_COMMANDER
  command.run();
#endif
}



#if ENABLE_MAGNETIC_SENSOR
void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrazione in corso...");
  for(int i=0; i<CALIBRATION_SAMPLES; i++) {
    double temp, x, y, z;
    dut.getMagneticFieldAndTemperature(&x, &y, &z, &temp);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(20);
    Serial.print(".");
  }
  
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
  
  Serial.println("\nOffset calcolati:");
  Serial.print("X: "); Serial.println(xOffset);
  Serial.print("Y: "); Serial.println(yOffset);
  Serial.print("Z: "); Serial.println(zOffset);
}
#endif