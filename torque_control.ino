/**
 * Torque control example of adaptive gripper with pressure feedback.
 * Based on SimpleFOC library.
 * Implements automatic pressure control with thresholds and derivative
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
#define PRESSURE_MAX 0.5     // Maximum threshold (to be calibrated)
#define PRESSURE_MIN 0.0     // Minimum threshold (to be calibrated)
#define DP_DT_THRESHOLD 1.0  // 1%/s pressure variation
#define TIMEOUT_MS 10000     // Safety timeout
#define SMOOTHING_FACTOR 0.3 // Exponential derivative filter
#define DPDT_STABLE_SAMPLES 100  // Number of samples to consider derivative stable (value to be calibrated)
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
int dpdtStableCount = 0; // Counter for samples with low derivative
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

  // Angle sensor initialization
  tle5012Sensor.init();
  motor.linkSensor(&tle5012Sensor);

  // Driver configuration
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  // Motor configuration
  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.init();
  motor.initFOC();
  Serial.println(F("Motor ready."));

#if ENABLE_MAGNETIC_SENSOR
  // 3D sensor initialization
  dut.begin();
  calibrateSensor();
  Serial.println("3D sensor calibration completed");

  // Button configuration
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
#endif

  Serial.println("System ready");
#if ENABLE_COMMANDER
  command.add('T', doTarget, "target voltage");
#endif
  _delay(1000);
}

void loop() {
#if ENABLE_MAGNETIC_SENSOR
  // Button reading with edge detection
  bool button1State = digitalRead(BUTTON1);
  bool button2State = digitalRead(BUTTON2);  // Add reading of button 2 state
  
  // 3D sensor data acquisition
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset;
  y -= yOffset;
  z -= zOffset;


  // Calculate pressure as vector magnitude
  currentPressure = sqrt(x*x + y*y + z*z);
  
  // Calculate filtered derivative
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

  // State machine for closure control
  if(canClose && button1State == LOW && prevButton1State == HIGH) {
    // Button falling edge
    isClosing = true;
    closingStartTime = currentTime;
    dpdtStableCount = 0; // Reset counter at the beginning of closure
    angleStableCount = 0; // Reset angle stability counter
    Serial.println("Starting closure...");
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
    // Calculate dynamic thresholds
    float dpdtThreshold = (DP_DT_THRESHOLD / 100.0) * PRESSURE_MAX;
    
    // Priority stop conditions
    if(currentPressure >= PRESSURE_MAX) {
      Serial.println("Stop: maximum pressure reached");
      isClosing = false;
    }
    else if((currentTime - closingStartTime) > TIMEOUT_MS) {
      Serial.println("Stop: safety timeout");
      isClosing = false;
    }
    // Check pressure derivative only if not already stopped for other reasons
    else {
        if(smoothedDpdt <= dpdtThreshold) {
            dpdtStableCount++;
            if (dpdtStableCount >= DPDT_STABLE_SAMPLES) {
                Serial.print("Stop: pressure stabilized (low derivative for ");
                Serial.print(DPDT_STABLE_SAMPLES);
                Serial.println(" samples)");
                isClosing = false;
            }
        } else {
            dpdtStableCount = 0; // Reset counter if derivative is no longer low
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
      target_voltage = -0.5;  // Holding voltage
      canClose = false;  // Disable closing until Button 2 is pressed
      Serial.println("Closing locked until Button 2 is pressed");
    } else {
      target_voltage = -3.0;  // Closing voltage
    }
  } 
  else if(digitalRead(BUTTON2) == LOW) {
    target_voltage = 2.0;  // Opening
    // Note: when button is released, the falling edge handler above will restart closing
  } 
  else {
    target_voltage = -0.3; // Holding
  }

  // Send diagnostic data
  Serial.print("P:");
  Serial.print(currentPressure);
  Serial.print(",dP/dt:");
  Serial.print(smoothedDpdt);  
  Serial.print(",Is closing:");
  Serial.println(isClosing);
 
#endif

  // Sensor update and motor control
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

  Serial.println("Calibration in progress...");
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
  
  Serial.println("\nCalculated offsets:");
  Serial.print("X: "); Serial.println(xOffset);
  Serial.print("Y: "); Serial.println(yOffset);
  Serial.print("Z: "); Serial.println(zOffset);
}
#endif