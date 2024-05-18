#include <Arduino.h>
#include "Pins.h"
#include "BalanceCar.h"


const double drive_amplitude = 2.0; // max angle setpoint in degrees
const int steer_amplitude = 60; // max steer to add in pwm units (out of 255)
const double DEADZONE = 0.3; // controller deadzone out of 1
double drive = 0.0;
double steer = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initializing...");
  voltageInit();
  carInitialize();
  digitalWrite(STBY_PIN, HIGH); // still no clue exactly what it does but  i dont want to change it
  Serial.println("Initialized");
}

void loop() {
  // Serial.print("Out:"); Serial.print(output);
  // //Serial.print(",AngleTest:"); Serial.print(angle_test);
  // Serial.print(",Angle:"); Serial.print(kalmanfilter_angle);
  // //Serial.print(",RawAngle:"); Serial.print(angle_raw);
  // Serial.print(",Drive:"); Serial.print(drive);
  // Serial.print(",Integral:"); Serial.print(integral);
  // Serial.print(",Voltage:"); Serial.println(getVoltage());
  // Serial.print("Left speed: "); Serial.print(rps_left);
  // Serial.print(", Right speed: "); Serial.println(rps_right);

  // if available data to read
  if (Serial.available() >= 10) {

    Serial.readStringUntil('D'); // drive input character
    char drive_str_in[3];
    Serial.readBytes(drive_str_in, 3); // store next three chars into drive_str_in
    int drive_input = atoi( drive_str_in ); // convert int
    
    Serial.readStringUntil('S'); // steer input char
    char steer_str_in[3];
    Serial.readBytes(steer_str_in, 3); // store next three chars into steer_str_in
    int steer_input = atoi( steer_str_in ); // convert to int
    
    // normalize to [-1,1]
    drive = drive_input*2.0 / 255.0 - 1.0;
    steer = steer_input*2.0 / 255.0 - 1.0;
    
    // add deadzone to steering
    if (abs(steer) <= DEADZONE) {
      steer = 0;
    }
  }
  
  update(drive * drive_amplitude, steer * steer_amplitude);

  delay(2);
}
