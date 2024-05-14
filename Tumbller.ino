#include <Arduino.h>
#include "Pins.h"
#include "BalanceCar.h"


int period = 4000;
const double angle_amplitude = 2.0;

double drive = 0.0;
double steer = 0.0;
const int steer_amplitude = 30;
const double DEADZONE = 0.3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("HI there please");
  voltageInit();
  carInitialize();
  digitalWrite(STBY_PIN, HIGH);
  Serial.println("MMM");
  
  //carBack(255);
  //prev_micros = micros();
  // analogWrite(PWMA_LEFT, 255);
  // analogWrite(PWMB_RIGHT, 255);
}

void loop() {
  
  //double angle = angle_amplitude * sin(2*PI/period * (millis() % period));
  //updateWithSetpoint(angle);

  // Serial.print("Out:"); Serial.print(output);
  // //Serial.print(",AngleTest:"); Serial.print(angle_test);
  // Serial.print(",Angle:"); Serial.print(kalmanfilter_angle);
  // //Serial.print(",RawAngle:"); Serial.print(angle_raw);
  // Serial.print(",Drive:"); Serial.print(drive);
  // Serial.print(",Integral:"); Serial.print(integral);
  // Serial.print(",Voltage:"); Serial.println(getVoltage());
  // Serial.print("Left speed: "); Serial.print(rps_left);
  // Serial.print(", Right speed: "); Serial.println(rps_right);
  if (Serial.available() >= 10) {
    Serial.readStringUntil('D');
    int drive_input = Serial.parseInt();
    Serial.readStringUntil('S');
    int steer_input = Serial.parseInt();
    drive = drive_input*2.0 / 255.0 - 1.0;
    steer = steer_input*2.0 / 255.0 - 1.0;
    if (abs(steer) <= DEADZONE) {
      steer = 0;
    }
  }
  
  update(drive * angle_amplitude, steer * steer_amplitude);
}
