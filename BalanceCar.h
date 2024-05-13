#include <avr/interrupt.h>
#include "binary.h"
#include "Arduino.h"
/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"
//#include "Command.h"
#include "voltage.h"
//#include "PinChangeInt.h"


MPU6050 mpu;
KalmanFilter kalmanfilter;
KalmanFilter kalmanfilter1;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
float Q_angle1 = 0.01, Q_gyro1 = 0.005, R_angle1 = 0.05, C_01 = 1, K11 = 0.05;

int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;

const double kp = 0.305;
const double kd = 0.1;
const double ki = 1.9;
double integral = 0.0;
double integral_max = 4;
double zero_setpoint = -0.75;
double prev_err = 0.0;
const double int_decay = 0.3;
double angle_raw = 0.0;
double angle_test = 0.0;
double vel = 0.0;
const double vel_max = 4.0;
double prevAngle = 0.0;
double output = 0.0;

unsigned long prev_micros;

double rps_left;
double rps_right;
const unsigned long ENCODER_COUNT_PER_ROTATION = 660;
const double RPS_PER_VOLT = 0.35;

void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void updateAngle(double dt) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle_raw = atan2(ay , az) * 57.3;
  kalmanfilter1.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle1, Q_gyro1, R_angle1, C_01, K11);
  angle_test = kalmanfilter1.angle;
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  //Serial.println(kalmanfilter_angle);
}

void updateEncoderSpeeds(double dt) {
  encoder_left_pulse_num_speed = pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed = pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  rps_left = 1.0*encoder_left_pulse_num_speed / ENCODER_COUNT_PER_ROTATION / dt;
  rps_right = 1.0*encoder_right_pulse_num_speed / ENCODER_COUNT_PER_ROTATION / dt;
}

double getOutputRPSfromSetpoint(double dt, double setpoint) {
  double p_error = kp * (zero_setpoint + setpoint - kalmanfilter_angle);
  integral += (zero_setpoint + setpoint - kalmanfilter_angle)*dt;
  //integral *= (1-int_decay*dt);
  integral = constrain(integral,-integral_max,integral_max);
  double i_error = ki * integral;
  double d_error = kd * ((zero_setpoint + setpoint - kalmanfilter_angle) - prev_err) / dt;
  prev_err = p_error;
  double output = p_error + i_error + d_error;
  return output;
}

double getOutputRPS(double dt) {
  return getOutputRPSfromSetpoint(dt, 0);
}

int rpsToPWM(double rps) {
  double percentOut = rps / (RPS_PER_VOLT * getVoltage());
  int dutyCycle = (percentOut * 255);
  return constrain(dutyCycle, -255, 255);
}

void balanceCarWithSetpoint(double dt, double setpoint) {
  vel += getOutputRPSfromSetpoint(dt, setpoint)*dt;
  //vel = constrain(vel, -vel_max, vel_max);
  int output = rpsToPWM(vel);
  pwm_left = output;
  pwm_right = output;
  if (abs(kalmanfilter_angle)>23) { 
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
    integral = 0;
    return;
  }
  if (output > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(BIN1, 1);
    analogWrite(PWMA_LEFT, output);
    analogWrite(PWMB_RIGHT, output);
  } else {
    digitalWrite(AIN1, 0);
    digitalWrite(BIN1, 0);
    analogWrite(PWMA_LEFT, -output);
    analogWrite(PWMB_RIGHT, -output);
  }
  //Serial.print(", Output: "); Serial.print(output);
  //Serial.print(", Voltage: "); Serial.println((analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5));
}

void balanceCar(double dt) {
  balanceCarWithSetpoint(dt, zero_setpoint);
}

/*void balanceCar()
{
  sei();
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  // if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  // {
  //   motion_mode = STOP;
  //   carStop();
  // }

  // if (motion_mode == STOP && key_flag != '4')
  // {
  //   car_speed_integeral = 0;
  //   setting_car_speed = 0;
  //   pwm_left = 0;
  //   pwm_right = 0;
  //   carStop();
  // }
  // else if (motion_mode == STOP)
  // {
  //   car_speed_integeral = 0;
  //   setting_car_speed = 0;
  //   pwm_left = 0;
  //   pwm_right = 0;
  // }
  // else
  // {
  //   if (pwm_left < 0)
  //   {
  //     digitalWrite(AIN1, 1);
  //     analogWrite(PWMA_LEFT, -pwm_left);
  //   }
  //   else
  //   {
  //     digitalWrite(AIN1, 0);
  //     analogWrite(PWMA_LEFT, pwm_left);
  //   }
  //   if (pwm_right < 0)
  //   {
  //     digitalWrite(BIN1, 1);
  //     analogWrite(PWMB_RIGHT, -pwm_right);
  //   }
  //   else
  //   {
  //     digitalWrite(BIN1, 0);
  //     analogWrite(PWMB_RIGHT, pwm_right);
  //   }
  // }
}*/

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}
ISR(PCINT2_vect) {
  encoder_count_right_a++;
}

// void updateWithSetpoint(double setpoint) {
//   //sei();
//   double dt = (micros() - prev_micros)*1.0 / 1E+6;
//   prev_micros = micros();
//   updateAngle(dt);
//   updateEncoderSpeeds(dt);
//   balanceCarWithSetpoint(dt, setpoint);
// }

void updateWithSetpoint(double setpoint) {
  double dt = (micros() - prev_micros)*1.0 / 1E+6;
  prev_micros = micros();

  // double accAngle = atan2(mpu.getAccelerationY(), mpu.getAccelerationZ())*RAD_TO_DEG;
  // double gyroRate = map(mpu.getRotationX(), -32768, 32767, -250, 250);
  // double gyroAngle = (float)gyroRate*dt;
  // Serial.println(accAngle);
  // angle_raw = gyroRate;
  // double alpha = 0.25/(0.25 + dt);
  // kalmanfilter_angle = alpha*(prevAngle+gyroAngle) + (1-alpha)*(accAngle);
  // prevAngle = kalmanfilter_angle;
  updateAngle(dt);

  double p_error = kp * (zero_setpoint + setpoint - kalmanfilter_angle);
  integral += (zero_setpoint + setpoint - kalmanfilter_angle)*dt;
  //integral *= (1-int_decay*dt);
  integral = constrain(integral,-integral_max,integral_max);
  double i_error = ki * integral;
  double d_error = kd * ((zero_setpoint + setpoint - kalmanfilter_angle) - prev_err) * dt;
  prev_err = p_error;
  output = p_error + i_error + d_error;
  
  pwm_left = rpsToPWM(output);
  pwm_right = rpsToPWM(output);
  if (abs(kalmanfilter_angle)>23) { 
    digitalWrite(AIN1, HIGH);
    digitalWrite(BIN1, LOW);
    analogWrite(PWMA_LEFT, 0);
    analogWrite(PWMB_RIGHT, 0);
    integral = 0;
    return;
  }
  if (output > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(BIN1, 1);
    analogWrite(PWMA_LEFT, pwm_left);
    analogWrite(PWMB_RIGHT, pwm_right);
  } else {
    digitalWrite(AIN1, 0);
    digitalWrite(BIN1, 0);
    analogWrite(PWMA_LEFT, -pwm_left);
    analogWrite(PWMB_RIGHT, -pwm_right);
  }
}

void update() {
  updateWithSetpoint(zero_setpoint);
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();
  Wire.begin();
  mpu.initialize();
  mpu.setXAccelOffset(1946);
  mpu.setYAccelOffset(1701);
  mpu.setZAccelOffset(1694);
  mpu.setXGyroOffset(-201);
  mpu.setYGyroOffset(292);
  mpu.setZGyroOffset(-1);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  PCICR |= B00000100; // enable pin change interrupt on Port D
  PCMSK2 |= B00010000; // enable pin change interrupt on Port D for PCINT20 (pin 4)
  // MsTimer2::set(dt*1000, update);
  prev_micros = micros();
  // MsTimer2::start();
}
