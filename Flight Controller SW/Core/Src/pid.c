/*
 * pid.c
 *
 *  Created on: Mar 14, 2023
 *      Author: AEren TURK
 */
#include "types.h"
#include "mpu.h"
extern double roll_pid_i;
extern double  roll_last_error;
extern double  pitch_pid_i;
extern double pitch_last_error;
extern double  yaw_pid_i;
extern double yaw_last_error;
extern double roll_control_signal;
extern double pitch_control_signal;
extern double yaw_control_signal;

extern double  rollError;
extern double pitchError;
int k=0;
float toplam1,toplam2;
extern SensorData_t pSensor;
extern float dt;
extern struct MotorPowers motorPowers;
extern float meanP;
extern float meanR;



double constrain(double x,double y,double z){
	if(x< y) return y;
	else if(x>z) return z;
	return x;
}

double max(double x,double y){

	if(x<y)return y;
	return x;

}
/*
 *
#define KP_roll_pitch 0.3
#define KI_roll_pitch  0.1
#define KD_roll_pitch  0.1
 */

double getControlSignalRoll( double delta_time_in_seconds) {
  double pid_p = rollError;
  double pid_d = (rollError - roll_last_error) / delta_time_in_seconds;
/*  if(error<3){
  roll_pid_i += error * delta_time_in_seconds;
  }*/
  double control_signal = (KP_roll_pitch * pid_p) + (KI_roll_pitch * roll_pid_i) + (KD_roll_pitch * pid_d);
  roll_last_error = rollError;
  return control_signal;
}

double getControlSignalPitch(double delta_time_in_seconds) {
  double pid_p = pitchError;
  double pid_d = (pitchError - pitch_last_error) / delta_time_in_seconds;
  //pitch_pid_i += error * delta_time_in_seconds;

 /*if(error<3){
	  pitch_pid_i += error * delta_time_in_seconds;
   }*/
  double control_signal = (KP_roll_pitch * pid_p) + (KI_roll_pitch * pitch_pid_i) + (KD_roll_pitch * pid_d);
  pitch_last_error = pitchError;
  return control_signal;
}

void  calculateMotorPowers(struct ReceiverCommands receiverCommands) {
  // calculate orientation errors (error: difference between desired orientation and actual orientation)


	double rcs,pcs;
   rollError = receiverCommands.RollAngle - meanR;
   pitchError = receiverCommands.PitchAngle - meanP;
 // double yawError = calculateYawError(receiverCommands, imu_values);
  double dtSec=(double)dt/168000000;

  // calculate control gains based on errors
  rcs = getControlSignalRoll(dtSec);
  pcs= getControlSignalPitch(dtSec);
  //yaw_control_signal = getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, imu_values.DeltaTimeInSeconds);

  // limit roll-pitch control signals
 /* roll_control_signal = constrain(rcs, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
  pitch_control_signal = constrain(pcs, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);

  */if(k<50){
	  k++;
	  toplam1+=pcs;
	  toplam2+=rcs;
  }
  if(k==50){
	  pitch_control_signal=toplam1/50;
	  roll_control_signal=toplam2/50;
	  k=0;
	  toplam1=toplam2=0;
  }
  roll_control_signal = constrain(roll_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
  pitch_control_signal = constrain(pitch_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
  // calculate power for each motor
  if(receiverCommands.Throttle>1050){

	   motorPowers.frontLeftMotorPower = round(receiverCommands.Throttle + roll_control_signal + pitch_control_signal - yaw_control_signal);
	   motorPowers.frontRightMotorPower = round(receiverCommands.Throttle - roll_control_signal + pitch_control_signal + yaw_control_signal)-25;
	   motorPowers.rearLeftMotorPower = round(receiverCommands.Throttle + roll_control_signal - pitch_control_signal + yaw_control_signal);
	   motorPowers.rearRightMotorPower = round(receiverCommands.Throttle - roll_control_signal - pitch_control_signal - yaw_control_signal)-20;

  }
  else{

	  motorPowers.frontLeftMotorPower = 0;

	  motorPowers.frontRightMotorPower = 0;


	  motorPowers.rearLeftMotorPower = 0;

	  motorPowers.rearRightMotorPower = 0;
  }


 reduceMotorPowers(motorPowers);

}



void reduceMotorPowers() { // to preserve balance if throttle limit exceeds the max value (180)
  int maxMotorPower = max(max(motorPowers.frontLeftMotorPower, motorPowers.frontRightMotorPower), max(motorPowers.rearLeftMotorPower, motorPowers.rearRightMotorPower));
  if (maxMotorPower > 2000) {
    double power_reduction_rate = (double)maxMotorPower / (double)2000;
    motorPowers.frontLeftMotorPower = round((double)motorPowers.frontLeftMotorPower / power_reduction_rate);
    motorPowers.frontRightMotorPower = round((double)motorPowers.frontRightMotorPower / power_reduction_rate);
    motorPowers.rearLeftMotorPower = round((double)motorPowers.rearLeftMotorPower / power_reduction_rate);
    motorPowers.rearRightMotorPower = round((double)motorPowers.rearRightMotorPower / power_reduction_rate);
  }



  if(motorPowers.frontLeftMotorPower==0 || motorPowers.frontRightMotorPower == 0 || motorPowers.rearLeftMotorPower==0 || motorPowers.rearRightMotorPower==0){

	  motorPowers.frontLeftMotorPower=0;
	  motorPowers.rearLeftMotorPower=0;
	  motorPowers.frontRightMotorPower=0;
	  motorPowers.rearRightMotorPower=0;

  }

  return motorPowers;
}

void resetPidVariables() {
  roll_pid_i = 0;
  roll_last_error = 0;
  pitch_pid_i = 0;
  pitch_last_error = 0;
  yaw_pid_i = 0;
  yaw_last_error = 0;
}

double fix360degrees(double val) {
  if (val > 180) {
    return val - 360;
  } else if (val < -180) {
    return val + 360;
  } else {
    return val;
  }
}
