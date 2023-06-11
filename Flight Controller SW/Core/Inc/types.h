/*
 * types.h
 *
 *  Created on: Mar 14, 2023
 *      Author: AEren TURK
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#define TX_MIN	1000
#define TX_MAX  2000

#define MAX_TILT_ANGLE  20
#define MIN_TILT_ANGLE  -MAX_TILT_ANGLE

#define THROTTLE_MAX	2000

#define KP_roll_pitch	3
#define KI_roll_pitch  0.0
#define KD_roll_pitch  0.8

#define KP_yaw = 0.40
#define KI_yaw = 0.50
#define KD_yaw = 0.00

#define QUADCOPTER_MAX_TILT_ANGLE 20.00// roll, pitch tilt angle limit in degrees
#define QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND 180.00


#define ROLL_PITCH_CONTROL_SIGNAL_LIMIT   400


struct ReceiverRawValues {
  char NewDataAvailable;
  char TransmitterCommunicationFailure;
  int ChannelValues[4];
};
struct ReceiverCommands {
  char Armed;
  char Error;
  int Throttle;
  double YawAngleChange;
  double PitchAngle;
  double RollAngle;
};
struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};
struct IMU_Values {
  char Error;
  char NewDataAvailable;
  double DeltaTimeInSeconds;
  struct Orientation CurrentOrientation;
  struct Orientation PreviousOrientation;
};
typedef struct MotorPowers {
  double frontLeftMotorPower;
  double frontRightMotorPower;
  double rearLeftMotorPower;
  double rearRightMotorPower;
}MotorPowers;
#endif /* INC_TYPES_H_ */
