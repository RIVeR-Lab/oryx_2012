/*
 * PtzManager.h
 *
 *  Created on: Mar 31, 2012
 *      Author: oryx
 */

#ifndef PTZMANAGER_H_
#define PTZMANAGER_H_
#include "OryxManager.h"
#include "oryx_manager/PTZ_managerConfig.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "tf/tf.h"
#include <cmath>
#define PAN_MOTOR_GEAR_RATIO 420
#define TILT_MOTOR_GEAR_RATIO 690
bool invert;
homingData panMotor, tiltMotor, boomMotor;
bool driveBoomWithoutMonitoring;
int leftRightStep, upDownStep;
ros::Publisher ptz_publisher;
ros::Publisher group_ptz_publisher;
ros::Publisher cameraOrientationPublisher;
void homeMotor(homingData* motor, const epos_manager::MotorInfo::ConstPtr& msg);
bool isMotorTooClose(homingData motor);
void sendVelocityMessage(homingData motor, int velocity);
void sendPositionMessage(homingData motor, int setpoint, bool absolute);
oryx_manager::PTZ_managerConfig previousConfig;
//EposManager::GroupEPOSControl generatePositionMessage;
//EposManager::GroupEPOSControl generateVelocityMessage()


#endif /* PTZMANAGER_H_ */
