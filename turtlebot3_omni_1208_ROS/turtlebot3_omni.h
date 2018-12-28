/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include <math.h>

///////////////////////
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>
//////////////////////

#define DEBUG

#define WHEEL_RADIUS                    0.05      // meter
#define WHEEL_SEPARATION_ANGLE          60        // degree
#define DISTANCE_CENTER_TO_WHEEL        0.122     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define RPM_CONSTANT_VALUE              0.229

#define CONTROL_PERIOD                  10

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_LINEAR_Y               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_LINEAR_Y         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define OMNIWHEEL_NUM                   3
#define LIMIT_X_MAX_VALUE               1023

#define FIRST                             0
#define SECOND                            1
#define THIRD                             2

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define sb 0.0
#define cb 1.0
#define sa 0.7071
#define ca 0.7071


/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[OMNIWHEEL_NUM]  = {0.0, 0.0, 0.0};
/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};
double  last_rad[OMNIWHEEL_NUM]       = {0.0, 0.0, 0.0};
unsigned long prev_update_time;
unsigned long init_time;
void receiveRemoteControlData(void);
void controlMotorSpeed(void);
void controlOmni();
double BLS_x(double u1,double u2);
double BLS_y(double u1,double u2);
///////////////////////////////////
static uint32_t tTime[10];

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[2] = {0.0, 0.0};
float goal_velocity_from_button[2] = {0.0, 0.0};
float goal_velocity_from_cmd[2] = {0.0, 0.0};
/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;
// Ref : http://www.revistas.unal.edu.co/index.php/ingeinv/article/view/47763/52384
