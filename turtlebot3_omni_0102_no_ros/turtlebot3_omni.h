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

#include <RC100.h>

#include "turtlebot3_omni_motor_driver.h"

#define DEBUG

#define WHEEL_RADIUS                    0.05      // meter
#define WHEEL_SEPARATION_ANGLE          60        // degree
#define DISTANCE_CENTER_TO_WHEEL        0.122     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define RPM_CONSTANT_VALUE              0.229

#define CONTROL_PERIOD                  500

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_LINEAR_Y               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_LINEAR_Y         1
#define SCALE_VELOCITY_ANGULAR_Z        1
#define CONTROL_MOTOR_SPEED_FREQUENCY   100   //hz
#define OMNIWHEEL_NUM                   3
#define LIMIT_X_MAX_VALUE               1023

#define FIRST                             0
#define SECOND                            1
#define THIRD                             2

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define sb 0.0
#define cb 1.0
#define sa 0.7071
#define ca 0.7071

#define rb 0.14
#define rw 0.05
#define c_ 0.28
#define Gama1 2.0
#define Gama2 1.35//1.35//0.8//
#define k1 8//10
#define kbeta  1.3//1.3//
#define Ib 0.0525
#define IB 0.0152
#define mb 6.577
#define mB 2.54
#define alpha Ib+(mb+mB)*rb*rb
#define beta_ mB*rb*0.185
#define gama mB*0.185*0.185+IB
double mx12;
double mx22;
double sx=0;
double cmcgx;
double my12;
double my22;
double sy=0;
double cmcgy;
double sigma_1x;
double sigma_2x;
double sigma_1y;
double sigma_2y;
static uint32_t tTimer[10];
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
// Ref : http://www.revistas.unal.edu.co/index.php/ingeinv/article/view/47763/52384
