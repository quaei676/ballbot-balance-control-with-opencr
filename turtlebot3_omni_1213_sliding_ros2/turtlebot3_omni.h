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
//////////////////////////////////////////////////////////////
#include <ros2arduino.h>
//#include <TurtleBot3_ROS2.h>

//#include <std_msgs/Bool.hpp>
//#include <std_msgs/Empty.hpp>
////#include <std_msgs/Int32.hpp>
//#include <sensor_msgs/Imu.hpp>
//#include <sensor_msgs/JointState.hpp>
////#include <sensor_msgs/BatteryState.hpp>
////#include <sensor_msgs/MagneticField.hpp>
////#include <geometry_msgs/Vector3.hpp>
//#include <geometry_msgs/Twist.hpp>
////#include <tf/tf.hpp>
////#include <tf/transform_broadcaster.hpp>
//#include <nav_msgs/Odometry.hpp>
//
//#include <turtlebot3_msgs/SensorState.hpp>
//#include <turtlebot3_msgs/Sound.hpp>
//#include <turtlebot3_msgs/VersionInfo.hpp>
/////////////////////////////////////////////////////////////
#include "turtlebot3_sensor.h"
#include "turtlebot3_diagnosis.h"
/////////////////////////////////////////////////////////////
#define HARDWARE_VER "0.1.0"
#define SOFTWARE_VER "0.1.0"
#define FIRMWARE_VER "0.1.0"
////////////////////////////////////////////////////////////
//#define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define RTPS_SERIAL                      Serial
#ifdef DEBUG
  #define DEBUG_PRINT(x)                 DEBUG_SERIAL.print(x)
#else
  #define DEBUG_PRINT(x)                  
#endif
///////////////////////////////////////////////////////////
#define SENSOR_STATE_PUBLISH_FREQUENCY         30    //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define IMU_PUBLISH_FREQUENCY                  30  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define ODOMETRY_PUBLISH_FREQUENCY             30    //hz
#define JOINT_STATE_PUBLISH_FREQUENCY          30   //hz
#define BATTERY_STATE_PUBLISH_FREQUENCY        30   //hz
#define MAGNETIC_FIELD_PUBLISH_FREQUENCY       30   //hz

#define CONTROL_MOTOR_SPEED_FREQUENCY          500   //hz
#define DEBUG_LOG_FREQUENCY                    10   //hz
////////////////////////////////////////////////////////////
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

#define CONTROL_PERIOD                  10

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define MIN_LINEAR_VELOCITY             -MAX_LINEAR_VELOCITY 
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

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define sb 0.0
#define cb 1.0
#define sa 0.7071
#define ca 0.7071

#define rb 0.14
//#define rb 0.1105
#define rw 0.05
#define c_ 0.28
#define Gama1 2.0
#define Gama2 1.35//1.35//0.8//
#define k1 8//10
#define kbeta  1.3//1.3//
#define Ib 0.0270
#define IB 0.0152
#define mb 3
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

/////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[2] = {0.0, 0.0};
float goal_velocity_from_button[2] = {0.0, 0.0};
float goal_velocity_from_cmd[3] = {0.0, 0.0, 0.0};

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
double  joint_states_pos[2]  = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  joint_states_vel[2]  = {0.0, 0.0};

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

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];
char sensor_state_header_frame_id[30];


void updateTFPrefix(bool isConnected);
void updateVariable(bool isConnected);
void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
bool calcOdometry(double diff_time);
void driveTest(uint8_t buttons);
void sendLogMsg(void);
void sendDebuglog(void);


void publishOdometry(nav_msgs::Odometry* msg, void* arg);
void publishImu(sensor_msgs::Imu* msg, void* arg);
void publishJointState(sensor_msgs::JointState* msg, void* arg);
void publishSensorState(turtlebot3_msgs::SensorState* msg, void* arg);
void publishVersionInfo(turtlebot3_msgs::VersionInfo* msg, void* arg);


void subscribeCmdVel(geometry_msgs::Twist* msg, void* arg);
void subscribeSound(turtlebot3_msgs::Sound* msg, void* arg);
void subscribeReset(std_msgs::Empty* msg, void* arg);
void subscribeTimeSync(builtin_interfaces::Time* msg, void* arg);


/*******************************************************************************
* TurtleBot3 Node Class
*******************************************************************************/
class TurtleBot3 : public ros2::Node
{
public:
  TurtleBot3()
  : Node()
  {
    /*******************************************************************************
    * Publisher
    *******************************************************************************/
    // // Odometry of Turtlebot3
    odom_pub_          = this->createPublisher<nav_msgs::Odometry>("odom");
    this->createWallFreq(ODOMETRY_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishOdometry, NULL, odom_pub_);
    DEBUG_PRINT("\r\n [Publisher Create]   /odom           : "); DEBUG_PRINT((odom_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // IMU of Turtlebot3
    imu_pub_           = this->createPublisher<sensor_msgs::Imu>("imu");
    this->createWallFreq(IMU_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImu, NULL, imu_pub_);
    DEBUG_PRINT("\r\n [Publisher Create]   /imu            : "); DEBUG_PRINT((imu_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Battey state of Turtlebot3 
    // battery_state_pub_ = this->createPublisher<sensor_msgs::BatteryState>("battery_state");
    // this->createWallFreq(BATTERY_STATE_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishBatteryState, NULL, battery_state_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /battery_state  : "); DEBUG_PRINT((battery_state_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Magnetic field
    // mag_pub_           = this->createPublisher<sensor_msgs::MagneticField>("magnetic_field");
    // this->createWallFreq(MAGNETIC_FIELD_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishMagneticField, NULL, mag_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /magnetic_field : "); DEBUG_PRINT((mag_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    // // (Not necessary) Command velocity of Turtlebot3 using RC100 remote controller
    // cmd_vel_rc100_pub_ = this->createPublisher<geometry_msgs::Twist>("cmd_vel_rc100");
    // this->createWallFreq(CMD_VEL_PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishCmdVelRC100, NULL, cmd_vel_rc100_pub_);
    // DEBUG_PRINT("\r\n [Publisher Create]   /cmd_vel_rc100  : "); DEBUG_PRINT((cmd_vel_rc100_pub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);
   

    /*******************************************************************************
    * Subscriber
    *******************************************************************************/
    cmd_vel_sub_       = this->createSubscriber<geometry_msgs::Twist>("cmd_vel", (ros2::CallbackFunc)subscribeCmdVel, NULL);
    /*DEBUG_PRINT("\r\n [Subscriber Create]  /cmd_vel        : "); DEBUG_PRINT((cmd_vel_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);

    sound_sub_         = this->createSubscriber<turtlebot3_msgs::Sound>("sound", (ros2::CallbackFunc)subscribeSound, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /sound          : "); DEBUG_PRINT((sound_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);*/

    reset_sub_         = this->createSubscriber<std_msgs::Empty>("reset", (ros2::CallbackFunc)subscribeReset, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /reset          : "); DEBUG_PRINT((reset_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);
    
    time_sync_sub_     = this->createSubscriber<builtin_interfaces::Time>("time_sync", (ros2::CallbackFunc)subscribeTimeSync, NULL);
    DEBUG_PRINT("\r\n [Subscriber Create]  /time_sync      : "); DEBUG_PRINT((time_sync_sub_!=NULL?"Success":"Fail")); DEBUG_PRINT(this->err_code);
  }


private:

  /* Publisher Pointer */
  ros2::Publisher<nav_msgs::Odometry>*            odom_pub_;
  ros2::Publisher<sensor_msgs::Imu>*              imu_pub_;
  
  //ros2::Publisher<sensor_msgs::BatteryState>*     battery_state_pub_;
  //ros2::Publisher<geometry_msgs::Twist>*          cmd_vel_rc100_pub_;
  //ros2::Publisher<sensor_msgs::MagneticField>*    mag_pub_;

  /* Subscriber Pointer */
  ros2::Subscriber<geometry_msgs::Twist>*         cmd_vel_sub_;
  //ros2::Subscriber<turtlebot3_msgs::Sound>*       sound_sub_;
  //ros2::Subscriber<std_msgs::Bool>*               motor_power_sub_;
  ros2::Subscriber<std_msgs::Empty>*              reset_sub_;
  ros2::Subscriber<builtin_interfaces::Time>*     time_sync_sub_;
};
