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

#include "turtlebot3_omni.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel    = 0.2;



double linear_x                = 0.1;
double linear_y                = 0.1;
double angular_z               = 0.1;
double goal_linear_x_velocity  = 0.1;
double goal_linear_y_velocity  = 0.1;
double goal_angular_velocity   = 0.1;
double imu_roll_init;
double imu_pitch_init;
double position_x               =0.0;
double position_y               =0.0;
double vx               =0.0;
double vy               =0.0;
double xd               =0.0;
double yd               =0.0;
double vd               =0.0;
double sumx_err =0.0;
/////////Matrix define///////////////
#include <MatrixMath.h>
#define N  (10)
mtx_type u[1][2];
mtx_type w1_x[2][N];
mtx_type b1_x[1][N];
mtx_type Z_x [1][N];
mtx_type wf_x [N][N];
mtx_type bf_x    [1][N];
mtx_type H_x     [1][N];
mtx_type wz_x    [N][1];
mtx_type wh_x    [N][1];
mtx_type Hout_x  [1];
mtx_type Zout_x  [1];
mtx_type out_x   [1];
mtx_type w1_y[2][N];
mtx_type b1_y[1][N];
mtx_type Z_y [1][N];
mtx_type wf_y [N][N];
mtx_type bf_y    [1][N];
mtx_type H_y     [1][N];
mtx_type wz_y    [N][1];
mtx_type wh_y    [N][1];
mtx_type Hout_y  [1];
mtx_type Zout_y  [1];
mtx_type out_y   [1];
mtx_type maxVal = 2;  // maxValimum random matrix entry range
#define eta 0.0001
double Kp=23.5;
double Ki=3.5;
float BLS_limit = 0.5;
void setup()
{
  DEBUG_SERIAL.begin(57600);
  while(!RTPS_SERIAL)
  {
    DEBUG_PRINT(".");
  }
  pinMode(LED_WORKING_CHECK, OUTPUT);
  // Initialize matrices
  for (int i = 0; i < N; i++)
  {
    w1_x[0][i] = random(0, maxVal)  / maxVal; 
    w1_x[1][i] = random(0, maxVal)  / maxVal; 
    b1_x[0][i] = random(0, maxVal)  / maxVal;
    bf_x[0][i] = random(0, maxVal)  / maxVal;
    wz_x[i][0] = random(0, maxVal)  / maxVal;
    wh_x[i][0] = random(0, maxVal)  / maxVal;
    w1_y[0][i] = random(0, maxVal)  / maxVal; 
    w1_y[1][i] = random(0, maxVal)  / maxVal; 
    b1_y[0][i] = random(0, maxVal)  / maxVal;
    bf_y[0][i] = random(0, maxVal)  / maxVal;
    wz_y[i][0] = random(0, maxVal)  / maxVal;
    wh_y[i][0] = random(0, maxVal)  / maxVal;
    for (int j = 0; j < N; j++)
    {
      wf_x[i][j] = random(0, maxVal)  / maxVal; 
      wf_y[i][j] = random(0, maxVal)  / maxVal; 
    }
  }
  // Setting for Dynamixel motors
  motor_driver.init();
  sensors.init();
  diagnosis.init();
  updateGyroCali(true);
  sensors.updateIMU();
  imu_roll_init=sensors.imu_.rpy[0];
  imu_pitch_init=sensors.imu_.rpy[1];
  ros2::init(&RTPS_SERIAL);
  sprintf(imu_frame_id, "imu_link");
  sprintf(joint_state_header_frame_id, "base_link");
  sprintf(sensor_state_header_frame_id, "sensor_state");
  // Start Dynamixel Control Interrupt
  //startDynamixelControlInterrupt();
}

void loop()
{
  static TurtleBot3 turtlebot3_node;
  uint32_t t = millis();

  
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    
    controlOmni();
    tTime[0] = t;
  }

  updateGyroCali(true);

  // Show LED status
  diagnosis.showLedStatus(true);


  // Call all the callbacks waiting to be called at that point in time
  ros2::spin(&turtlebot3_node);}

void startDynamixelControlInterrupt()
{
  Timer.stop();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlOmni);
  Timer.start();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControl(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if (received_data & RC100_BTN_U)
    {
      linear_x  += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if (received_data & RC100_BTN_D)
    {
      linear_x  -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }

    if (received_data & RC100_BTN_L)
    {
      linear_y -= VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }
    else if (received_data & RC100_BTN_R)
    {
      linear_y += VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }

    if (received_data & RC100_BTN_1)
    {

    }
    else if (received_data & RC100_BTN_2)
    {
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_3)
    {

    }
    else if (received_data & RC100_BTN_4)
    {
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_6)
    {
      linear_x  = const_cmd_vel;
      linear_y  = 0.0;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      linear_x  = 0.0;
      linear_y  = 0.0;
      angular_z = 0.0;
    }

    if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }

    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }

    goal_linear_x_velocity  = linear_x;
    goal_linear_y_velocity  = linear_y;
    goal_angular_velocity   = angular_z;
  }
}

/*******************************************************************************
* Control onmi speed
*******************************************************************************/
void controlOmni()
{
  bool dxl_comm_result = false;
  int64_t wheel_value[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};
  double wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};
  double Tx,Ty,Tz;
  static bool init_imu = false;
  static int imu_count=0;
  sensors.updateIMU();
  static int i=0;
  Tx=Ty=0;
  if (init_imu==false)
  {   init_imu=true;
      imu_count++;
      if (imu_count>9500 && sensors.imu_.rpy[0]<4.0 && sensors.imu_.rpy[1]<4.0) 
      {
        i++;
        imu_roll_init+=sensors.imu_.rpy[0];
        imu_pitch_init+=sensors.imu_.rpy[1];
      }
      if (i==1)
      {   
        init_imu=true;
        imu_roll_init=0;
        imu_pitch_init=0;
      }
  }
   else
   {
    //vx=-vx;position_x=-position_x;
    //vy=-vy;position_y=-position_y;
    //vx=0;vy=0;position_x=0;position_y=0;
    xd=position_x;yd=position_y;
//    Kp=10.5;Ki=2.5;  
    Kp=6.5;Ki=2.5;   
    Tx=((Kp*((sensors.imu_.rpy[0]-0.0)*M_PI /180))+(Ki*sensors.imu_.gyroRaw[0]*(2000.0*M_PI / 5898240.0))+(3.0*(vx-goal_velocity_from_cmd[0])/rb)+(3.0*(position_x-xd)/rb));//+BLS_x((0.0-sensors.imu_.rpy[0])*M_PI /180,0.1*sensors.imu_.gyroRaw[0]*(-2000.0*M_PI / 5898240.0));
    Ty=-((Kp*(sensors.imu_.rpy[1]-0.0)*M_PI /180)+(Ki*sensors.imu_.gyroRaw[1]*(2000.0*M_PI / 5898240.0))+(2.5*(vy-goal_velocity_from_cmd[1])/rb)+(3.0*(position_y-yd)/rb));//+BLS_y((0.0-sensors.imu_.rpy[1])*M_PI /180,0.1*sensors.imu_.gyroRaw[1]*(-2000.0*M_PI / 5898240.0));
    //6.7
    //Tx=((-Kp*((sensors.imu_.rpy[0]-0.0)*M_PI /180))+(-Ki*sensors.imu_.gyroRaw[0]*(2000.0*M_PI / 5898240.0))+(6.7*(position_x-xd))+(0.0*(vx-0.005))+1.7*sumx_err);
    //Ty=((-Kp*(sensors.imu_.rpy[1]-0.0)*M_PI /180)+(-Ki*sensors.imu_.gyroRaw[1]*(2000.0*M_PI / 5898240.0))+(6.7*(position_y-yd))+(0.0*(vy-0.0)));    
    if(sumx_err>20)
    {sumx_err=20;}
    if(pow(position_x-xd,2)<0.000025)
    {sumx_err=sumx_err/2;}
    /*Serial.print(position_x);
    Serial.print(" ");
    Serial.print(position_y);
    Serial.print("\n");*/
    /*Tx=-3.5*(position_x-0.5);
    Ty=0;*/
    Tz=0;
    wheel_angular_velocity[1] =(2.0*cb)/(3.0*ca)*Ty-(2.0*sb)/(3.0*ca)*Tx+1/(3.0*sa)*Tz;
    wheel_angular_velocity[0] = (-cb+sqrt(3)*sb)/(3*ca)*Ty+(sb+sqrt(3)*cb)/(3*ca)*Tx+1/(3*sa)*Tz; 
    wheel_angular_velocity[2] = -(cb+sqrt(3)*sb)/(3*ca)*Ty+(sb-sqrt(3)*cb)/(3*ca)*Tx+1/(3*sa)*Tz;
    for (int id = 0; id < OMNIWHEEL_NUM; id++)
    {
      wheel_value[id] = wheel_angular_velocity[id] * (9.54*10.0/6.0) /  RPM_CONSTANT_VALUE;// 9.54
      if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
      else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
    }
    
    publishSensorStateMsg();
#ifdef DEBUG
  /*Serial.print("Vx : ");  Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);*/
#endif

  dxl_comm_result = motor_driver.controlMotor((int64_t)wheel_value[0], (int64_t)wheel_value[1], (int64_t)wheel_value[2]);
  if (dxl_comm_result == false)
  {
    return;
  }
   }
}
/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{ 
  static int32_t first_encoder=0;
  static int32_t second_encoder=0;
  static int32_t third_encoder=0;
  bool dxl_comm_result = false;
  dxl_comm_result = motor_driver.readEncoder(first_encoder, second_encoder, third_encoder);
  
  if (dxl_comm_result == true)
  {
    updateMotorInfo(first_encoder, second_encoder, third_encoder);
  }
  else
    return;

}
void updateMotorInfo(int32_t first_tick, int32_t second_tick, int32_t third_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};
  double w1=0.0,w2=0.0,w3=0.0;
  if (init_encoder)
  { prev_update_time = millis();
    init_time        = millis();
    for (int index = 0; index < OMNIWHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0.0;
      last_tick[index]      = 0.0;
      last_rad[index]       = 0.0;
      last_velocity[index]  = 0.0;
    }  
    last_tick[FIRST] = first_tick;
    last_tick[SECOND] = second_tick;
    last_tick[THIRD] = third_tick;
    init_encoder = false;
    return;
  }

  current_tick = first_tick;

  last_diff_tick[FIRST] = current_tick - last_tick[FIRST];
  last_tick[FIRST]      = current_tick;
  last_rad[FIRST]       += TICK2RAD * (float)last_diff_tick[FIRST];

  current_tick = second_tick;

  last_diff_tick[SECOND] = current_tick - last_tick[SECOND];
  last_tick[SECOND]      = current_tick;
  last_rad[SECOND]       += TICK2RAD * double(last_diff_tick[SECOND]);

  current_tick = third_tick;

  last_diff_tick[THIRD] = current_tick - last_tick[THIRD];
  last_tick[THIRD]      = current_tick;
  last_rad[THIRD]       += TICK2RAD * double(last_diff_tick[THIRD]);
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  unsigned long time_1=     time_now - init_time       ;
  prev_update_time = time_now;
  //if(sqrt(pow(xd-position_x,2)+pow(yd-position_y,2))<=0.005)
  w2=TICK2RAD * (double)last_diff_tick[FIRST]/((double)(step_time * 0.001));
  w1=TICK2RAD * (double)last_diff_tick[SECOND]/((double)(step_time * 0.001));
  w3=TICK2RAD * (double)last_diff_tick[THIRD]/((double)(step_time * 0.001));
  
  vx=sqrt(3)/(3*ca)*0.05*(w2-w3)*0.62;
  vy=0.05/(3*ca)*(-2*w1+w2+w3)*0.62;
  position_x=position_x+(sqrt(3))/(3*ca)*0.05*TICK2RAD *(last_diff_tick[FIRST]-last_diff_tick[THIRD])*0.62;
  position_y=position_y+(0.05)/(3*ca)*TICK2RAD *(2*last_diff_tick[SECOND]-last_diff_tick[FIRST]-last_diff_tick[THIRD])*0.62;
  sumx_err=sumx_err+(position_x-xd)*step_time*0.001;
  odom_pose[0]=position_x;
  odom_pose[1]=position_y;
  odom_pose[2]=sensors.imu_.rpy[2];
  odom_vel[0] = vx;
  odom_vel[1] = vy;
  odom_vel[2] = 0.0;
  /*if(time_1* 0.001>5.2)
      {xd=xd+step_time* 0.001*0.05;vd=0.01;}
  else if(time_1* 0.001>5)
      {xd=-0.05;vd=-0.05;}
  if (xd>=0.3) {xd =0.3;vd=0;}*/

  //int32_t *dxl_present_position = 0;
  //dxl_present_position=dxl_wb.syncRead("Present_Position");
  //Serial.print("Present_Position");
  //Serial.print((double)(dxl_present_init_position-dxl_present_position[0])*0.001534); 
}
double BLS_x(double u1,double u2)
{
  u[0][0]=u1;
  u[0][1]=u2;
  Matrix.Multiply((mtx_type*)u, (mtx_type*)w1_x, 1, 2, N, (mtx_type*)Z_x);
  Matrix.Add((mtx_type*) Z_x, (mtx_type*) b1_x, 1, N, (mtx_type*) Z_x);
  Matrix.Multiply((mtx_type*)Z_x, (mtx_type*)wf_x, 1, N, N, (mtx_type*)H_x);
  Matrix.Add((mtx_type*) H_x, (mtx_type*) bf_x, 1, N, (mtx_type*) H_x);
  Matrix.tanh((mtx_type*) H_x, 1, N);
  Matrix.Multiply((mtx_type*)Z_x, (mtx_type*)wz_x, 1, N, 1, (mtx_type*)Zout_x);
  Matrix.Multiply((mtx_type*)H_x, (mtx_type*)wh_x, 1, N, 1, (mtx_type*)Hout_x);
  Matrix.Add((mtx_type*) Zout_x, (mtx_type*) Hout_x, 1, 1, (mtx_type*) out_x);
  ///////////////////////////////////////////////////////////////////////////////
  mtx_type H_2 [1][N];
  Matrix.Dotproduct((mtx_type*) H_x, (mtx_type*) H_x, 1, N,(mtx_type*) H_2 );
  Matrix.Scale((mtx_type*) H_2, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Scalesubtract((u[0][0]+u[0][1])*eta, (mtx_type*)H_2, 1, N, (mtx_type*) H_2);
  mtx_type H_2_T [N][1];
    /////////////////////////////////////////////////////////////////////////////////
  Matrix.Transpose((mtx_type*)H_2, 1, N, (mtx_type*) H_2_T);
  Matrix.Dotproduct((mtx_type*) H_2_T, (mtx_type*) wh_x, 1, N,(mtx_type*) H_2_T );
  mtx_type diff_wf_x [N][N];
  Matrix.Multiply((mtx_type*)H_2_T, (mtx_type*)Z_x, N, 1, N, (mtx_type*)diff_wf_x);
  Matrix.Subtract((mtx_type*) wf_x, (mtx_type*)diff_wf_x, N, N, (mtx_type*) wf_x);
  //////////////////////////////////////////////////////////////////////////////////
  Matrix.Dotproduct((mtx_type*) H_2, (mtx_type*) wh_x, 1, N,(mtx_type*) H_2 );
  Matrix.Subtract((mtx_type*) bf_x, (mtx_type*)H_2, 1, N, (mtx_type*) bf_x);
  //////////////////////////////////////////////////////////
  Matrix.Scale((mtx_type*) H_x, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Scale((mtx_type*) Z_x, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Subtract((mtx_type*) wz_x, (mtx_type*)Z_x, 1, N, (mtx_type*) wz_x);
  Matrix.Subtract((mtx_type*) wh_x, (mtx_type*)H_x, 1, N, (mtx_type*) wh_x);
  if (out_x[0]>BLS_limit) out_x[0]=BLS_limit;
  else if (out_x[0]<-1.0*BLS_limit) out_x[0]=-1.0*BLS_limit; 
  return out_x[0];
}
double BLS_y(double u1,double u2)
{
  u[0][0]=u1;
  u[0][1]=u2;
  Matrix.Multiply((mtx_type*)u, (mtx_type*)w1_y, 1, 2, N, (mtx_type*)Z_y);
  Matrix.Add((mtx_type*) Z_y, (mtx_type*) b1_y, 1, N, (mtx_type*) Z_y);
  Matrix.Multiply((mtx_type*)Z_y, (mtx_type*)wf_y, 1, N, N, (mtx_type*)H_y);
  Matrix.Add((mtx_type*) H_y, (mtx_type*) bf_y, 1, N, (mtx_type*) H_y);
  Matrix.tanh((mtx_type*) H_y, 1, N);
  Matrix.Multiply((mtx_type*)Z_y, (mtx_type*)wz_y, 1, N, 1, (mtx_type*)Zout_y);
  Matrix.Multiply((mtx_type*)H_y, (mtx_type*)wh_y, 1, N, 1, (mtx_type*)Hout_y);
  Matrix.Add((mtx_type*) Zout_y, (mtx_type*) Hout_y, 1, 1, (mtx_type*) out_y);
  ///////////////////////////////////////////////////////////////////////////////
  mtx_type H_2 [1][N];
  Matrix.Dotproduct((mtx_type*) H_y, (mtx_type*) H_y, 1, N,(mtx_type*) H_2 );
  Matrix.Scale((mtx_type*) H_2, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Scalesubtract((u[0][0]+u[0][1])*eta, (mtx_type*)H_2, 1, N, (mtx_type*) H_2);
  mtx_type H_2_T [N][1];
    /////////////////////////////////////////////////////////////////////////////////
  Matrix.Transpose((mtx_type*)H_2, 1, N, (mtx_type*) H_2_T);
  Matrix.Dotproduct((mtx_type*) H_2_T, (mtx_type*) wh_y, 1, N,(mtx_type*) H_2_T );
  mtx_type diff_wf_y [N][N];
  Matrix.Multiply((mtx_type*)H_2_T, (mtx_type*)Z_y, N, 1, N, (mtx_type*)diff_wf_y);
  Matrix.Subtract((mtx_type*) wf_y, (mtx_type*)diff_wf_y, N, N, (mtx_type*) wf_y);
  //////////////////////////////////////////////////////////////////////////////////
  Matrix.Dotproduct((mtx_type*) H_2, (mtx_type*) wh_y, 1, N,(mtx_type*) H_2 );
  Matrix.Subtract((mtx_type*) bf_y, (mtx_type*)H_2, 1, N, (mtx_type*) bf_y);
  //////////////////////////////////////////////////////////
  Matrix.Scale((mtx_type*) H_y, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Scale((mtx_type*) Z_y, 1, N,(u[0][0]+u[0][1])*eta );
  Matrix.Subtract((mtx_type*) wz_y, (mtx_type*)Z_x, 1, N, (mtx_type*) wz_y);
  Matrix.Subtract((mtx_type*) wh_y, (mtx_type*)H_x, 1, N, (mtx_type*) wh_y);
  if (out_y[0]>BLS_limit) out_y[0]=BLS_limit;
  else if (out_y[0]<-1.0*BLS_limit) out_y[0]=-1.0*BLS_limit;    
  return out_y[0];
}
/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  if (isConnected)//(nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      //nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      //nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}
void subscribeReset(std_msgs::Empty* msg, void* arg)
{
  (void)(msg);
  (void)(arg);

  char log_msg[50];

  sprintf(log_msg, "Start Calibration of Gyro");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);

  sprintf(log_msg, "Reset Odometry");
  DEBUG_SERIAL.println(log_msg);
  //nh.loginfo(log_msg);  
}
void subscribeTimeSync(builtin_interfaces::Time* msg, void* arg)
{
  (void)(arg);
  
  ros2::syncTimeFromRemote(msg);
}
//IMU data: angular velocity, linear acceleration, orientation
void publishImu(sensor_msgs::Imu* msg, void* arg)
{
  (void)(arg);
  sensor_msgs::Imu imu_msg = sensors.getIMU();
  memcpy(msg, &imu_msg, sizeof(sensor_msgs::Imu));
  
  msg->header.stamp    = ros2::now();
  strcpy(msg->header.frame_id, imu_frame_id);
}


//Odometry : 
void publishOdometry(nav_msgs::Odometry* msg, void* arg)
{
  (void)(arg);
  msg->header.stamp            = ros2::now();
  strcpy(msg->header.frame_id, odom_header_frame_id);
  strcpy(msg->child_frame_id, odom_child_frame_id);
  msg->pose.pose.position.x    = odom_pose[0];
  msg->pose.pose.position.y    = odom_pose[1];
  msg->pose.pose.position.z    = odom_pose[2];
  //msg->pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
  msg->pose.pose.orientation.x = 0;
  msg->pose.pose.orientation.y = 0;
  msg->pose.pose.orientation.z = 0;
  msg->pose.pose.orientation.w = 0;
  msg->twist.twist.linear.x    = odom_vel[0];
  msg->twist.twist.linear.y    = odom_vel[2];
}
void subscribeCmdVel(geometry_msgs::Twist* msg, void* arg)
{
  (void)(arg);
  goal_velocity_from_cmd[0]  = constrain(msg->linear.x,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[1]  = constrain(msg->linear.y,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
}
