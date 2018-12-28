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

#include "turtlebot3_core_config.h"
#include "turtlebot3_omni.h"


/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
//HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
double const_cmd_vel    = 0.2;
/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;


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
 // Initialize matrices
 // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);

  tf_broadcaster.init(nh);
  motor_driver.init();
  sensors.init();
  diagnosis.init();
  for (int i = 0; i < N; i++)
  {
    w1_x[0][i] = random(maxVal) - maxVal / 2.0f; 
    w1_x[1][i] = random(maxVal) - maxVal / 2.0f; 
    b1_x[0][i] = random(maxVal) - maxVal / 2.0f;
    bf_x[0][i] = random(maxVal) - maxVal / 2.0f;
    wz_x[i][0] = random(maxVal) - maxVal / 2.0f;
    wh_x[i][0] = random(maxVal) - maxVal / 2.0f;
    w1_y[0][i] = random(maxVal) - maxVal / 2.0f; 
    w1_y[1][i] = random(maxVal) - maxVal / 2.0f; 
    b1_y[0][i] = random(maxVal) - maxVal / 2.0f;
    bf_y[0][i] = random(maxVal) - maxVal / 2.0f;
    wz_y[i][0] = random(maxVal) - maxVal / 2.0f;
    wh_y[i][0] = random(maxVal) - maxVal / 2.0f;
    for (int j = 0; j < N; j++)
    {
      wf_x[i][j] = random(maxVal) - maxVal / 2.0f; 
      wf_y[i][j] = random(maxVal) - maxVal / 2.0f; 
    }
  }
  // Setting for Dynamixel motors
  
  // Start Dynamixel Control Interrupt
  //startDynamixelControlInterrupt();
  pinMode(LED_WORKING_CHECK, OUTPUT);
  setup_end = true;
  sensors.updateIMU();
  imu_roll_init=sensors.imu_.rpy[0];
  imu_pitch_init=sensors.imu_.rpy[1];
}

void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    tTime[3] = t;
  }
  if ((t-tTime[4]) >= 5)
  {
    controlOmni();
    tTime[4] = t;
  }
  // Update the IMU unit
  
  sensors.updateIMU();
  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

void startDynamixelControlInterrupt()
{
  /*Timer.stop();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlOmni);
  Timer.start();
  Timer.resume();*/
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
  static int i=0;
  sensors.updateIMU();
  Tx=Ty=0;
  if (init_imu==false)
  {   i=1;
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
    Kp=10.5;Ki=1.5;
    //4.5
    //Tx=((-Kp*((IMU.rpy[0]-0.0)*M_PI /180))+(-Ki*IMU.gyroRaw[0]*(2000.0*M_PI / 5898240.0))+(15.5*(position_x-xd)))+BLS_x((0.0-IMU.rpy[0])*M_PI /180,0.1*IMU.gyroRaw[0]*(-2000.0*M_PI / 5898240.0));
    //Ty=((-Kp*(IMU.rpy[1]-0.0)*M_PI /180)+(-Ki*IMU.gyroRaw[1]*(2000.0*M_PI / 5898240.0))+(15.5*(position_y-yd)))+BLS_y((0.0-IMU.rpy[1])*M_PI /180,0.1*IMU.gyroRaw[1]*(-2000.0*M_PI / 5898240.0));
    Tx=((-Kp*((sensors.imu_.rpy[0]-0.0)*M_PI /180))+(-Ki*sensors.imu_.gyroRaw[0]*(2000.0*M_PI / 5898240.0))+(2.2*(position_x-xd))+(0.0*(vx-0.005)))+1.7*sumx_err;
    Ty=((-Kp*(sensors.imu_.rpy[1]-0.0)*M_PI /180)+(-Ki*sensors.imu_.gyroRaw[1]*(2000.0*M_PI / 5898240.0))+(3.7*(position_y-yd))+(0.0*(vy-0.0)));    

    /*Serial.print(position_x);
    Serial.print(" ");
    Serial.print(position_y);
    Serial.print("\n");*/
    /*Tx=-3.5*(position_x-0.5);
    Ty=0;*/
    Tz=0;
    wheel_angular_velocity[0] =(2.0*cb)/(3.0*ca)*Ty-(2.0*sb)/(3.0*ca)*Tx+1/(3.0*sa)*Tz;
    wheel_angular_velocity[1] = (-cb+sqrt(3)*sb)/(3*ca)*Ty+(sb+sqrt(3)*cb)/(3*ca)*Tx+1/(3*sa)*Tz; 
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
    return;
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
  static uint32_t tTime=0;
  prev_update_time = time_now;
  //if(sqrt(pow(xd-position_x,2)+pow(yd-position_y,2))<=0.005)
  w1=TICK2RAD * (double)last_diff_tick[FIRST]/((double)(step_time * 0.001));
  w2=TICK2RAD * (double)last_diff_tick[SECOND]/((double)(step_time * 0.001));
  w3=TICK2RAD * (double)last_diff_tick[THIRD]/((double)(step_time * 0.001));
  
  vx=sqrt(3)/(3*ca)*0.05*(w2-w3);
  vy=0.05/(3*ca)*(-2*w1+w2+w3);
  position_x=position_x+(sqrt(3))/(3*ca)*0.05*TICK2RAD *(last_diff_tick[SECOND]-last_diff_tick[THIRD])*0.62;
  position_y=position_y+(0.05)/(3*ca)*TICK2RAD *(2*last_diff_tick[FIRST]-last_diff_tick[SECOND]-last_diff_tick[THIRD])*0.62;
  sumx_err=sumx_err+(position_x-xd)*step_time*0.001;
  if(sumx_err>0.30)
    {sumx_err=0.30;}
  if(sumx_err<-0.30)
    {sumx_err=-0.30;}
  odom_pose[0]=position_x;  
  odom_pose[1]=position_y;
  odom_vel[0]=vx;
  odom_vel[1]=vy;
  if(time_1* 0.001>10.2)
      {xd=xd+step_time* 0.001*0.01;vd=0.01;}
  else if(time_1* 0.001>10)
      {xd=-0.05;vd=-0.05;}
  if (xd>=0.25) {xd =0.25;vd=0;}
  if ((time_1-tTime)*0.001>0.5 )
  {
    tTime=time_1;
    //Serial.print("\n x : "); Serial.print(position_x);
    //Serial.print("\n xd : "); Serial.print(xd);
    //Serial.print("\n y : "); Serial.print(position_y);
  }
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
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

}
/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}
/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();


      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  ;
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] ;

  sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}
/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();


  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}


/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}
/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      nh.loginfo(log_msg); 

      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}
