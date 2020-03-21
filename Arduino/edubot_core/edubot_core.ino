/*******************************************************************************
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

/* Authors:  */

#include "edubot_core_config.h"

#define ESP32

const char*    ssid     = "U+NetB277";   //  wifi ssid
const char*    password = "32F200000";  //  wifi password
IPAddress      server(192,100,100,120);  //  Set the rosserial socket server IP address
const uint16_t serverPort = 11411;      // Set the rosserial socket server port

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  edubot.begin(115200);

  // Connect the ESP32 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(reset_sub);
  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(range_left_pub);
  nh.advertise(range_right_pub);

  tf_broadcaster.init(nh);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom(); 
  initJointStates(); 
  prev_update_time = millis();
  setup_end = true;

}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{ 

  uint32_t t = millis();

  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
    {
      controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    } 
    else {
      controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    }
    tTime[0] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishBatteryStateMsg();
    publishDriveInformation();
    publishRangeMsg();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();  
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  // Send log message after ROS connection
  sendLogMsg();


  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());

}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  //sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0}; /// stores arduino time
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT] = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT] = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
  joint_states.effort = joint_states_eff;    
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish range (Distance data)
*******************************************************************************/
void publishRangeMsg(void)
{
  range_left_msg.header.stamp = rosNow();
  range_left_msg.header.frame_id = tof_left_frame_id ;
  range_left_msg.header.seq = 0 ;
  range_left_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_left_msg.field_of_view = 0.436332;  // 25 degree FOV -> 0.436332 rad
  range_left_msg.min_range = 0;  // 0 meter
  range_left_msg.max_range = ((float)edubot.tof_L.signal_rate);  // data chang singal_rate data
  range_left_msg.range = ((float)edubot.tof_L.distance_mm / 1000); // meter
  range_left_pub.publish(&range_left_msg);

  range_right_msg.header.stamp = rosNow();
  range_right_msg.header.frame_id = tof_right_frame_id;
  range_right_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_right_msg.field_of_view = 0.436332;  // 25 degree FOV -> 0.436332 rad
  range_right_msg.min_range = 0;  // 0 meter
  range_right_msg.max_range = ((float)edubot.tof_R.signal_rate);  // data chang singal_rate data
  range_right_msg.range = ((float)edubot.tof_R.distance_mm /1000); // meter  
  range_right_pub.publish(&range_right_msg);
}


/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 0.65f; //Ah
  battery_state_msg.voltage = edubot.batteryGetVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 3.2f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
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
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // get motor encorder 
  updateMotorInfo(edubot.motor.getLeftStep(), edubot.motor.getRightStep());

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

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
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");

        sprintf(tof_left_frame_id, "tof_left_link");
        sprintf(tof_right_frame_id, "tof_right_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcpy(tof_left_frame_id, get_tf_prefix);
        strcpy(tof_right_frame_id, get_tf_prefix);        

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");

        strcat(tof_left_frame_id, "/tof_left_link");
        strcat(tof_right_frame_id, "/tof_right_link");        
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on TOF LEFT [%s]", tof_left_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on TOF RIGHT [%s]", tof_right_frame_id);
      nh.loginfo(log_msg);             

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
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
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Returns the index of the array value closest to p in the given array 
*******************************************************************************/
 int nearArray( int p)
 {
   int tmp = 0;
   int min = 200000;
   int near = 0;
   int found = 0; 
   for(int i=0; i<301; i++)
   {
     tmp = m_TimeTableTemp[i] - p;
     
     if (abs(min) > abs(tmp))
     {
       min = tmp;
       near = m_TimeTableTemp[i];
       found = i;
     }
   }
   return found;
}

/*******************************************************************************
* Send Debug data
*******************************************************************************/

bool controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{    

  float wheel_velocity_cmd[2];
  
  int edubot_step_period[WHEEL_NUM] = {0,0};
  int edubot_step_period2[WHEEL_NUM] = {0,0};
  int edubot_speed_unit[WHEEL_NUM] = {0,0};

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  if( abs(wheel_velocity_cmd[LEFT]) < 0.001 ){

    edubot_speed_unit[LEFT] = 0 ;

  }else if( wheel_velocity_cmd[LEFT] > 0 ){
    
    // 모터 스텝각          : 18도
    // 모터 1회전시 스텝 수  : 360/18 = 20;
    // 휠 1회전시 스텝 수    : 1000 = 20 * 기어비(50)
    // 스텝 1개 발생 시간 : m_TimeTable[unit] usec  // unit 은 0 ~ 300 
    // 휠반지름 : r meter // 17mm = 0.017 m
    // 1개 step 발생 주기 usec = 2*pi*r휠반지름(m) * 1000 *1000/ ( 휠 Speed ( m/sec ) * 휠 1회전시 스텝수 )
    //                       = 2*3.14159265359*0.017 * 1000 *1000/ ( 휠 Speed ( m/sec ) * 1000 )
    //                       = 106.81415 / 휠 Speed ( m/sec )

    edubot_step_period[LEFT] = (int)106.81415 / wheel_velocity_cmd[LEFT] ;
    edubot_step_period2[LEFT] = constrain(edubot_step_period[LEFT] , -m_TimeTableTemp[1], m_TimeTableTemp[1]);
    edubot_speed_unit[LEFT] = nearArray(edubot_step_period2[LEFT]);

  }else if( wheel_velocity_cmd[LEFT] < 0 ){

    edubot_step_period[LEFT] = (int)106.81415 / -wheel_velocity_cmd[LEFT] ;
    edubot_step_period2[LEFT] = constrain(edubot_step_period[LEFT] , -m_TimeTableTemp[1], m_TimeTableTemp[1]);
    edubot_speed_unit[LEFT] = -nearArray(edubot_step_period2[LEFT]);
  }

  if( abs(wheel_velocity_cmd[RIGHT]) < 0.001 ){

    edubot_speed_unit[RIGHT] = 0 ;

  }else if( wheel_velocity_cmd[RIGHT] > 0 ){

    edubot_step_period[RIGHT] = (int)106.81415 / wheel_velocity_cmd[RIGHT] ;
    edubot_step_period2[RIGHT] = constrain(edubot_step_period[RIGHT] , -m_TimeTableTemp[1], m_TimeTableTemp[1]);
    edubot_speed_unit[RIGHT] = nearArray(edubot_step_period2[RIGHT]);

  }else if( wheel_velocity_cmd[RIGHT] < 0 ){

    edubot_step_period[RIGHT] = (int)106.81415 / -wheel_velocity_cmd[RIGHT] ;
    edubot_step_period2[RIGHT] = constrain(edubot_step_period[RIGHT] , -m_TimeTableTemp[1], m_TimeTableTemp[1]);
    edubot_speed_unit[RIGHT] = -nearArray(edubot_step_period2[RIGHT]);

  }  

  edubot.motor.setSpeed(edubot_speed_unit[LEFT],edubot_speed_unit[RIGHT]); 

  Serial.print( ang_vel, 5 ); 
  Serial.print( ", " ) ;
  Serial.print( edubot_speed_unit[LEFT]) ;
  Serial.print( ", " ) ;
  Serial.println( edubot_speed_unit[RIGHT]) ;

  return true;
}


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0; 
  //DEBUG_SERIAL.print("delta_s : ");DEBUG_SERIAL.println(delta_s, 10 ) ;
    
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  //orientation = sensors.getOrientation();
  /*theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);*/
  theta = DEG2RAD(edubot.imu.getYaw());
  //theta = -DEG2RAD(edubot.motor.getAngle());
  delta_theta = theta - last_theta;
  
  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  //odom_pose[0] = edubot.motor.getY()/1000;
  //odom_pose[1] = edubot.motor.getX()/1000; 

  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  v = delta_s / step_time;
  w = delta_theta / step_time;
  
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;  
  
  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
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
      //sensors.initIMU();
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
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with Edubot " + name;
   
  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to Edubot!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
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
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

}

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Step Motor");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Encoder(left) : " + String(edubot.motor.getLeftStep()));
  DEBUG_SERIAL.println("Encoder(right) : " + String(edubot.motor.getRightStep()));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Edubot");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");   
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}

sensor_msgs::Imu getIMU(void)
{
  sensor_msgs::Imu           imu_msg_;
  imu_msg_.angular_velocity.x = edubot.imu.getGyroX();
  imu_msg_.angular_velocity.y = edubot.imu.getGyroY();
  imu_msg_.angular_velocity.z = edubot.imu.getGyroZ();
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = edubot.imu.getAccX();
  imu_msg_.linear_acceleration.y = edubot.imu.getAccY();
  imu_msg_.linear_acceleration.z = edubot.imu.getAccZ(); 

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(edubot.imu.getYaw());

  imu_msg_.orientation.w = odom_quat.w;
  imu_msg_.orientation.x = odom_quat.x;
  imu_msg_.orientation.y = odom_quat.y;
  imu_msg_.orientation.z = odom_quat.z;

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;  
}
