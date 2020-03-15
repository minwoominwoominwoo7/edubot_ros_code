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

/* Authors:   */

#ifndef EDUBOT_CORE_CONFIG_H_
#define EDUBOT_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/VersionInfo.h>

#include <EduBot.h>
#include "edubot.h"
#include "steptbltmp.h"

#include <math.h>

#define FIRMWARE_VER "1.2.3"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define ENCODER_RES       (20*50)                               // 1바퀴 회전시 엔코더 펄스 수 
#define TICK2RAD          (((360./ENCODER_RES) * M_PI) / 180.)  // 엔코더 1펄스일때 회전하는 각도(라디안) 

// #define DEBUG                            
#define DEBUG_SERIAL                     Serial

#define EDUBOT_LIMIT_MAX_VELOCITY            300 // 300 unit -> 596 usec step period ->  0.17912 m/sec wheel speed                                         

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishImuMsg(void);
void publishVersionInfoMsg(void);
void publishBatteryStateMsg(void);
void publishDriveInformation(void);
void publishRangeMsg(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateTime(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateGoalVelocity(void);
void updateTFPrefix(bool isConnected);
void initOdom(void);
void initJointStates(void);
bool calcOdometry(double diff_time);
void sendLogMsg(void);
void waitForSerialLink(bool isConnected);
bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);
int nearArray(unsigned int p);
sensor_msgs::Imu getIMU(void);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char tof_left_frame_id[30];
char tof_right_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Version information of Edubot
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Edubot
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(step motor) state of Edubot
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Battey state of Edubot
sensor_msgs::BatteryState battery_state_msg;
ros::Publisher battery_state_pub("battery_state", &battery_state_msg);

// Range field
sensor_msgs::Range range_left_msg;
ros::Publisher range_left_pub("range_left", &range_left_msg);
sensor_msgs::Range range_right_msg;
ros::Publisher range_right_pub("range_right", &range_right_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Edubot
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* SoftwareTimer of Edubot
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

#endif // EDUBOT_CORE_CONFIG_H_
