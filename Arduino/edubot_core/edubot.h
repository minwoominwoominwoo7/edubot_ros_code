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

/* Authors:*/

#ifndef EDUBOT_H_
#define EDUBOT_H_

#define NAME                             "Edubot"

#define WHEEL_RADIUS                     0.017           // meter
#define WHEEL_SEPARATION                 0.057           // meter
#define TURNING_RADIUS                   0.0285           // meter
#define ROBOT_RADIUS                     0.05           // meter 
#define ENCODER_MIN                      -1000     // raw -20*50
#define ENCODER_MAX                     1000      // raw 20*50

#define MAX_LINEAR_VELOCITY              0.18 // m/s  300 unit -> 596 usec step period ->  0.17912 m/sec wheel speed
#define MAX_ANGULAR_VELOCITY             6.23 // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //EDUBOT_H_
