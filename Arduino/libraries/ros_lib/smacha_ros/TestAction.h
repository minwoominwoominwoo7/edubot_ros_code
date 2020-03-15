#ifndef _ROS_smacha_ros_TestAction_h
#define _ROS_smacha_ros_TestAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "smacha_ros/TestActionGoal.h"
#include "smacha_ros/TestActionResult.h"
#include "smacha_ros/TestActionFeedback.h"

namespace smacha_ros
{

  class TestAction : public ros::Msg
  {
    public:
      typedef smacha_ros::TestActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef smacha_ros::TestActionResult _action_result_type;
      _action_result_type action_result;
      typedef smacha_ros::TestActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    TestAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "smacha_ros/TestAction"; };
    const char * getMD5(){ return "ab82b566022c36cd43fd6616bdfbeffe"; };

  };

}
#endif
