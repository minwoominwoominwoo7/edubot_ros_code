#ifndef _ROS_smacha_ros_TestGoal_h
#define _ROS_smacha_ros_TestGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smacha_ros
{

  class TestGoal : public ros::Msg
  {
    public:
      typedef float _goal_type;
      _goal_type goal;

    TestGoal():
      goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal));
     return offset;
    }

    const char * getType(){ return "smacha_ros/TestGoal"; };
    const char * getMD5(){ return "96f1fc969cebfe9056357b5db1aa501e"; };

  };

}
#endif
