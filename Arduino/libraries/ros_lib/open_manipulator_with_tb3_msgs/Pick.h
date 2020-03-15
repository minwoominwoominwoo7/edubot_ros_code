#ifndef _ROS_SERVICE_Pick_h
#define _ROS_SERVICE_Pick_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_with_tb3_msgs
{

static const char PICK[] = "open_manipulator_with_tb3_msgs/Pick";

  class PickRequest : public ros::Msg
  {
    public:
      typedef const char* _state_type;
      _state_type state;

    PickRequest():
      state("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
     return offset;
    }

    const char * getType(){ return PICK; };
    const char * getMD5(){ return "af6d3a99f0fbeb66d3248fa4b3e675fb"; };

  };

  class PickResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;

    PickResponse():
      result("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_result = strlen(this->result);
      varToArr(outbuffer + offset, length_result);
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_result;
      arrToVar(length_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
     return offset;
    }

    const char * getType(){ return PICK; };
    const char * getMD5(){ return "c22f2a1ed8654a0b365f1bb3f7ff2c0f"; };

  };

  class Pick {
    public:
    typedef PickRequest Request;
    typedef PickResponse Response;
  };

}
#endif
