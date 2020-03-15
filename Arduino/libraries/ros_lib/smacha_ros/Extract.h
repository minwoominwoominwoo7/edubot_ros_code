#ifndef _ROS_SERVICE_Extract_h
#define _ROS_SERVICE_Extract_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smacha_ros
{

static const char EXTRACT[] = "smacha_ros/Extract";

  class ExtractRequest : public ros::Msg
  {
    public:
      typedef const char* _script_type;
      _script_type script;
      typedef const char* _state_type;
      _state_type state;

    ExtractRequest():
      script(""),
      state("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_script = strlen(this->script);
      varToArr(outbuffer + offset, length_script);
      offset += 4;
      memcpy(outbuffer + offset, this->script, length_script);
      offset += length_script;
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
      uint32_t length_script;
      arrToVar(length_script, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_script; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_script-1]=0;
      this->script = (char *)(inbuffer + offset-1);
      offset += length_script;
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

    const char * getType(){ return EXTRACT; };
    const char * getMD5(){ return "483942736ee5c2d408652855689402a2"; };

  };

  class ExtractResponse : public ros::Msg
  {
    public:
      typedef const char* _sub_script_type;
      _sub_script_type sub_script;
      typedef const char* _super_script_type;
      _super_script_type super_script;

    ExtractResponse():
      sub_script(""),
      super_script("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_sub_script = strlen(this->sub_script);
      varToArr(outbuffer + offset, length_sub_script);
      offset += 4;
      memcpy(outbuffer + offset, this->sub_script, length_sub_script);
      offset += length_sub_script;
      uint32_t length_super_script = strlen(this->super_script);
      varToArr(outbuffer + offset, length_super_script);
      offset += 4;
      memcpy(outbuffer + offset, this->super_script, length_super_script);
      offset += length_super_script;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_sub_script;
      arrToVar(length_sub_script, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sub_script; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sub_script-1]=0;
      this->sub_script = (char *)(inbuffer + offset-1);
      offset += length_sub_script;
      uint32_t length_super_script;
      arrToVar(length_super_script, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_super_script; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_super_script-1]=0;
      this->super_script = (char *)(inbuffer + offset-1);
      offset += length_super_script;
     return offset;
    }

    const char * getType(){ return EXTRACT; };
    const char * getMD5(){ return "3fdcf85a4130562d243e69943565c122"; };

  };

  class Extract {
    public:
    typedef ExtractRequest Request;
    typedef ExtractResponse Response;
  };

}
#endif
