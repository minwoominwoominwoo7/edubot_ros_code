#ifndef _ROS_SERVICE_Generate_h
#define _ROS_SERVICE_Generate_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smacha_ros
{

static const char GENERATE[] = "smacha_ros/Generate";

  class GenerateRequest : public ros::Msg
  {
    public:
      typedef const char* _script_type;
      _script_type script;

    GenerateRequest():
      script("")
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
     return offset;
    }

    const char * getType(){ return GENERATE; };
    const char * getMD5(){ return "450e743c8271cfb09f02d096e428927f"; };

  };

  class GenerateResponse : public ros::Msg
  {
    public:
      typedef const char* _code_type;
      _code_type code;

    GenerateResponse():
      code("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_code = strlen(this->code);
      varToArr(outbuffer + offset, length_code);
      offset += 4;
      memcpy(outbuffer + offset, this->code, length_code);
      offset += length_code;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_code;
      arrToVar(length_code, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_code; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_code-1]=0;
      this->code = (char *)(inbuffer + offset-1);
      offset += length_code;
     return offset;
    }

    const char * getType(){ return GENERATE; };
    const char * getMD5(){ return "9d589d39a46fa5aba4838a23b9cc4e62"; };

  };

  class Generate {
    public:
    typedef GenerateRequest Request;
    typedef GenerateResponse Response;
  };

}
#endif
