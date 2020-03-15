#ifndef _ROS_SERVICE_Contain_h
#define _ROS_SERVICE_Contain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smacha_ros
{

static const char CONTAIN[] = "smacha_ros/Contain";

  class ContainRequest : public ros::Msg
  {
    public:
      typedef const char* _script_type;
      _script_type script;
      typedef const char* _container_name_type;
      _container_name_type container_name;
      typedef const char* _container_type_type;
      _container_type_type container_type;
      uint32_t states_length;
      typedef char* _states_type;
      _states_type st_states;
      _states_type * states;

    ContainRequest():
      script(""),
      container_name(""),
      container_type(""),
      states_length(0), states(NULL)
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
      uint32_t length_container_name = strlen(this->container_name);
      varToArr(outbuffer + offset, length_container_name);
      offset += 4;
      memcpy(outbuffer + offset, this->container_name, length_container_name);
      offset += length_container_name;
      uint32_t length_container_type = strlen(this->container_type);
      varToArr(outbuffer + offset, length_container_type);
      offset += 4;
      memcpy(outbuffer + offset, this->container_type, length_container_type);
      offset += length_container_type;
      *(outbuffer + offset + 0) = (this->states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->states_length);
      for( uint32_t i = 0; i < states_length; i++){
      uint32_t length_statesi = strlen(this->states[i]);
      varToArr(outbuffer + offset, length_statesi);
      offset += 4;
      memcpy(outbuffer + offset, this->states[i], length_statesi);
      offset += length_statesi;
      }
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
      uint32_t length_container_name;
      arrToVar(length_container_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_container_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_container_name-1]=0;
      this->container_name = (char *)(inbuffer + offset-1);
      offset += length_container_name;
      uint32_t length_container_type;
      arrToVar(length_container_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_container_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_container_type-1]=0;
      this->container_type = (char *)(inbuffer + offset-1);
      offset += length_container_type;
      uint32_t states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->states_length);
      if(states_lengthT > states_length)
        this->states = (char**)realloc(this->states, states_lengthT * sizeof(char*));
      states_length = states_lengthT;
      for( uint32_t i = 0; i < states_length; i++){
      uint32_t length_st_states;
      arrToVar(length_st_states, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_states; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_states-1]=0;
      this->st_states = (char *)(inbuffer + offset-1);
      offset += length_st_states;
        memcpy( &(this->states[i]), &(this->st_states), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return CONTAIN; };
    const char * getMD5(){ return "8e0e39ec70b8269bf1c11757fc5d287d"; };

  };

  class ContainResponse : public ros::Msg
  {
    public:
      typedef const char* _script_type;
      _script_type script;

    ContainResponse():
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

    const char * getType(){ return CONTAIN; };
    const char * getMD5(){ return "450e743c8271cfb09f02d096e428927f"; };

  };

  class Contain {
    public:
    typedef ContainRequest Request;
    typedef ContainResponse Response;
  };

}
#endif
