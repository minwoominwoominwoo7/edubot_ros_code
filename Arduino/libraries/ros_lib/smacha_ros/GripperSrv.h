#ifndef _ROS_SERVICE_GripperSrv_h
#define _ROS_SERVICE_GripperSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace smacha_ros
{

static const char GRIPPERSRV[] = "smacha_ros/GripperSrv";

  class GripperSrvRequest : public ros::Msg
  {
    public:
      typedef float _max_effort_type;
      _max_effort_type max_effort;
      typedef geometry_msgs::Point _position_type;
      _position_type position;

    GripperSrvRequest():
      max_effort(0),
      position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->max_effort);
      offset += this->position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_effort));
      offset += this->position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GRIPPERSRV; };
    const char * getMD5(){ return "55c5f76802195886850d5d55381f7b6d"; };

  };

  class GripperSrvResponse : public ros::Msg
  {
    public:
      typedef bool _response_type;
      _response_type response;

    GripperSrvResponse():
      response(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_response;
      u_response.real = this->response;
      *(outbuffer + offset + 0) = (u_response.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_response;
      u_response.base = 0;
      u_response.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->response = u_response.real;
      offset += sizeof(this->response);
     return offset;
    }

    const char * getType(){ return GRIPPERSRV; };
    const char * getMD5(){ return "003b81baa95ab323fc1ddf3c7d0bee81"; };

  };

  class GripperSrv {
    public:
    typedef GripperSrvRequest Request;
    typedef GripperSrvResponse Response;
  };

}
#endif
