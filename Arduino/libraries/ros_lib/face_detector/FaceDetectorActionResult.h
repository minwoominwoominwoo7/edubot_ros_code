#ifndef _ROS_face_detector_FaceDetectorActionResult_h
#define _ROS_face_detector_FaceDetectorActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "face_detector/FaceDetectorResult.h"

namespace face_detector
{

  class FaceDetectorActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef face_detector::FaceDetectorResult _result_type;
      _result_type result;

    FaceDetectorActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "face_detector/FaceDetectorActionResult"; };
    const char * getMD5(){ return "d3986ecc4dd47eb1142da1c68ce02ae7"; };

  };

}
#endif
