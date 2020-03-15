#ifndef _ROS_face_detector_FaceDetectorGoal_h
#define _ROS_face_detector_FaceDetectorGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace face_detector
{

  class FaceDetectorGoal : public ros::Msg
  {
    public:

    FaceDetectorGoal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "face_detector/FaceDetectorGoal"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
