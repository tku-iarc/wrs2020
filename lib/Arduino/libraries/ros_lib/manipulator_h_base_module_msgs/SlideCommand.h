#ifndef _ROS_manipulator_h_base_module_msgs_SlideCommand_h
#define _ROS_manipulator_h_base_module_msgs_SlideCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace manipulator_h_base_module_msgs
{

  class SlideCommand : public ros::Msg
  {
    public:
      typedef float _pos_type;
      _pos_type pos;

    SlideCommand():
      pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos));
     return offset;
    }

    const char * getType(){ return "manipulator_h_base_module_msgs/SlideCommand"; };
    const char * getMD5(){ return "dbf6546e7fe3735fd3cf39bc063251a4"; };

  };

}
#endif
