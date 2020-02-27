#ifndef _ROS_imu_3d_inertia_h
#define _ROS_imu_3d_inertia_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace imu_3d
{

  class inertia : public ros::Msg
  {
    public:
      typedef float _shift_x_type;
      _shift_x_type shift_x;
      typedef float _shift_y_type;
      _shift_y_type shift_y;
      typedef float _yaw_type;
      _yaw_type yaw;

    inertia():
      shift_x(0),
      shift_y(0),
      yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->shift_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->shift_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->shift_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->shift_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
     return offset;
    }

    const char * getType(){ return "imu_3d/inertia"; };
    const char * getMD5(){ return "f936b1ec3455b492b95e76e4a8bd4c8b"; };

  };

}
#endif
