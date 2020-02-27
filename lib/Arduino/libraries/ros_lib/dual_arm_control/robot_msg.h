#ifndef _ROS_dual_arm_control_robot_msg_h
#define _ROS_dual_arm_control_robot_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dual_arm_control
{

  class robot_msg : public ros::Msg
  {
    public:
      typedef float _A_type;
      _A_type A;

    robot_msg():
      A(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_A;
      u_A.real = this->A;
      *(outbuffer + offset + 0) = (u_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_A.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_A.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->A);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_A;
      u_A.base = 0;
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->A = u_A.real;
      offset += sizeof(this->A);
     return offset;
    }

    const char * getType(){ return "dual_arm_control/robot_msg"; };
    const char * getMD5(){ return "86dc5e57feab1a2b50e6db6b5a647d08"; };

  };

}
#endif
