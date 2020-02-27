#ifndef _ROS_linear_motion_Slide_Feedback_h
#define _ROS_linear_motion_Slide_Feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace linear_motion
{

  class Slide_Feedback : public ros::Msg
  {
    public:
      typedef int32_t _curr_pos_type;
      _curr_pos_type curr_pos;
      typedef bool _is_busy_type;
      _is_busy_type is_busy;

    Slide_Feedback():
      curr_pos(0),
      is_busy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_curr_pos;
      u_curr_pos.real = this->curr_pos;
      *(outbuffer + offset + 0) = (u_curr_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_curr_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_curr_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_curr_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->curr_pos);
      union {
        bool real;
        uint8_t base;
      } u_is_busy;
      u_is_busy.real = this->is_busy;
      *(outbuffer + offset + 0) = (u_is_busy.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_busy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_curr_pos;
      u_curr_pos.base = 0;
      u_curr_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_curr_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_curr_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_curr_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->curr_pos = u_curr_pos.real;
      offset += sizeof(this->curr_pos);
      union {
        bool real;
        uint8_t base;
      } u_is_busy;
      u_is_busy.base = 0;
      u_is_busy.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_busy = u_is_busy.real;
      offset += sizeof(this->is_busy);
     return offset;
    }

    const char * getType(){ return "linear_motion/Slide_Feedback"; };
    const char * getMD5(){ return "dc38682b93f23bb5e4355ce1732c0d4b"; };

  };

}
#endif
