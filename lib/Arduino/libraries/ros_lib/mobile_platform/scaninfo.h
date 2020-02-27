#ifndef _ROS_mobile_platform_scaninfo_h
#define _ROS_mobile_platform_scaninfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mobile_platform
{

  class scaninfo : public ros::Msg
  {
    public:
      typedef int32_t _dis_type;
      _dis_type dis;
      typedef float _angle_type;
      _angle_type angle;
      uint32_t scanstate_length;
      typedef int32_t _scanstate_type;
      _scanstate_type st_scanstate;
      _scanstate_type * scanstate;

    scaninfo():
      dis(0),
      angle(0),
      scanstate_length(0), scanstate(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_dis;
      u_dis.real = this->dis;
      *(outbuffer + offset + 0) = (u_dis.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dis.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dis.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dis.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dis);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      *(outbuffer + offset + 0) = (this->scanstate_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scanstate_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scanstate_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scanstate_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scanstate_length);
      for( uint32_t i = 0; i < scanstate_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_scanstatei;
      u_scanstatei.real = this->scanstate[i];
      *(outbuffer + offset + 0) = (u_scanstatei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_scanstatei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_scanstatei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_scanstatei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scanstate[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_dis;
      u_dis.base = 0;
      u_dis.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dis.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dis.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dis.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dis = u_dis.real;
      offset += sizeof(this->dis);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      uint32_t scanstate_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scanstate_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scanstate_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scanstate_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scanstate_length);
      if(scanstate_lengthT > scanstate_length)
        this->scanstate = (int32_t*)realloc(this->scanstate, scanstate_lengthT * sizeof(int32_t));
      scanstate_length = scanstate_lengthT;
      for( uint32_t i = 0; i < scanstate_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_scanstate;
      u_st_scanstate.base = 0;
      u_st_scanstate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_scanstate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_scanstate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_scanstate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_scanstate = u_st_scanstate.real;
      offset += sizeof(this->st_scanstate);
        memcpy( &(this->scanstate[i]), &(this->st_scanstate), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "mobile_platform/scaninfo"; };
    const char * getMD5(){ return "5a20075f483d7bdb2c2ec5ed3a2280bb"; };

  };

}
#endif
