#ifndef _ROS_SERVICE_LetRobotSay_h
#define _ROS_SERVICE_LetRobotSay_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nodejs_pkg
{

static const char LETROBOTSAY[] = "nodejs_pkg/LetRobotSay";

  class LetRobotSayRequest : public ros::Msg
  {
    public:
      typedef const char* _info_type;
      _info_type info;

    LetRobotSayRequest():
      info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_info = strlen(this->info);
      varToArr(outbuffer + offset, length_info);
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_info;
      arrToVar(length_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
     return offset;
    }

    const char * getType(){ return LETROBOTSAY; };
    const char * getMD5(){ return "c10fc26d5cca9a4b9114f5fc5dea9570"; };

  };

  class LetRobotSayResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    LetRobotSayResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return LETROBOTSAY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class LetRobotSay {
    public:
    typedef LetRobotSayRequest Request;
    typedef LetRobotSayResponse Response;
  };

}
#endif
