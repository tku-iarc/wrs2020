#ifndef _ROS_SERVICE_VacuumCmd_h
#define _ROS_SERVICE_VacuumCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vacuum_cmd_msg
{

static const char VACUUMCMD[] = "vacuum_cmd_msg/VacuumCmd";

  class VacuumCmdRequest : public ros::Msg
  {
    public:
      typedef const char* _cmd_type;
      _cmd_type cmd;

    VacuumCmdRequest():
      cmd("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_cmd = strlen(this->cmd);
      varToArr(outbuffer + offset, length_cmd);
      offset += 4;
      memcpy(outbuffer + offset, this->cmd, length_cmd);
      offset += length_cmd;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_cmd;
      arrToVar(length_cmd, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cmd; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cmd-1]=0;
      this->cmd = (char *)(inbuffer + offset-1);
      offset += length_cmd;
     return offset;
    }

    const char * getType(){ return VACUUMCMD; };
    const char * getMD5(){ return "43a54fa49066cddcf148717d9d4a6353"; };

  };

  class VacuumCmdResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    VacuumCmdResponse():
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

    const char * getType(){ return VACUUMCMD; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class VacuumCmd {
    public:
    typedef VacuumCmdRequest Request;
    typedef VacuumCmdResponse Response;
  };

}
#endif
