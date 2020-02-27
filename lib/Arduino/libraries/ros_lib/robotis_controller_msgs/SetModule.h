#ifndef _ROS_SERVICE_SetModule_h
#define _ROS_SERVICE_SetModule_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotis_controller_msgs
{

static const char SETMODULE[] = "robotis_controller_msgs/SetModule";

  class SetModuleRequest : public ros::Msg
  {
    public:
      typedef const char* _module_name_type;
      _module_name_type module_name;

    SetModuleRequest():
      module_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_module_name = strlen(this->module_name);
      varToArr(outbuffer + offset, length_module_name);
      offset += 4;
      memcpy(outbuffer + offset, this->module_name, length_module_name);
      offset += length_module_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_module_name;
      arrToVar(length_module_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_name-1]=0;
      this->module_name = (char *)(inbuffer + offset-1);
      offset += length_module_name;
     return offset;
    }

    const char * getType(){ return SETMODULE; };
    const char * getMD5(){ return "de9d5cb4606cf9ecb083691bbb6f5b15"; };

  };

  class SetModuleResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    SetModuleResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return SETMODULE; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class SetModule {
    public:
    typedef SetModuleRequest Request;
    typedef SetModuleResponse Response;
  };

}
#endif
