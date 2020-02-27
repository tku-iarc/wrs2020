#ifndef _ROS_SERVICE_strategy_start_h
#define _ROS_SERVICE_strategy_start_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mobile_platform
{

static const char STRATEGY_START[] = "mobile_platform/strategy_start";

  class strategy_startRequest : public ros::Msg
  {
    public:
      typedef bool _data_type;
      _data_type data;

    strategy_startRequest():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return STRATEGY_START; };
    const char * getMD5(){ return "8b94c1b53db61fb6aed406028ad6332a"; };

  };

  class strategy_startResponse : public ros::Msg
  {
    public:
      typedef bool _get_type;
      _get_type get;

    strategy_startResponse():
      get(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_get;
      u_get.real = this->get;
      *(outbuffer + offset + 0) = (u_get.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->get);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_get;
      u_get.base = 0;
      u_get.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->get = u_get.real;
      offset += sizeof(this->get);
     return offset;
    }

    const char * getType(){ return STRATEGY_START; };
    const char * getMD5(){ return "09f518c966a327b7b44a7e1645859313"; };

  };

  class strategy_start {
    public:
    typedef strategy_startRequest Request;
    typedef strategy_startResponse Response;
  };

}
#endif
