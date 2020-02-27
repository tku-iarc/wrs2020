#ifndef _ROS_robotis_controller_msgs_StatusMsg_h
#define _ROS_robotis_controller_msgs_StatusMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robotis_controller_msgs
{

  class StatusMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _type_type;
      _type_type type;
      typedef const char* _module_name_type;
      _module_name_type module_name;
      typedef const char* _status_msg_type;
      _status_msg_type status_msg;
      enum { STATUS_UNKNOWN =  0 };
      enum { STATUS_INFO =  1 };
      enum { STATUS_WARN =  2 };
      enum { STATUS_ERROR =  3 };

    StatusMsg():
      header(),
      type(0),
      module_name(""),
      status_msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      uint32_t length_module_name = strlen(this->module_name);
      varToArr(outbuffer + offset, length_module_name);
      offset += 4;
      memcpy(outbuffer + offset, this->module_name, length_module_name);
      offset += length_module_name;
      uint32_t length_status_msg = strlen(this->status_msg);
      varToArr(outbuffer + offset, length_status_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->status_msg, length_status_msg);
      offset += length_status_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t length_module_name;
      arrToVar(length_module_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_name-1]=0;
      this->module_name = (char *)(inbuffer + offset-1);
      offset += length_module_name;
      uint32_t length_status_msg;
      arrToVar(length_status_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_msg-1]=0;
      this->status_msg = (char *)(inbuffer + offset-1);
      offset += length_status_msg;
     return offset;
    }

    const char * getType(){ return "robotis_controller_msgs/StatusMsg"; };
    const char * getMD5(){ return "8d546af394a35cb47516d4d064603220"; };

  };

}
#endif
