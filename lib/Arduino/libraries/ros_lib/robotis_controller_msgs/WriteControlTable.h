#ifndef _ROS_robotis_controller_msgs_WriteControlTable_h
#define _ROS_robotis_controller_msgs_WriteControlTable_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotis_controller_msgs
{

  class WriteControlTable : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef const char* _start_item_name_type;
      _start_item_name_type start_item_name;
      typedef uint16_t _data_length_type;
      _data_length_type data_length;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;

    WriteControlTable():
      joint_name(""),
      start_item_name(""),
      data_length(0),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      uint32_t length_start_item_name = strlen(this->start_item_name);
      varToArr(outbuffer + offset, length_start_item_name);
      offset += 4;
      memcpy(outbuffer + offset, this->start_item_name, length_start_item_name);
      offset += length_start_item_name;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data_length);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
      uint32_t length_start_item_name;
      arrToVar(length_start_item_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_start_item_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_start_item_name-1]=0;
      this->start_item_name = (char *)(inbuffer + offset-1);
      offset += length_start_item_name;
      this->data_length =  ((uint16_t) (*(inbuffer + offset)));
      this->data_length |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->data_length);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "robotis_controller_msgs/WriteControlTable"; };
    const char * getMD5(){ return "0d4e0e3e28ede43d877e2fb8f0a1a011"; };

  };

}
#endif
