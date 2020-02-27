#ifndef _ROS_robotis_controller_msgs_SyncWriteItem_h
#define _ROS_robotis_controller_msgs_SyncWriteItem_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotis_controller_msgs
{

  class SyncWriteItem : public ros::Msg
  {
    public:
      typedef const char* _item_name_type;
      _item_name_type item_name;
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      uint32_t value_length;
      typedef uint32_t _value_type;
      _value_type st_value;
      _value_type * value;

    SyncWriteItem():
      item_name(""),
      joint_name_length(0), joint_name(NULL),
      value_length(0), value(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_item_name = strlen(this->item_name);
      varToArr(outbuffer + offset, length_item_name);
      offset += 4;
      memcpy(outbuffer + offset, this->item_name, length_item_name);
      offset += length_item_name;
      *(outbuffer + offset + 0) = (this->joint_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_name_length);
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_joint_namei = strlen(this->joint_name[i]);
      varToArr(outbuffer + offset, length_joint_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name[i], length_joint_namei);
      offset += length_joint_namei;
      }
      *(outbuffer + offset + 0) = (this->value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value_length);
      for( uint32_t i = 0; i < value_length; i++){
      *(outbuffer + offset + 0) = (this->value[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_item_name;
      arrToVar(length_item_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_item_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_item_name-1]=0;
      this->item_name = (char *)(inbuffer + offset-1);
      offset += length_item_name;
      uint32_t joint_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_name_length);
      if(joint_name_lengthT > joint_name_length)
        this->joint_name = (char**)realloc(this->joint_name, joint_name_lengthT * sizeof(char*));
      joint_name_length = joint_name_lengthT;
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_st_joint_name;
      arrToVar(length_st_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_name-1]=0;
      this->st_joint_name = (char *)(inbuffer + offset-1);
      offset += length_st_joint_name;
        memcpy( &(this->joint_name[i]), &(this->st_joint_name), sizeof(char*));
      }
      uint32_t value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->value_length);
      if(value_lengthT > value_length)
        this->value = (uint32_t*)realloc(this->value, value_lengthT * sizeof(uint32_t));
      value_length = value_lengthT;
      for( uint32_t i = 0; i < value_length; i++){
      this->st_value =  ((uint32_t) (*(inbuffer + offset)));
      this->st_value |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_value |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_value |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(uint32_t));
      }
     return offset;
    }

    const char * getType(){ return "robotis_controller_msgs/SyncWriteItem"; };
    const char * getMD5(){ return "f558e04f04dbcc25ce64aa1f45f3dbdd"; };

  };

}
#endif
