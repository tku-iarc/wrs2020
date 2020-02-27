#ifndef _ROS_SERVICE_GetJointPose_h
#define _ROS_SERVICE_GetJointPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace manipulator_h_base_module_msgs
{

static const char GETJOINTPOSE[] = "manipulator_h_base_module_msgs/GetJointPose";

  class GetJointPoseRequest : public ros::Msg
  {
    public:
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;

    GetJointPoseRequest():
      joint_name_length(0), joint_name(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
     return offset;
    }

    const char * getType(){ return GETJOINTPOSE; };
    const char * getMD5(){ return "01d1dab1fc816c24d1eda912a6b60345"; };

  };

  class GetJointPoseResponse : public ros::Msg
  {
    public:
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      uint32_t joint_value_length;
      typedef float _joint_value_type;
      _joint_value_type st_joint_value;
      _joint_value_type * joint_value;
      typedef float _slide_pos_type;
      _slide_pos_type slide_pos;

    GetJointPoseResponse():
      joint_name_length(0), joint_name(NULL),
      joint_value_length(0), joint_value(NULL),
      slide_pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      *(outbuffer + offset + 0) = (this->joint_value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_value_length);
      for( uint32_t i = 0; i < joint_value_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_value[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->slide_pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      uint32_t joint_value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_value_length);
      if(joint_value_lengthT > joint_value_length)
        this->joint_value = (float*)realloc(this->joint_value, joint_value_lengthT * sizeof(float));
      joint_value_length = joint_value_lengthT;
      for( uint32_t i = 0; i < joint_value_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_value));
        memcpy( &(this->joint_value[i]), &(this->st_joint_value), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->slide_pos));
     return offset;
    }

    const char * getType(){ return GETJOINTPOSE; };
    const char * getMD5(){ return "1d82d4fe7a659b1644c882a530422535"; };

  };

  class GetJointPose {
    public:
    typedef GetJointPoseRequest Request;
    typedef GetJointPoseResponse Response;
  };

}
#endif
