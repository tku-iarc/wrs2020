#ifndef _ROS_SERVICE_GetJointModule_h
#define _ROS_SERVICE_GetJointModule_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotis_controller_msgs
{

static const char GETJOINTMODULE[] = "robotis_controller_msgs/GetJointModule";

  class GetJointModuleRequest : public ros::Msg
  {
    public:
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;

    GetJointModuleRequest():
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

    const char * getType(){ return GETJOINTMODULE; };
    const char * getMD5(){ return "01d1dab1fc816c24d1eda912a6b60345"; };

  };

  class GetJointModuleResponse : public ros::Msg
  {
    public:
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      uint32_t module_name_length;
      typedef char* _module_name_type;
      _module_name_type st_module_name;
      _module_name_type * module_name;

    GetJointModuleResponse():
      joint_name_length(0), joint_name(NULL),
      module_name_length(0), module_name(NULL)
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
      *(outbuffer + offset + 0) = (this->module_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->module_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->module_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->module_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->module_name_length);
      for( uint32_t i = 0; i < module_name_length; i++){
      uint32_t length_module_namei = strlen(this->module_name[i]);
      varToArr(outbuffer + offset, length_module_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->module_name[i], length_module_namei);
      offset += length_module_namei;
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
      uint32_t module_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      module_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      module_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      module_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->module_name_length);
      if(module_name_lengthT > module_name_length)
        this->module_name = (char**)realloc(this->module_name, module_name_lengthT * sizeof(char*));
      module_name_length = module_name_lengthT;
      for( uint32_t i = 0; i < module_name_length; i++){
      uint32_t length_st_module_name;
      arrToVar(length_st_module_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_module_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_module_name-1]=0;
      this->st_module_name = (char *)(inbuffer + offset-1);
      offset += length_st_module_name;
        memcpy( &(this->module_name[i]), &(this->st_module_name), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return GETJOINTMODULE; };
    const char * getMD5(){ return "1f9dc32600ec95afe667839e777ecbdd"; };

  };

  class GetJointModule {
    public:
    typedef GetJointModuleRequest Request;
    typedef GetJointModuleResponse Response;
  };

}
#endif
