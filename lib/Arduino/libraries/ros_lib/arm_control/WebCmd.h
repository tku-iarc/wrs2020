#ifndef _ROS_arm_control_WebCmd_h
#define _ROS_arm_control_WebCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_control
{

  class WebCmd : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _cmd_type;
      _cmd_type cmd;
      typedef const char* _mode_type;
      _mode_type mode;
      uint32_t pose_length;
      typedef float _pose_type;
      _pose_type st_pose;
      _pose_type * pose;
      uint32_t euler_length;
      typedef float _euler_type;
      _euler_type st_euler;
      _euler_type * euler;
      uint32_t noa_length;
      typedef float _noa_type;
      _noa_type st_noa;
      _noa_type * noa;
      typedef float _value_type;
      _value_type value;
      typedef int32_t _joint_type;
      _joint_type joint;

    WebCmd():
      name(""),
      cmd(""),
      mode(""),
      pose_length(0), pose(NULL),
      euler_length(0), euler(NULL),
      noa_length(0), noa(NULL),
      value(0),
      joint(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_cmd = strlen(this->cmd);
      varToArr(outbuffer + offset, length_cmd);
      offset += 4;
      memcpy(outbuffer + offset, this->cmd, length_cmd);
      offset += length_cmd;
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      *(outbuffer + offset + 0) = (this->pose_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pose_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pose_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pose_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_length);
      for( uint32_t i = 0; i < pose_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pose[i]);
      }
      *(outbuffer + offset + 0) = (this->euler_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->euler_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->euler_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->euler_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->euler_length);
      for( uint32_t i = 0; i < euler_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->euler[i]);
      }
      *(outbuffer + offset + 0) = (this->noa_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->noa_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->noa_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->noa_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->noa_length);
      for( uint32_t i = 0; i < noa_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->noa[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      union {
        int32_t real;
        uint32_t base;
      } u_joint;
      u_joint.real = this->joint;
      *(outbuffer + offset + 0) = (u_joint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_cmd;
      arrToVar(length_cmd, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cmd; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cmd-1]=0;
      this->cmd = (char *)(inbuffer + offset-1);
      offset += length_cmd;
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      uint32_t pose_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pose_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pose_length);
      if(pose_lengthT > pose_length)
        this->pose = (float*)realloc(this->pose, pose_lengthT * sizeof(float));
      pose_length = pose_lengthT;
      for( uint32_t i = 0; i < pose_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_pose));
        memcpy( &(this->pose[i]), &(this->st_pose), sizeof(float));
      }
      uint32_t euler_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      euler_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      euler_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      euler_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->euler_length);
      if(euler_lengthT > euler_length)
        this->euler = (float*)realloc(this->euler, euler_lengthT * sizeof(float));
      euler_length = euler_lengthT;
      for( uint32_t i = 0; i < euler_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_euler));
        memcpy( &(this->euler[i]), &(this->st_euler), sizeof(float));
      }
      uint32_t noa_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      noa_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      noa_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      noa_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->noa_length);
      if(noa_lengthT > noa_length)
        this->noa = (float*)realloc(this->noa, noa_lengthT * sizeof(float));
      noa_length = noa_lengthT;
      for( uint32_t i = 0; i < noa_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_noa));
        memcpy( &(this->noa[i]), &(this->st_noa), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
      union {
        int32_t real;
        uint32_t base;
      } u_joint;
      u_joint.base = 0;
      u_joint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint = u_joint.real;
      offset += sizeof(this->joint);
     return offset;
    }

    const char * getType(){ return "arm_control/WebCmd"; };
    const char * getMD5(){ return "871863590aa03fd7e78cced2a913184a"; };

  };

}
#endif
