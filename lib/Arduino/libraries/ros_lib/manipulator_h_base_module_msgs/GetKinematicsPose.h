#ifndef _ROS_SERVICE_GetKinematicsPose_h
#define _ROS_SERVICE_GetKinematicsPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace manipulator_h_base_module_msgs
{

static const char GETKINEMATICSPOSE[] = "manipulator_h_base_module_msgs/GetKinematicsPose";

  class GetKinematicsPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _group_name_type;
      _group_name_type group_name;

    GetKinematicsPoseRequest():
      group_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_group_name = strlen(this->group_name);
      varToArr(outbuffer + offset, length_group_name);
      offset += 4;
      memcpy(outbuffer + offset, this->group_name, length_group_name);
      offset += length_group_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_group_name;
      arrToVar(length_group_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_name-1]=0;
      this->group_name = (char *)(inbuffer + offset-1);
      offset += length_group_name;
     return offset;
    }

    const char * getType(){ return GETKINEMATICSPOSE; };
    const char * getMD5(){ return "967d0b0c0d858ded8a6a69abbce0c981"; };

  };

  class GetKinematicsPoseResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _group_pose_type;
      _group_pose_type group_pose;
      uint32_t euler_length;
      typedef float _euler_type;
      _euler_type st_euler;
      _euler_type * euler;
      uint32_t orientation_length;
      typedef float _orientation_type;
      _orientation_type st_orientation;
      _orientation_type * orientation;
      typedef float _phi_type;
      _phi_type phi;

    GetKinematicsPoseResponse():
      group_pose(),
      euler_length(0), euler(NULL),
      orientation_length(0), orientation(NULL),
      phi(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->group_pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->euler_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->euler_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->euler_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->euler_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->euler_length);
      for( uint32_t i = 0; i < euler_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->euler[i]);
      }
      *(outbuffer + offset + 0) = (this->orientation_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->orientation_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->orientation_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->orientation_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_length);
      for( uint32_t i = 0; i < orientation_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->orientation[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->phi);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->group_pose.deserialize(inbuffer + offset);
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
      uint32_t orientation_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      orientation_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->orientation_length);
      if(orientation_lengthT > orientation_length)
        this->orientation = (float*)realloc(this->orientation, orientation_lengthT * sizeof(float));
      orientation_length = orientation_lengthT;
      for( uint32_t i = 0; i < orientation_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_orientation));
        memcpy( &(this->orientation[i]), &(this->st_orientation), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phi));
     return offset;
    }

    const char * getType(){ return GETKINEMATICSPOSE; };
    const char * getMD5(){ return "cdd5aca28f89aaebaf1168c873d0293f"; };

  };

  class GetKinematicsPose {
    public:
    typedef GetKinematicsPoseRequest Request;
    typedef GetKinematicsPoseResponse Response;
  };

}
#endif
