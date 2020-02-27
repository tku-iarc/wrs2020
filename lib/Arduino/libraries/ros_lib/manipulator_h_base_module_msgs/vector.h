#ifndef _ROS_manipulator_h_base_module_msgs_vector_h
#define _ROS_manipulator_h_base_module_msgs_vector_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace manipulator_h_base_module_msgs
{

  class vector : public ros::Msg
  {
    public:
      uint32_t row_length;
      typedef float _row_type;
      _row_type st_row;
      _row_type * row;

    vector():
      row_length(0), row(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->row_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->row_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->row_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->row_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->row_length);
      for( uint32_t i = 0; i < row_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->row[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t row_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      row_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      row_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      row_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->row_length);
      if(row_lengthT > row_length)
        this->row = (float*)realloc(this->row, row_lengthT * sizeof(float));
      row_length = row_lengthT;
      for( uint32_t i = 0; i < row_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_row));
        memcpy( &(this->row[i]), &(this->st_row), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "manipulator_h_base_module_msgs/vector"; };
    const char * getMD5(){ return "7f476b09ffe66a6e073b86346f9b64b0"; };

  };

}
#endif
