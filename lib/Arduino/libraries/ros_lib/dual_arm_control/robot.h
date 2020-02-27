#ifndef _ROS_SERVICE_robot_h
#define _ROS_SERVICE_robot_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dual_arm_control
{

static const char ROBOT[] = "dual_arm_control/robot";

  class robotRequest : public ros::Msg
  {
    public:
      typedef float _slide_R_type;
      _slide_R_type slide_R;
      typedef float _joint1_R_type;
      _joint1_R_type joint1_R;
      typedef float _joint2_R_type;
      _joint2_R_type joint2_R;
      typedef float _joint3_R_type;
      _joint3_R_type joint3_R;
      typedef float _joint4_R_type;
      _joint4_R_type joint4_R;
      typedef float _joint5_R_type;
      _joint5_R_type joint5_R;
      typedef float _joint6_R_type;
      _joint6_R_type joint6_R;
      typedef float _joint7_R_type;
      _joint7_R_type joint7_R;
      typedef float _gripper_R_type;
      _gripper_R_type gripper_R;
      typedef float _slide_L_type;
      _slide_L_type slide_L;
      typedef float _joint1_L_type;
      _joint1_L_type joint1_L;
      typedef float _joint2_L_type;
      _joint2_L_type joint2_L;
      typedef float _joint3_L_type;
      _joint3_L_type joint3_L;
      typedef float _joint4_L_type;
      _joint4_L_type joint4_L;
      typedef float _joint5_L_type;
      _joint5_L_type joint5_L;
      typedef float _joint6_L_type;
      _joint6_L_type joint6_L;
      typedef float _joint7_L_type;
      _joint7_L_type joint7_L;
      typedef float _gripper_L_type;
      _gripper_L_type gripper_L;

    robotRequest():
      slide_R(0),
      joint1_R(0),
      joint2_R(0),
      joint3_R(0),
      joint4_R(0),
      joint5_R(0),
      joint6_R(0),
      joint7_R(0),
      gripper_R(0),
      slide_L(0),
      joint1_L(0),
      joint2_L(0),
      joint3_L(0),
      joint4_L(0),
      joint5_L(0),
      joint6_L(0),
      joint7_L(0),
      gripper_L(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_slide_R;
      u_slide_R.real = this->slide_R;
      *(outbuffer + offset + 0) = (u_slide_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slide_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_slide_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_slide_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slide_R);
      union {
        float real;
        uint32_t base;
      } u_joint1_R;
      u_joint1_R.real = this->joint1_R;
      *(outbuffer + offset + 0) = (u_joint1_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint1_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint1_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint1_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint1_R);
      union {
        float real;
        uint32_t base;
      } u_joint2_R;
      u_joint2_R.real = this->joint2_R;
      *(outbuffer + offset + 0) = (u_joint2_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint2_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint2_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint2_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint2_R);
      union {
        float real;
        uint32_t base;
      } u_joint3_R;
      u_joint3_R.real = this->joint3_R;
      *(outbuffer + offset + 0) = (u_joint3_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint3_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint3_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint3_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint3_R);
      union {
        float real;
        uint32_t base;
      } u_joint4_R;
      u_joint4_R.real = this->joint4_R;
      *(outbuffer + offset + 0) = (u_joint4_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint4_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint4_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint4_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint4_R);
      union {
        float real;
        uint32_t base;
      } u_joint5_R;
      u_joint5_R.real = this->joint5_R;
      *(outbuffer + offset + 0) = (u_joint5_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint5_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint5_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint5_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint5_R);
      union {
        float real;
        uint32_t base;
      } u_joint6_R;
      u_joint6_R.real = this->joint6_R;
      *(outbuffer + offset + 0) = (u_joint6_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint6_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint6_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint6_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint6_R);
      union {
        float real;
        uint32_t base;
      } u_joint7_R;
      u_joint7_R.real = this->joint7_R;
      *(outbuffer + offset + 0) = (u_joint7_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint7_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint7_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint7_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint7_R);
      union {
        float real;
        uint32_t base;
      } u_gripper_R;
      u_gripper_R.real = this->gripper_R;
      *(outbuffer + offset + 0) = (u_gripper_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper_R.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper_R.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_R);
      union {
        float real;
        uint32_t base;
      } u_slide_L;
      u_slide_L.real = this->slide_L;
      *(outbuffer + offset + 0) = (u_slide_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slide_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_slide_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_slide_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slide_L);
      union {
        float real;
        uint32_t base;
      } u_joint1_L;
      u_joint1_L.real = this->joint1_L;
      *(outbuffer + offset + 0) = (u_joint1_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint1_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint1_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint1_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint1_L);
      union {
        float real;
        uint32_t base;
      } u_joint2_L;
      u_joint2_L.real = this->joint2_L;
      *(outbuffer + offset + 0) = (u_joint2_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint2_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint2_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint2_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint2_L);
      union {
        float real;
        uint32_t base;
      } u_joint3_L;
      u_joint3_L.real = this->joint3_L;
      *(outbuffer + offset + 0) = (u_joint3_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint3_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint3_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint3_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint3_L);
      union {
        float real;
        uint32_t base;
      } u_joint4_L;
      u_joint4_L.real = this->joint4_L;
      *(outbuffer + offset + 0) = (u_joint4_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint4_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint4_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint4_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint4_L);
      union {
        float real;
        uint32_t base;
      } u_joint5_L;
      u_joint5_L.real = this->joint5_L;
      *(outbuffer + offset + 0) = (u_joint5_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint5_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint5_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint5_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint5_L);
      union {
        float real;
        uint32_t base;
      } u_joint6_L;
      u_joint6_L.real = this->joint6_L;
      *(outbuffer + offset + 0) = (u_joint6_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint6_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint6_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint6_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint6_L);
      union {
        float real;
        uint32_t base;
      } u_joint7_L;
      u_joint7_L.real = this->joint7_L;
      *(outbuffer + offset + 0) = (u_joint7_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint7_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint7_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint7_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint7_L);
      union {
        float real;
        uint32_t base;
      } u_gripper_L;
      u_gripper_L.real = this->gripper_L;
      *(outbuffer + offset + 0) = (u_gripper_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper_L.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper_L.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_L);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_slide_R;
      u_slide_R.base = 0;
      u_slide_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_slide_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_slide_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_slide_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->slide_R = u_slide_R.real;
      offset += sizeof(this->slide_R);
      union {
        float real;
        uint32_t base;
      } u_joint1_R;
      u_joint1_R.base = 0;
      u_joint1_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint1_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint1_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint1_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint1_R = u_joint1_R.real;
      offset += sizeof(this->joint1_R);
      union {
        float real;
        uint32_t base;
      } u_joint2_R;
      u_joint2_R.base = 0;
      u_joint2_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint2_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint2_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint2_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint2_R = u_joint2_R.real;
      offset += sizeof(this->joint2_R);
      union {
        float real;
        uint32_t base;
      } u_joint3_R;
      u_joint3_R.base = 0;
      u_joint3_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint3_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint3_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint3_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint3_R = u_joint3_R.real;
      offset += sizeof(this->joint3_R);
      union {
        float real;
        uint32_t base;
      } u_joint4_R;
      u_joint4_R.base = 0;
      u_joint4_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint4_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint4_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint4_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint4_R = u_joint4_R.real;
      offset += sizeof(this->joint4_R);
      union {
        float real;
        uint32_t base;
      } u_joint5_R;
      u_joint5_R.base = 0;
      u_joint5_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint5_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint5_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint5_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint5_R = u_joint5_R.real;
      offset += sizeof(this->joint5_R);
      union {
        float real;
        uint32_t base;
      } u_joint6_R;
      u_joint6_R.base = 0;
      u_joint6_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint6_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint6_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint6_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint6_R = u_joint6_R.real;
      offset += sizeof(this->joint6_R);
      union {
        float real;
        uint32_t base;
      } u_joint7_R;
      u_joint7_R.base = 0;
      u_joint7_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint7_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint7_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint7_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint7_R = u_joint7_R.real;
      offset += sizeof(this->joint7_R);
      union {
        float real;
        uint32_t base;
      } u_gripper_R;
      u_gripper_R.base = 0;
      u_gripper_R.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper_R.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper_R.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper_R.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper_R = u_gripper_R.real;
      offset += sizeof(this->gripper_R);
      union {
        float real;
        uint32_t base;
      } u_slide_L;
      u_slide_L.base = 0;
      u_slide_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_slide_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_slide_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_slide_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->slide_L = u_slide_L.real;
      offset += sizeof(this->slide_L);
      union {
        float real;
        uint32_t base;
      } u_joint1_L;
      u_joint1_L.base = 0;
      u_joint1_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint1_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint1_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint1_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint1_L = u_joint1_L.real;
      offset += sizeof(this->joint1_L);
      union {
        float real;
        uint32_t base;
      } u_joint2_L;
      u_joint2_L.base = 0;
      u_joint2_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint2_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint2_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint2_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint2_L = u_joint2_L.real;
      offset += sizeof(this->joint2_L);
      union {
        float real;
        uint32_t base;
      } u_joint3_L;
      u_joint3_L.base = 0;
      u_joint3_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint3_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint3_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint3_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint3_L = u_joint3_L.real;
      offset += sizeof(this->joint3_L);
      union {
        float real;
        uint32_t base;
      } u_joint4_L;
      u_joint4_L.base = 0;
      u_joint4_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint4_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint4_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint4_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint4_L = u_joint4_L.real;
      offset += sizeof(this->joint4_L);
      union {
        float real;
        uint32_t base;
      } u_joint5_L;
      u_joint5_L.base = 0;
      u_joint5_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint5_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint5_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint5_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint5_L = u_joint5_L.real;
      offset += sizeof(this->joint5_L);
      union {
        float real;
        uint32_t base;
      } u_joint6_L;
      u_joint6_L.base = 0;
      u_joint6_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint6_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint6_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint6_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint6_L = u_joint6_L.real;
      offset += sizeof(this->joint6_L);
      union {
        float real;
        uint32_t base;
      } u_joint7_L;
      u_joint7_L.base = 0;
      u_joint7_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint7_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint7_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint7_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint7_L = u_joint7_L.real;
      offset += sizeof(this->joint7_L);
      union {
        float real;
        uint32_t base;
      } u_gripper_L;
      u_gripper_L.base = 0;
      u_gripper_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper_L = u_gripper_L.real;
      offset += sizeof(this->gripper_L);
     return offset;
    }

    const char * getType(){ return ROBOT; };
    const char * getMD5(){ return "bb58b233829e057866371dd14661701c"; };

  };

  class robotResponse : public ros::Msg
  {
    public:
      typedef const char* _state_type;
      _state_type state;

    robotResponse():
      state("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
     return offset;
    }

    const char * getType(){ return ROBOT; };
    const char * getMD5(){ return "af6d3a99f0fbeb66d3248fa4b3e675fb"; };

  };

  class robot {
    public:
    typedef robotRequest Request;
    typedef robotResponse Response;
  };

}
#endif
