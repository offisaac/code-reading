/*! @file user_data.h
 *  @brief Containers for user paraments
 *        用户参数数据类
 */
#ifndef _USER_DATA_H_
#define _USER_DATA_H_

#include "PIDmethod.h"
#include "boardComProtocol.h"

const float wheelRadius = 0.19f / 2.f; // 轮半径
const float trackWidth = 0.52f;        // 轮距
const float balance_point = 0.f;

#ifdef __cplusplus
extern "C"
{
#endif
  class UserData_Classdef
  {
  public:
    UserData_Classdef() { Init(); }
    /**************************轮控制参数**************************/
    /*轮子转向环*/
    float lqr_yaw_kp;
    /*轮子距离环*/
    float distance_max;
    /*重心自适应*/
    float setPoint_kp;
    float setPoint_ki;
    float setPoint_I_Max;
    float setPoint_O_Max;
    /*普平模式管理*/
    float wheelBalance_kp;
    float wheelBalance_O_Max;
    /*轮子直立前馈*/
    float stand_ff;
    /**************************腿控制参数*********************************/
    /*车体roll轴平衡*/
    float roll_keep_kp;
    float roll_keep_ki;
    float roll_keep_imax;
    float roll_keep_kd;
    float roll_keep_omax;
    /*双轮差速内倾*/
    float turn_adaption_kp;
    float turn_adaption_omax;
    float turn_adaption_kp2;
    float turn_adaption_kp3;
    /*轮腿高度自适应*/
    float length_keep_kp;
    float length_keep_ki;
    float length_keep_imax;
    float length_keep_kd;
    float length_ff_gain; // 前馈增益系数
    /*轮腿防劈叉*/
    float j_turn_kp;
    float j_follow_kp;
    float j_follow_kd;
    /*lqr增益参数*/
    Fit_Params leg_lqr_params[2][6];

    /*上坡水平分力前馈*/
    float slope_angle;
    float slope_ff_gain;

    void Init()
    {
      /**************************轮控制参数**************************/
      /*轮子转向环*/
      lqr_yaw_kp = 2;
      // turn_step = 1.f;
      /*重心自适应*/
      setPoint_kp = 0.25;
      setPoint_ki = 0.;
      setPoint_I_Max = 0.12f;
      setPoint_O_Max = 0.12f;
      /*普平模式管理*/
      wheelBalance_kp = 0.2f;
      wheelBalance_O_Max = 0.2f;
      /*轮子直立环前馈*/
      stand_ff = 0.0f;
      /**************************腿控制参数**************************/
      /*车体转向内倾*/
      // turn_adaption_kp = 0.1;
      // turn_adaption_omax = 0.08;
      turn_adaption_kp = 0.03;
      turn_adaption_omax = 0.08;
      turn_adaption_kp2 = 20.;
      turn_adaption_kp3 = 500.;
      /*车体roll轴自平衡*/
      roll_keep_kp = -1.2;
      roll_keep_ki = 0.;
      roll_keep_imax = 0.0;
      roll_keep_kd = 0.03;
      roll_keep_omax = 0.25;
      /*轮腿防劈叉*/
      // j_turn_kp = 8.;
      // j_follow_kp = 300;
      // j_follow_kd = 6.;
      j_turn_kp = 4/18.*35.;
      j_follow_kp = 150/18.*35.;
      j_follow_kd = 3./18.*35.;
      /*高度自适应*/
      length_keep_kp = 400.;
      length_keep_ki = 1000.;
      length_keep_imax = 50;
      length_keep_kd = 30.;
      length_ff_gain = 0.4f;

      /*上坡补偿*/
      slope_angle = 20.f / ratio_rad_to_degree;
      // slope_ff_gain = 1.5f;
      slope_ff_gain = 1.f;

      /**************************LQR控制参数*********************************/
      /*轮子距离环*/
      // leg_lqr_params[0][0].a = 7.624548e-01;
      // leg_lqr_params[0][0].b = -8.531826e-02;
      // leg_lqr_params[0][0].c = -3.719468e-01;
      // leg_lqr_params[0][0].d = -2.662243e-01;
      leg_lqr_params[0][0].a = 0;
      leg_lqr_params[0][0].b = 0;
      leg_lqr_params[0][0].c = 0;
      leg_lqr_params[0][0].d = -4.;

      /*轮子速度环*/
      leg_lqr_params[0][1].a = 4.472673e+00;
      leg_lqr_params[0][1].b = -4.552566e-01;
      leg_lqr_params[0][1].c = -2.274744e+00;
      leg_lqr_params[0][1].d = -1.667055e+00;

      /*轮子直立环*/
      // alpha
      leg_lqr_params[0][2].a = -2.815917e+01;
      leg_lqr_params[0][2].b = 5.407807e+01;
      leg_lqr_params[0][2].c = -5.050877e+01;
      leg_lqr_params[0][2].d = -1.140111e+00;

      // dalpha
      leg_lqr_params[0][3].a = 6.157640e+00;
      leg_lqr_params[0][3].b = -5.841816e+00;
      leg_lqr_params[0][3].c = -3.547612e+00;
      leg_lqr_params[0][3].d = -2.554585e-01;

      // beta
      leg_lqr_params[0][4].a = -2.406978e+01;
      leg_lqr_params[0][4].b = -1.070885e+01;
      leg_lqr_params[0][4].c = 3.683818e+01;
      leg_lqr_params[0][4].d = -2.281059e+01;

      // dbeta
      leg_lqr_params[0][5].a = 2.799039e+00;
      leg_lqr_params[0][5].b = -5.591316e+00;
      leg_lqr_params[0][5].c = 5.127052e+00;
      leg_lqr_params[0][5].d = -3.300334e+00;

      /*轮腿路程环*/
      // leg_lqr_params[1][0].a = 5.376054e+00;
      // leg_lqr_params[1][0].b = -2.908572e+00;
      // leg_lqr_params[1][0].c = -1.020320e+00;
      // leg_lqr_params[1][0].d = 1.012920e+00;
      leg_lqr_params[1][0].a = 0;
      leg_lqr_params[1][0].b = 0;
      leg_lqr_params[1][0].c = 0;
      leg_lqr_params[1][0].d = 0;

      /*轮腿速度环*/
      leg_lqr_params[1][1].a = 5.470545e+01;
      leg_lqr_params[1][1].b = -2.910852e+01;
      leg_lqr_params[1][1].c = -1.107726e+01;
      leg_lqr_params[1][1].d = 1.070159e+01;

      /*轮腿直立环*/
      // alpha
      leg_lqr_params[1][2].a = 5.341424e+02;
      leg_lqr_params[1][2].b = -5.677078e+02;
      leg_lqr_params[1][2].c = 2.091032e+02;
      leg_lqr_params[1][2].d = 5.458692e+00;

      // dalpha
      leg_lqr_params[1][3].a = 3.064386e+01;
      leg_lqr_params[1][3].b = -3.852745e+01;
      leg_lqr_params[1][3].c = 1.679569e+01;
      leg_lqr_params[1][3].d = 1.191438e+00;

      // beta
      leg_lqr_params[1][4].a = -1.563579e+02;
      leg_lqr_params[1][4].b = 3.740500e+02;
      leg_lqr_params[1][4].c = -2.721588e+02;
      leg_lqr_params[1][4].d = -4.293818e+01;

      // dbeta
      leg_lqr_params[1][5].a = -9.372464e+00;
      leg_lqr_params[1][5].b = 4.017459e+01;
      leg_lqr_params[1][5].c = -3.561566e+01;
      leg_lqr_params[1][5].d = -3.792442e-01;
    }
  };
#ifdef __cplusplus
};
#endif

#endif
