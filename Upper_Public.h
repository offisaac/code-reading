/*! @file Upper_Public.h
 * 
 *  @brief public function
 *        顶层公用函数和数据结构
 */
#ifndef _UPPER_PUBLIC_H_
#define _UPPER_PUBLIC_H_

enum legside_Enumdef
{
    RIGHT_JOINT,
    LEFT_JOINT
};


/*lqr线性拟合参数*/
typedef struct _Fit_Params
{
    float a = 0;//三次项
    float b = 0;//二次项
    float c = 0;//一次项
    float d = 0;//常数项
}Fit_Params;

namespace upper
{
    const float degree2rad_ratio = 3.1415926f/180.f;
    const float rad2degree_ratio = 180.f/3.1415926f;

    template<typename F, typename F1>
    F constrain(F input, F1 threshold)
    {
        if (threshold < 0)
            threshold = -threshold;

        if (input <= -threshold)
            return -threshold;
        else if (input >= threshold)
            return threshold;

        return input;
    }

    template<typename F, typename F1>
    F constrain(F input, F1 threshold_1, F1 threshold_2)
    {
        F min, max;
        if (threshold_1 > threshold_2)
        {
            max = threshold_1;
            min = threshold_2;
        }
        else
        {
            min = threshold_1;
            max = threshold_2;
        }

        if (input <= min)
            return min;
        else if (input >= max)
            return max;
        return input;
    }
}



#endif
