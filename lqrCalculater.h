/*! @file lqrCalculater.h
 *
 *  @brief LQR calculation
 *        矩阵计算lqr控制器
 */
#ifndef _LQRCALCULATER_H_
#define _LQRCALCULATER_H_

#include <stdint.h>
#include "PIDmethod.h"
using namespace std;


template<int StateVariableNum, int outNum>
class lqrCalculater
{
public:
	void init(Fit_Params _MatK_fit[outNum][StateVariableNum]);
	void updateData(float _target[StateVariableNum], float _current[StateVariableNum], float legLength);
	void adjust();

	float total_out[outNum] = {};
	float single_out[outNum][StateVariableNum] = {};
private:
	Fit_Params MatK_fit[outNum][StateVariableNum];

	float K[outNum][StateVariableNum] = {};
	float error[StateVariableNum] = {};

	void fitMatK(float h);
};


namespace lqrID
{
	enum
	{
		distance = 0,
		speed = 1,
		alpha = 2,
		dalpha = 3,
		beta = 4,
		dbeta = 5
	};
}


template<int StateVariableNum, int outNum>
void lqrCalculater<StateVariableNum, outNum>::init(Fit_Params _MatK_fit[outNum][StateVariableNum])
{
	for (int i = 0; i < outNum; ++i) {
		for (int j = 0; j < StateVariableNum; ++j) {
			MatK_fit[i][j] = _MatK_fit[i][j];
		}
	}
}

template<int StateVariableNum, int outNum>
void lqrCalculater<StateVariableNum, outNum>::updateData(float _target[StateVariableNum], float _current[StateVariableNum], float legLength)
{
	fitMatK(legLength);
	for (int i = 0; i < StateVariableNum; i++)
	{
		error[i] = _target[i] - _current[i];
	}
}

template<int StateVariableNum, int outNum>
void lqrCalculater<StateVariableNum, outNum>::fitMatK(float h)
{
	for (int i = 0; i < StateVariableNum; i++)
	{
		for (int j = 0; j < outNum; j++)
		{
			K[j][i] = MatK_fit[j][i].a * powf(h, 3) + MatK_fit[j][i].b * powf(h, 2) + MatK_fit[j][i].c * h + MatK_fit[j][i].d;
		}
	}
}

template<int StateVariableNum, int outNum>
void lqrCalculater<StateVariableNum, outNum>::adjust()
{
	total_out[0] = 0;
	total_out[1] = 0;
	for (int i = 0; i < StateVariableNum; i++)
	{
		total_out[0] += K[0][i] * error[i];
		total_out[1] += K[1][i] * error[i];
		single_out[0][i] = K[0][i] * error[i];
		single_out[1][i] = K[1][i] * error[i];
	}
}

#endif

