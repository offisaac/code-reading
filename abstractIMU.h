/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractIMU.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象IMU库，用于对接顶层认为定义坐标系，与imu的实际坐标系之间的数据转化
 *        方便将仿真用的理想顶层，对接到实车
 * @version 1.0
 * @date 2023-03-04
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef ABSTRACTIMU_H
#define ABSTRACTIMU_H

#include "Drivers/Devices/Lpms_Be2/LPMS_BE2.h"

#ifdef __cplusplus
/*线性参数*/
struct LinearDataStructdef
{
    float x;
    float y;
    float z;
};
/*角度参数*/
struct AngularDataStructdef
{
    float pitch;
    float yaw;
    float roll;
};

namespace abstractIMU
{
    /**
     * @brief 线性加速度极性结构体
     *
     */
    struct linerPolarityStruct
    {
        int8_t x = 1;
        int8_t y = 1;
        int8_t z = 1;
    };

    /**
     * @brief 角度极性结构体
     *
     */
    struct anglePolarityStruct
    {
        int8_t pitch = 1;
        int8_t roll = 1;
        int8_t yaw = 1;
    };
    /**
     * @brief 抽象加速度坐标结构体
     *
     */
    struct abstractAccCoordinateStruct
    {
        uint8_t x = 0;
        uint8_t y = 1;
        uint8_t z = 2;
    };
    /**
     * @brief 抽象欧拉角坐标结构体
     *
     */
    struct abstractEulerCoordinateStruct
    {
        uint8_t Pitch = 0;
        uint8_t Yaw = 1;
        uint8_t Roll = 2;
    };
    /**
     * @brief IMU欧拉角世界（现实）坐标系枚举
     *
     */
    enum imuEulerCoordinate
    {
        imuWorldPitch = 0,
        imuWorldYaw = 1,
        imuWorldRoll = 2
    };
    /**
     * @brief IMU直角坐标系世界（现实）坐标系枚举
     *
     */
    enum imuAccCoordinate
    {
        imuWorldAccX = 0,
        imuWorldAccY = 1,
        imuWorldAccZ = 2
    };
    /**
     * @brief IMU抽象类基类，完成了绝大部分的逻辑操作
     * 在继承类中，加入imu类的指针成员，绑定imu变量，定义9个获取imu数据的纯虚函数，即可正常使用
     */
    class abstractIMUBaseClassdef
    {
    private:
        // 抽象坐标系结构体，用于存储人定坐标系与imu坐标系的对应关系
        abstractAccCoordinateStruct abstractAccData;
        abstractEulerCoordinateStruct abstractEularData;

        // 获取imu在世界（现实）坐标系下的加速度
        inline virtual float getACCX() = 0;
        inline virtual float getACCY() = 0;
        inline virtual float getACCZ() = 0;

        // 获取imu在世界（现实）坐标系下的欧拉角、及对应的角速度
        inline virtual float getEularPitch() = 0;
        inline virtual float getEularYaw() = 0;
        inline virtual float getEularRoll() = 0;

        inline virtual float getGyroPitch() = 0;
        inline virtual float getGyroYaw() = 0;
        inline virtual float getGyroRoll() = 0;

        inline virtual bool isInit() = 0; // 检测是否导入了imu指针，防止未导入时，因为指针为空指针，调用函数而进入硬件中断
    public:
        LinearDataStructdef accData;
        AngularDataStructdef eularData, gyroData;

        // 加速度极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        linerPolarityStruct accPolarity;
        // 欧拉角极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        anglePolarityStruct eularPolarity;
        // 欧拉角基准值，用于校准，会在校准极性后，加上此基准值
        AngularDataStructdef eularBaseData = {0, 0, 0};
        float angle_unit_convert = 1;

        // 绑定加速度坐标系，传入三个枚举值，分别代表x,y,z轴对应的imu坐标系
        // 若传入imuWorldAccY,imuWorldAccX,imuWorldAccZ
        // 则代表将imu的y轴对应到人为定义的x轴，imu的x轴对应到人为定义的y轴，imu的z轴对应到人为定义的z轴
        inline void bindEulerCoordinate(imuEulerCoordinate abstractPitch, imuEulerCoordinate abstractYaw, imuEulerCoordinate abstractRoll)
        {
            abstractEularData.Pitch = abstractPitch;
            abstractEularData.Yaw = abstractYaw;
            abstractEularData.Roll = abstractRoll;
        }

        // 绑定加速度坐标系，传入三个枚举值，分别代表pitch, yaw, roll轴对应的imu坐标系
        // 若传入imuWorldPitch,imuWorldRoll,imuWorldYaw
        // 则代表将imu的pitch轴对应到人为定义的pitch轴，imu的Roll轴对应到人为定义的yaw轴，imu的Yaw轴对应到人为定义的roll轴
        inline void bindAccCoordinate(imuAccCoordinate abstractX, imuAccCoordinate abstractY, imuAccCoordinate abstractZ)
        {
            abstractAccData.x = abstractX;
            abstractAccData.y = abstractY;
            abstractAccData.z = abstractZ;
        }

        // 获取imu世界坐标系数据，转化为人为定义的坐标系、对齐极性后的数据
        // 功能基本实现，继承类不用再写
        void update()
        {
            float tempAccData[3];
            tempAccData[imuWorldAccX] = this->getACCX();
            tempAccData[imuWorldAccY] = this->getACCY();
            tempAccData[imuWorldAccZ] = this->getACCZ();

            accData.x = tempAccData[abstractAccData.x] * accPolarity.x * angle_unit_convert;
            accData.y = tempAccData[abstractAccData.y] * accPolarity.y * angle_unit_convert;
            accData.z = tempAccData[abstractAccData.z] * accPolarity.z * angle_unit_convert;

            float tempGyroData[3];
            tempGyroData[imuWorldPitch] = this->getGyroPitch();
            tempGyroData[imuWorldYaw] = this->getGyroYaw();
            tempGyroData[imuWorldRoll] = this->getGyroRoll();

            gyroData.pitch = tempGyroData[abstractEularData.Pitch] * angle_unit_convert * eularPolarity.pitch;
            gyroData.yaw = tempGyroData[abstractEularData.Yaw] * angle_unit_convert * eularPolarity.yaw;
            gyroData.roll = tempGyroData[abstractEularData.Roll] * angle_unit_convert * eularPolarity.roll;

            float tempEularData[3];
            tempEularData[imuWorldPitch] = this->getEularPitch();
            tempEularData[imuWorldYaw] = this->getEularYaw();
            tempEularData[imuWorldRoll] = this->getEularRoll();

            eularData.pitch = tempEularData[abstractEularData.Pitch] * eularPolarity.pitch * angle_unit_convert + eularBaseData.pitch;
            eularData.yaw = tempEularData[abstractEularData.Yaw] * eularPolarity.yaw * angle_unit_convert + eularBaseData.yaw;
            eularData.roll = tempEularData[abstractEularData.Roll] * eularPolarity.roll * angle_unit_convert + eularBaseData.roll;
        }
    };
};
/**
 * @brief IMU抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 模板主类不写实现，是为了避免在传入没有特化过的IMU类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class IMUtype>
class abstractIMUClassdef
{
};

#if USE_SRML_LPMS_BE2
/**
 * @brief 特化模板类，用于阿路比imu
 */
template <>
class abstractIMUClassdef<LPMS_BE2_Typedef> : public abstractIMU::abstractIMUBaseClassdef
{
private:
    inline virtual float getACCX()
    {
        return imu->get_data().linearAccX;
    }
    inline virtual float getACCY()
    {
        return imu->get_data().linearAccY;
    }
    inline virtual float getACCZ()
    {
        return imu->get_data().linearAccZ;
    }

    inline virtual float getEularPitch()
    {
        return imu->get_data().Euler_Pitch;
    }
    inline virtual float getEularYaw()
    {
        return imu->get_data().Euler_Yaw;
    }
    inline virtual float getEularRoll()
    {
        return imu->get_data().Euler_Roll;
    }

    /* 对于阿路比imu */
    /* pitch绑定caliGyroY */
    /* yaw绑定caliGyroZ */
    /* roll绑定caliGyroX */
    inline virtual float getGyroPitch()
    {
        return imu->get_data().caliGyroY;
    }
    inline virtual float getGyroYaw()
    {
        return imu->get_data().caliGyroZ;
    }
    inline virtual float getGyroRoll()
    {
        return imu->get_data().caliGyroX;
    }

    inline virtual bool isInit()
    {
        if (imu == nullptr)
            return 0;
        else
            return 1;
    }

public:
    LPMS_BE2_Typedef *imu = nullptr;
    inline void bindIMU(LPMS_BE2_Typedef *_lpms)
    {
        this->imu = _lpms;
    }
    inline void processRecData(uint8_t *data)
    {
        if (isInit() == 0)
            return;
        imu->LPMS_BE2_Get_Data(data);
        imu->LPMS_BE2_Data_Convert();
    }
};
#endif  /* USE_SRML_LPMS_BE2 */

#if USE_SRML_MPU6050
/**
 * @brief 特化模板类，用于MPU6050
 */
template <>
class abstractIMUClassdef<mpu_rec_s> : public abstractIMU::abstractIMUBaseClassdef
{
private:
    mpu_rec_s *mpu_s = nullptr;

    inline virtual float getACCX()
    {
        return mpu_s->accel[0];
    }
    inline virtual float getACCY()
    {
        return mpu_s->accel[1];
    }
    inline virtual float getACCZ()
    {
        return mpu_s->accel[2];
    }

    inline virtual float getEularPitch()
    {
        return mpu_s->pitch;
    }
    inline virtual float getEularYaw()
    {
        return mpu_s->yaw;
    }
    inline virtual float getEularRoll()
    {
        return mpu_s->roll;
    }

    inline virtual float getGyroPitch()
    {
        return mpu_s->gyro[1];
    }
    inline virtual float getGyroYaw()
    {
        return mpu_s->gyro[0];
    }
    inline virtual float getGyroRoll()
    {
        return mpu_s->gyro[2];
    }

    inline virtual bool isInit()
    {
        if (mpu_s == nullptr)
            return 0;
        else
            return 1;
    }

public:
    inline void bindIMU(mpu_rec_s *_mpu_s)
    {
        this->mpu_s = _mpu_s;
    }
    /* 注意，该update需要在任务里定期执行，频率与MPU6050的读取相关 */
    inline void update()
    {
        if (isInit() == 0)
            return;
        dmp_read_data(mpu_s);
        abstractIMUBaseClassdef::update();
    }
};
#endif /* USE_SRML_MPU6050 */

#endif /* __cplusplus */

#endif /* ABSTRACTIMU_H */
