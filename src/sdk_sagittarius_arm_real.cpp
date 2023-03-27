/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>
 */
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_log.h"
namespace sdk_sagittarius_arm
{
    /// @brief SagittariusArmReal 初始化类
    /// @param strSerialName - std::string
    /// 对应机械臂的串口名，默认是“/dev/sagittarius”
    /// @param Baudrate - int 波特率，默认是1000000
    /// @param vel - int 舵机的最高运行速度，对应7个舵机的速度0～4096，
    /// 建议设置在1000左右
    /// @param acc - int 舵机的加速度， 对应舵机的加速度0～254。建议设置在10以内
    SagittariusArmReal::SagittariusArmReal(std::string strSerialName,
                                           int baudrate, int vel = 500,
                                           int acc = 5)
    {
        int result;
        int iTimeLimit = 1;
        pSDKarm = NULL;
        float tmp_lower_joint_limits[JOINT_NUM] = JOINT_LOWER_LIMITS;
        float tmp_upper_joint_limits[JOINT_NUM] = JOINT_UPPER_LIMITS;
        memset(lower_joint_limits, 0, JOINT_NUM * sizeof(float));
        memset(upper_joint_limits, 0, JOINT_NUM * sizeof(float));
        memcpy(lower_joint_limits, tmp_lower_joint_limits,
               JOINT_NUM * sizeof(float));
        memcpy(upper_joint_limits, tmp_upper_joint_limits,
               JOINT_NUM * sizeof(float));

        pSDKarm = new sdk_sagittarius_arm::CSDarmCommonSerial(
            strSerialName, baudrate, iTimeLimit);
        result = pSDKarm->Init();
        pSDKarm->StartReceiveSerail();
        SetServoVelocity(vel);
        sleep(1);
        SetServoAcceleration(acc);
        sleep(1);
        // SetServoTorque(1000);
        ControlTorque("lock");
        sleep(1);
    }

    SagittariusArmReal::~SagittariusArmReal()
    {
        log_print(LOG_TYPE_DEBUG, "free the SagittariusArmReal\n");
        if (pSDKarm != NULL)
        {
            delete pSDKarm;
        }
    }
    /// @brief CheckUpperLower 检测输入的角度是否在允许的范围内
    /// @param js「」 -
    /// js[0]～js[5]对应1～6号舵机的弧度值。请一次性按顺序传入6个舵机值
    bool SagittariusArmReal::CheckUpperLower(float js[])
    {
        int i;
        for (i = 0; i < JOINT_NUM; i++)
        {
            if ((js[i] < lower_joint_limits[i]) ||
                (js[i] > upper_joint_limits[i]))
            {
                log_print(LOG_TYPE_ERROR, "joint[%d] is not in the range\n", i);
                return false;
            }
        }
        return true;
    }
    /// @brief CheckUpperLowerWithIndex 检测输入的角度是否在允许的范围内
    /// @param sv「」 -
    /// sv[].id为舵机号，sv[].value为对应舵机的弧度值。对应的舵机的数组可以不按顺序排列，但最多六个舵机。
    /// @param num - num为要控制的舵机的个数。最多六个舵机。
    bool SagittariusArmReal::CheckUpperLowerWithIndex(ServoStruct sv[], int num)
    {
        int i;
        for (i = 0; i < num; i++)
        {
            if ((sv[i].value < lower_joint_limits[sv[i].id]) ||
                (sv[i].value > upper_joint_limits[sv[i].id]))
            {
                log_print(LOG_TYPE_ERROR, "joint[%d] is not in the range\n",
                          sv[i].id);
                return false;
            }
        }
        return true;
    }
    /// @brief SetServoVelocity 设置所有舵机的最高速度
    /// @param arm_vel - 对就舵机的速度0～4096， 建议设置在1000左右
    bool SagittariusArmReal::SetServoVelocity(int arm_vel)
    {
        log_print(LOG_TYPE_INFO, "arm_vel is %d\n", arm_vel);
        if (pSDKarm != NULL) pSDKarm->SetArmVel(arm_vel);
        return true;
    }
    /// @brief SetServoAcceleration 设置所有舵机的加速度
    /// @param arm_acc - 对就舵机的加速度0～254。建议设置在10以内
    bool SagittariusArmReal::SetServoAcceleration(int arm_acc)
    {
        log_print(LOG_TYPE_INFO, "arm_acceleration is %d\n", arm_acc);
        if (pSDKarm != NULL) pSDKarm->SetArmAcc(arm_acc);
        return true;
    }
    /// @brief SetServoTorque 设置所有舵机的扭距
    /// @param arm_torque「」 -
    /// arm_torque「」按顺序存放每个舵机扭矩的大小。一次性按顺序存放7个舵机
    bool SagittariusArmReal::SetServoTorque(int arm_torque[])
    {
        if (pSDKarm != NULL) pSDKarm->SetArmTorque(arm_torque);
        return true;
    }
    /// @brief GetCurrentJointStatus 获取当前各个秀
    /// @param js「」 -
    /// js「」按顺序存放每个舵机当前弧度的大小。一次性按顺序获取7个舵机
    bool SagittariusArmReal::GetCurrentJointStatus(float js[])
    {
        memcpy(js, pSDKarm->joint_status, sizeof(pSDKarm->joint_status));
        return true;
    }
    /// @brief ControlTorque 设置舵机锁舵或者释放
    /// @param msg - 字符串，"free" 或者 "lock"
    void SagittariusArmReal::ControlTorque(const std::string msg)
    {
        if (msg == "free")
        {
            torque_status = false;
            pSDKarm->SendArmLockOrFree(0);
        }
        else
        {
            torque_status = true;
            pSDKarm->SendArmLockOrFree(1);
        }
    }
    /// @brief SetAllServoRadian 控制舵机运行到相应的弧度
    /// @param joint_positions「」 -
    /// joint_positions「0」~joint_positions「5」分别对应舵机1～6的弧定，一次性按顺序输入弧度
    void SagittariusArmReal::SetAllServoRadian(float joint_positions[])
    {
        if (CheckUpperLower(joint_positions) == false) return;
        if (pSDKarm != NULL)
        {
            angle[0] = joint_positions[0];
            angle[1] = joint_positions[1];
            angle[2] = joint_positions[2];
            angle[3] = joint_positions[3];
            angle[4] = joint_positions[4];
            angle[5] = joint_positions[5];
            if (torque_status)
            {
                pSDKarm->SendArmAllServerCB(angle[0], angle[1], angle[2],
                                            angle[3], angle[4],
                                            angle[5]); //插补
                // log_print(LOG_TYPE_INFO, "[%f,%f,%f,%f,%f,%f]\n",
                // angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }
        }
    }
    /// @brief SetServoRadianWithIndex 设定指定舵机的弧度。
    /// @param sv「」 -
    /// sv[].id为舵机号，sv[].value为对应舵机的弧度值。对应的舵机的数组可以不按顺序排列，但最多六个舵机。
    /// @param num - num为要控制的舵机的个数。最多六个舵机。
    void SagittariusArmReal::SetServoRadianWithIndex(ServoStruct sv[], int num)
    {
        if (CheckUpperLowerWithIndex(sv, num) == false) return;
        if (pSDKarm != NULL)
        {
            if (torque_status)
            {
                pSDKarm->SendArmAllServerWithIndex(sv, num);
            }
        }
    }

    /// @brief arm_calculate_gripper_degree_position 距离转换成角度
    /// @param dist - 夹爪的距离值
    short
    SagittariusArmReal::arm_calculate_gripper_degree_position(const float dist)
    {
        double half_dist = dist / 2.0;
        short result = -(3462 * half_dist) * 10;
        return result;
    }
    /// @brief arm_set_gripper_linear_position 设置夹爪的位置
    /// @param dist - 夹爪的距离值 -0.068~0.0(其中-0.068表示全关闭，0.0表示全开)
    void SagittariusArmReal::arm_set_gripper_linear_position(const float dist)
    {
        short g_degree = arm_calculate_gripper_degree_position(dist);
        // log_print(LOG_TYPE_INFO, "degree is %d\n",g_degree);
        if (torque_status)
        {
            pSDKarm->SendArmEndAction(0, g_degree);
        }
    }

    /// @brief SagittariusArmKinematics 初始化类
    /// @param x - float 末端在 x 轴的偏移, 单位为米
    /// @param y - float 末端在 y 轴的偏移, 单位为米
    /// @param z - float 末端在 z 轴的偏移, 单位为米
    SagittariusArmKinematics::SagittariusArmKinematics(float x, float y,
                                                       float z)
    {
        // NO.     w(x,    y,      z)     q(x,        y,           z)
        // 1       (0,     0,      -1)     (0.045,     0,          0.07)
        // 2       (0,     -1,     0)      (0.045,     -0.0001,    0.125)
        // 3       (0,     -1,     0)      (0.078,     -0.0002,    0.304)
        // 4       (-1,    0,      0)      (0.1625,    -0.0002,    0.304)
        // 5       (0,     -1,     0)      (0.24685,   0.00045,    0.304)
        // 6       (-1,    0,      0)      (0.308237,  0.00045,    0.304)
        Eigen::Vector3d w1(0, 0, -1), w2(0, -1, 0), w3(0, -1, 0), w4(-1, 0, 0),
            w5(0, -1, 0), w6(-1, 0, 0); //角速度方向
        Eigen::Vector3d q1(0.045, 0, 0.07), q2(0.045, -0.0001, 0.125),
            q3(0.078, -0.0002, 0.304), q4(0.1625, -0.0002, 0.304),
            q5(0.24685, 0.00045, 0.304),
            q6(0.308237, 0.00045, 0.304); //螺旋轴位置
        Eigen::VectorXd s1(6, 1), s2(6, 1), s3(6, 1), s4(6, 1), s5(6, 1),
            s6(6, 1); //螺旋轴序列
        s1 << w1, -w1.cross(q1);
        s2 << w2, -w2.cross(q2);
        s3 << w3, -w3.cross(q3);
        s4 << w4, -w4.cross(q4);
        s5 << w5, -w5.cross(q5);
        s6 << w6, -w6.cross(q6);
        Slist.resize(6, 6);
        Slist << s1, s2, s3, s4, s5, s6;

        M << 1, 0, 0, 0.363937 + x, 0, 1, 0, 0.00045 + y, 0, 0, 1, 0.304 + z, 0,
            0, 0, 1;

        float tmp_lower_joint_limits[JOINT_NUM] = JOINT_LOWER_LIMITS;
        float tmp_upper_joint_limits[JOINT_NUM] = JOINT_UPPER_LIMITS;
        memset(lower_joint_limits, 0, JOINT_NUM * sizeof(float));
        memset(upper_joint_limits, 0, JOINT_NUM * sizeof(float));
        memcpy(lower_joint_limits, tmp_lower_joint_limits,
               JOINT_NUM * sizeof(float));
        memcpy(upper_joint_limits, tmp_upper_joint_limits,
               JOINT_NUM * sizeof(float));
    }

    SagittariusArmKinematics::~SagittariusArmKinematics()
    {
        log_print(LOG_TYPE_DEBUG, "free the SagittariusArmKinematics\n");
    }

    /// @brief getIKinThetaMatrix 给定末端位姿的矩阵，运算出机械臂各个关节的角度
    /// @param M - 末端在世界坐标上表达的矩阵
    /// @param theta_result「」- 运算出关节的解会存储在这个数组中，单位是弧度制
    /// @param eomg - 姿态的精度
    /// @param ev - 位置的精度
    bool
    SagittariusArmKinematics::getIKinThetaMatrix(const Eigen::MatrixXd& M_EE,
                                                 float theta_result[],
                                                 double eomg, double ev)
    {
        Eigen::VectorXd theta(JOINT_NUM);
        theta << 0, 0, 0, 0, 0, 0;
        // theta << theta_result[0], theta_result[1], theta_result[2],
        // theta_result[3], theta_result[4], theta_result[5];
        bool iRet = mr::IKinSpace(Slist, M, M_EE.matrix(), theta, eomg, ev);
        if (!iRet)
        {
            log_print(LOG_TYPE_WARN, "posture not found\r\n");
            return iRet;
        }
        for (int i = 0; i < JOINT_NUM; i++)
        {
            if (theta(i) <= -M_PI * 2)
                theta(i) = fmod(theta(i), (-M_PI * 2));
            else if (theta(i) >= M_PI * 2)
                theta(i) = fmod(theta(i), (M_PI * 2));
            if (theta(i) < lower_joint_limits[i])
                theta(i) += M_PI * 2;
            else if (theta(i) > upper_joint_limits[i])
                theta(i) -= M_PI * 2;

            theta_result[i] = theta[i];
        }
        log_print(LOG_TYPE_DEBUG,
                  "joint theta: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
                  theta_result[1], theta_result[2], theta_result[3],
                  theta_result[4], theta_result[5], theta_result[6]);

        return iRet;
    }

    /// @brief getIKinThetaEuler
    /// 给定末端位置和以欧拉角(静态RPY)表达的姿态，运算出机械臂各个关节的角度
    /// @param x - 末端在 x 轴上的值，单位是米
    /// @param y - 末端在 y 轴上的值，单位是米
    /// @param z - 末端在 z 轴上的值，单位是米
    /// @param roll - 末端欧拉角 roll, 单位是角度制
    /// @param pitch - 末端欧拉角 pitch, 单位是角度制
    /// @param yaw - 末端欧拉角 yaw, 单位是角度制
    /// @param theta_result「」- 运算出关节的解会存储在这个数组中，单位是弧度制
    /// @param eomg - 姿态的精度
    /// @param ev - 位置的精度
    bool SagittariusArmKinematics::getIKinThetaEuler(float x, float y, float z,
                                                     float roll, float pitch,
                                                     float yaw,
                                                     float theta_result[],
                                                     double eomg, double ev)
    {
        // 欧拉角转旋转矩阵
        Eigen::AngleAxisd yawAxis =
            Eigen::AngleAxisd(yaw * M_PI / 180, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchAxis =
            Eigen::AngleAxisd(pitch * M_PI / 180, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rollAxis =
            Eigen::AngleAxisd(roll * M_PI / 180, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R =
            yawAxis.matrix() * pitchAxis.matrix() * rollAxis.matrix();

        Eigen::Isometry3d T =
            Eigen::Isometry3d::Identity(); // 变换矩阵, 实质为4*4的矩阵
        T.rotate(R);                       // 输入旋转矩阵
        T.pretranslate(Eigen::Vector3d(x, y, z)); // 平移向量

        return getIKinThetaMatrix(T.matrix(), theta_result, eomg, ev);
    }

    /// @brief getIKinThetaQuaternion
    /// 给定末端位置和以四元数表达的姿态，运算出机械臂各个关节的角度
    /// @param x - 末端在 x 轴上的值，单位是米
    /// @param y - 末端在 y 轴上的值，单位是米
    /// @param z - 末端在 z 轴上的值，单位是米
    /// @param ox - 末端四元数的 x 值
    /// @param oy - 末端四元数的 y 值
    /// @param oz - 末端四元数的 z 值
    /// @param ow - 末端四元数的 w 值
    /// @param theta_result「」- 运算出关节的解会存储在这个数组中，单位是弧度制
    /// @param eomg - 姿态的精度，单位是米
    /// @param ev - 位置的精度，单位是米
    bool SagittariusArmKinematics::getIKinThetaQuaternion(
        float x, float y, float z, float ox, float oy, float oz, float ow,
        float theta_result[], double eomg, double ev)
    {
        // 四元数转旋转矩阵
        Eigen::Quaterniond quaternion(ow, ox, oy, oz);
        Eigen::Matrix3d R = quaternion.matrix();

        Eigen::Isometry3d T =
            Eigen::Isometry3d::Identity(); // 变换矩阵, 实质为4*4的矩阵
        T.rotate(R);                       // 输入旋转矩阵
        T.pretranslate(Eigen::Vector3d(x, y, z)); // 平移向量

        return getIKinThetaMatrix(T.matrix(), theta_result, eomg, ev);
    }

    /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的矩阵
    /// @param theta「」 - 6个关节的角度，单位是弧度制
    /// @param M - 运算出的矩阵会存储在这个对象中
    bool SagittariusArmKinematics::getFKinMatrix(float theta[],
                                                 Eigen::MatrixXd& M_EE)
    {
        Eigen::VectorXd v_theta(JOINT_NUM);
        // v_theta << 0, 0, 0, 0, 0, 0;
        v_theta << theta[0], theta[1], theta[2], theta[3], theta[4], theta[5];
        M_EE = mr::FKinSpace(M, Slist, v_theta);
        return true;
    }

    /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的矩阵
    /// @param theta「」 - 6个关节的角度，单位是弧度制
    /// @param xyz - 运算出的位置会存储在这个对象中，单位是米
    /// @param euler - 运算出的姿态会存储在这个对象中，单位是角度制
    bool SagittariusArmKinematics::getFKinEuler(float theta[], float xyz[],
                                                float euler[])
    {
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix3d R;
        getFKinMatrix(theta, T);
        R = T.block<3, 3>(0, 0);
        Eigen::Vector3d eulerAngle = R.eulerAngles(2, 1, 0);

        // 通过四元素和逻辑运算，解决突变或万向节死锁问题
        Eigen::Quaterniond Q(R);
        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (Q.w() * Q.x() + Q.y() * Q.z());
        double cosr_cosp = +1.0 - 2.0 * (Q.x() * Q.x() + Q.y() * Q.y());
        euler[0] = atan2(sinr_cosp, cosr_cosp) * 180 / M_PI;

        // pitch (y-axis rotation)
        double sinp = +2.0 * (Q.w() * Q.y() - Q.z() * Q.x());
        if (fabs(sinp) >= 1)
            euler[1] = copysign(M_PI / 2, sinp) * 180 / M_PI; //
        else
            euler[1] = asin(sinp) * 180 / M_PI;

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (Q.w() * Q.z() + Q.x() * Q.y());
        double cosy_cosp = +1.0 - 2.0 * (Q.y() * Q.y() + Q.z() * Q.z());
        euler[2] = atan2(siny_cosp, cosy_cosp) * 180 / M_PI;
        if (euler[2] < 0) euler[2] += 360.0;

        // 通过矩阵和逻辑运算，解决突变或万向节死锁问题
        // double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        // bool singular = sy < 1e-6;
        // double x, y, z;
        // if (!singular)
        // {
        //     x = atan2( R(2,1), R(2,2));
        //     y = atan2(-R(2,0), sy);
        //     z = atan2( R(1,0), R(0,0));
        // }
        // else
        // {
        //     x = atan2(-R(1,2), R(1,1));
        //     y = atan2(-R(2,0), sy);
        //     z = 0;
        // }
        // eulerAngle << x, y, z;

        // euler[0] = eulerAngle[0];
        // euler[1] = eulerAngle[1];
        // euler[2] = eulerAngle[2];

        xyz[0] = T(0, 3);
        xyz[1] = T(1, 3);
        xyz[2] = T(2, 3);

        return true;
    }

    /// @brief getFKinMatrix 给定6个关节的角度，运算出机械臂末端的矩阵
    /// @param theta「」 - 6个关节的角度，单位是弧度制
    /// @param xyz - 运算出的位置会存储在这个对象中，单位是米
    /// @param quaternion - 运算出的姿态会存储在这个对象中
    bool SagittariusArmKinematics::getFKinQuaternion(float theta[], float xyz[],
                                                     float quaternion[])
    {
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix3d R;
        getFKinMatrix(theta, T);
        R = T.block<3, 3>(0, 0);
        Eigen::Quaterniond Q(R);
        quaternion[0] = Q.x();
        quaternion[1] = Q.y();
        quaternion[2] = Q.z();
        quaternion[3] = Q.w();
        xyz[0] = T(0, 3);
        xyz[1] = T(1, 3);
        xyz[2] = T(2, 3);
        return true;
    }

} // namespace sdk_sagittarius_arm
