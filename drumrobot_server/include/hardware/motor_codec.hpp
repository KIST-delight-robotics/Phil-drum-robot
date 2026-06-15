#pragma once

#include <linux/can.h>
#include <cmath>
#include <tuple>
#include <iostream>

using namespace std;

class TMotorServoCodec {
public:
    // node_id: motor.node_id, 0: tempoaray origin  1: setting a permanent zero point
    void setOrigin(uint32_t node_id, struct can_frame *frame, uint8_t set_origin_mode);
    
    // node_id: motor.node_id, pos [rad], speed[erpm], acceleration [erpm/s^2]
    void setPositionVelocity(uint32_t node_id, struct can_frame *frame, float pos, int16_t spd, int16_t RPA);

    // id, position, speed, current, temperature, error
    std::tuple<int, float, float, float, int8_t, int8_t> parseReceiveCommand(struct can_frame *frame);

    // node_id: motor.node_id, current break
    void setCurrentBrake(uint32_t node_id, struct can_frame *frame, float current);

    // node_id: motor.node_id, speed [erpm]
    void setVelocity(uint32_t node_id, struct can_frame *frame, float spd_erpm);

    // node_id: motor.node_id, pos [rad]
    void setPosition(uint32_t node_id, struct can_frame *frame, float pos);

private:
};

class TMotorMITCodec {
public:
    float GLOBAL_P_MIN = -12.5;
    float GLOBAL_P_MAX = 12.5;
    float GLOBAL_KP_MIN = 0;
    float GLOBAL_KP_MAX = 500;
    float GLOBAL_KD_MIN = 0;
    float GLOBAL_KD_MAX = 5;
    float GLOBAL_V_MIN, GLOBAL_V_MAX, GLOBAL_T_MIN, GLOBAL_T_MAX;
    float GLOBAL_I_MAX, Kt;


    void parseSendCommand(struct can_frame *frame, int canId, int dlc, float p_des, float v_des, float kp, float kd, float t_ff);
    std::tuple<int, float, float, float> parseReceiveCommand(struct can_frame *frame);

    // node_id: motor.node_id
    void getCheck(uint32_t node_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getControlMode(uint32_t node_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getExit(uint32_t node_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getZero(uint32_t node_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getQuickStop(uint32_t node_id, struct can_frame *frame);

private:
    int floatToUint(float x, float x_min, float x_max, unsigned int bits);
    float uintToFloat(int x_int, float x_min, float x_max, int bits);
};

class MaxonMotorCodec {
public:
    std::tuple<int, float, float, unsigned char> parseReceiveCommand(struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getActualPos(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getCheck(uint32_t can_send_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getStop(uint32_t node_id, struct can_frame *frame);

    // node_id: motor.node_id
    void getOperational(uint32_t node_id, struct can_frame *frame);

    // tx_pdo_id_0: motor.tx_pdo_ids[0]
    void getShutdown(uint32_t tx_pdo_id_0, struct can_frame *frame);

    // tx_pdo_id_0: motor.tx_pdo_ids[0]
    void getEnable(uint32_t tx_pdo_id_0, struct can_frame *frame);

    void getSync(struct can_frame *frame);

    // ====== CSP 모드 관련 명령 구성 메서드 ======

    // can_send_id: motor.can_send_id
    void getCSPMode(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getTorqueOffset(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getPosOffset(uint32_t can_send_id, struct can_frame *frame);

    // tx_pdo_id_1: motor.tx_pdo_ids[1]
    void setPosition(uint32_t tx_pdo_id_1, struct can_frame *frame, float p_des_radians);

    // ====== HMM 모드 관련 명령 구성 메서드 ======
    
    // can_send_id: motor.can_send_id
    void getHomeMode(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getFollowingErrorWindow(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getHomeoffsetDistance(uint32_t can_send_id, struct can_frame *frame, int degree);

    // motor.can_send_id
    void getHomePosition(uint32_t can_send_id, struct can_frame *frame, int degree);

    // can_send_id: motor.can_send_id
    void getHomingMethodL(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getHomingMethodR(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getHomingMethodTest(uint32_t can_send_id, struct can_frame *frame);

    // tx_pdo_id_0: motor.tx_pdo_ids[0]
    void getStartHoming(uint32_t tx_pdo_id_0, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getCurrentThresholdR(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getCurrentThresholdL(uint32_t can_send_id, struct can_frame *frame);

    // ===== CSV 모드 관련 명령 구성 메서드 ======

    // can_send_id: motor.can_send_id
    void getCSVMode(uint32_t can_send_id, struct can_frame *frame);

    // can_send_id: motor.can_send_id
    void getVelOffset(uint32_t can_send_id, struct can_frame *frame);

    // tx_pdo_id_2: motor.tx_pdo_ids[2]
    void getTargetVelocity(uint32_t tx_pdo_id_2, struct can_frame *frame, int targetVelocity);

    // ====== CST 모드 관련 명령 구성 메서드 ======

    // can_send_id: motor.can_send_id
    void getCSTMode(uint32_t can_send_id, struct can_frame *frame);

    // tx_pdo_id_3: motor.tx_pdo_ids[3]
    void setTorque(uint32_t tx_pdo_id_3, struct can_frame *frame, int targetTorquemNm);

    // can_send_id: motor.can_send_id
    void setTargetTorque(uint32_t can_send_id, struct can_frame *frame, int targetTorque);
};