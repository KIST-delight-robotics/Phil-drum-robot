#include "hardware/motor.hpp"

Motor::Motor(int id)
    : id(id) {

}

Motor::~Motor() {

}

double Motor::joint_angle2motor_position(double joint_angle) {
    double motor_position;
    
    motor_position = (joint_angle - initial_joint_angle) * direction_sign;

    return motor_position;
}

double Motor::motor_position2joint_angle(double motor_position) {
    double joint_angle;
    
    joint_angle = motor_position * direction_sign + initial_joint_angle;

    return joint_angle;
}

TMotor::TMotor(int id)
    : Motor(id) {

}

MaxonMotor::MaxonMotor(int id)
    : Motor(id) {
        
}

Dynamicxel::Dynamicxel(int id)
    : Motor(id) {
        
}