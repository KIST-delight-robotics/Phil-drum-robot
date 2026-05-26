#include "hardware/motor.hpp"

Motor::Motor(int id)
    : id(id) {

}

Motor::~Motor() {

}

double Motor::joint_angle_to_motor_position(double joint_angle) {
    double motor_position;
    
    motor_position = (joint_angle - initial_joint_angle) * direction_sign;

    return motor_position;
}

double Motor::motor_position_to_joint_angle(double motor_position) {
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

DynamixelMotor::DynamixelMotor(int id)
    : Motor(id) {
        
}

void DynamixelMotor::dxl_torque_off() {
    // DXL 토크 Off
    uint8_t err = 0;
    pkt->write1ByteTxRx(port, dxl_id, 64, 0, &err);
}

void DynamixelMotor::write_command(std::vector<double> command) {
    sw = std::make_unique<dynamixel::GroupSyncWrite>(port, pkt, 108, 12);

    // 목표 값 배열 정의
    int32_t values_motor[3];
    uint8_t param_motor[12];

    command_to_values(values_motor, command);
    // memcpy를 사용해 정수 배열의 내용을 바이트 배열로 복사
    memcpy(param_motor, values_motor, sizeof(values_motor));
    
    sw->addParam(dxl_id, param_motor);

    sw->txPacket();
    sw->clearParam();
}

std::pair<bool, double> DynamixelMotor::read_data() {
    double position = 0.0;
    bool available = true;

    // GroupSyncRead 생성 (주소 132 = Present Position, 길이 4byte)
    dynamixel::GroupSyncRead groupSyncRead(port, pkt, 132, 4);

    // 모터 ID 등록
    groupSyncRead.addParam(dxl_id);

    // 데이터 요청
    int dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "[DynamixelMotor] SyncRead failed\n";
        return std::make_pair(false, position);
    }

    // 모터 값 읽기
    if (groupSyncRead.isAvailable(dxl_id, 132, 4)) {
        position = tick_to_angle(groupSyncRead.getData(dxl_id, 132, 4));
    } else {
        available = false;
        std::cerr << "[DynamixelMotor] ID:" << dxl_id << " data not available!\n";
    }

    // 다음 사용 위해 clear
    groupSyncRead.clearParam();

    return std::make_pair(available, position);;
}

int32_t DynamixelMotor::angle_to_tick(double angle)
{
    double degree = angle * 180.0 / M_PI;
    degree = std::clamp(degree, -180.0, 180.0);
    const double ticks_per_degree = 4096.0 / 360.0;
    double ticks = 2048.0 - (degree * ticks_per_degree);

    return static_cast<int32_t>(std::round(ticks));
}

double DynamixelMotor::tick_to_angle(int32_t ticks)
{
    const double degrees_per_tick = 360.0 / 4096.0;
    double angle = (2048.0 - static_cast<double>(ticks)) * degrees_per_tick;
    return angle * M_PI / 180.0;
}

void DynamixelMotor::command_to_values(int32_t values[], std::vector<double> command)
{
    // Profile Acceleration
    values[0] = static_cast<int32_t>(1000*command[0]);  // ms
    
    // Profile Velocity
    values[1] = static_cast<int32_t>(1000*command[1]);  // ms

    // Goal Position
    values[2] = angle_to_tick(command[2]);
}