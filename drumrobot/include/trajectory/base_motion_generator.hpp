#pragma once

#include <queue>
#include <vector>
#include <array>
#include <map>
#include <fstream>
#include <iostream>
#include <cmath>

#include "nlohmann/json.hpp"

#include "common/motion_queue.hpp"
#include "common/robot_config.hpp"
#include "kinematics/kinematics_solver.hpp"

struct BaseMotionPoint {
    double right_x;
    double right_y;
    double right_z;

    double left_x;
    double left_y;
    double left_z;

    double waist;
    double right_wrist;
    double left_wrist;
};

class BaseMotionGenerator {
public:
    BaseMotionGenerator();
    ~BaseMotionGenerator();
 
    // drum_coordinate.json 로드 + solver 초기화
    void initialize();
 
    std::queue<BaseMotionPoint> generate_motion(std::vector<DrumEvent> rds, int num_point);
 
private:
    KinematicsSolver solver;        // 허리각 최적화(getWaistParams)용 IK
 
    // ---------------------------------------------------------
    // 악기 좌표 테이블
    //   악기 번호 1~10 : S, FT, MT, HT, HH(closed), R, RC, LC, OHH(open), RB
    //   index 0~9 로 저장 (악기번호 - 1)
    // ---------------------------------------------------------
 
    std::array<std::array<double, 3>, ROBOT::NUM_INSTRUMENT> drum_position_R;   // [m]
    std::array<std::array<double, 3>, ROBOT::NUM_INSTRUMENT> drum_position_L;
    std::array<double, ROBOT::NUM_INSTRUMENT> wrist_angle_on_impact_R;          // [rad]
    std::array<double, ROBOT::NUM_INSTRUMENT> wrist_angle_on_impact_L;
 
    bool table_loaded = false;
 
    // ---------------------------------------------------------
    // 상수
    // ---------------------------------------------------------
    const double BPM = 100.0;       // hitDetectionThreshold 계산용 기준 BPM
                                    // (PathManager::bpmOfScore 와 동일 역할)
 
    // ---------------------------------------------------------
    // 호출 간 유지되는 상태
    //   measure_state : [이전시간, 이전악기번호, state, 이전offset]
    //     state 0 : 0<-0 / 1 : 0<-1 / 2 : 1<-0 / 3 : 1<-1
    //   last_final_position : 직전 구간의 도착 좌표 (다음 구간 시작점)
    // ---------------------------------------------------------
    std::array<double, 4> measure_state_R{0.0, 1.0, 0.0, 0.0};  // 시작: snare(1)
    std::array<double, 4> measure_state_L{0.0, 1.0, 0.0, 0.0};
 
    std::array<double, 3> last_final_position_R{0.0, 0.0, 0.0};
    std::array<double, 3> last_final_position_L{0.0, 0.0, 0.0};
    std::array<double, 3> last_initial_position_R{0.0, 0.0, 0.0};
    std::array<double, 3> last_initial_position_L{0.0, 0.0, 0.0};
    bool has_last_hit = false;
 
    double last_waist_angle = 0.0;      // 직전 구간의 허리각 (해 없을 때 유지용)
 
    // ---------------------------------------------------------
    // 한 손에 대해 parseTrajectoryData 가 산출하는 정보
    // ---------------------------------------------------------
    struct ParsedHand {
        double initial_time = 0.0;
        double final_time = 0.0;
        int initial_inst = 0;       // 악기 번호 (0 = 없음)
        int final_inst = 0;
        int initial_offset = 0;
        int final_offset = 0;
        int is_making_trajectory = 0;   // 0:신규 / 1:진행중 / 2:제자리대기
        std::array<double, 4> next_state{0.0, 0.0, 0.0, 0.0};
    };
 
    // ---------------------------------------------------------
    // 내부 함수 (PathManager 대응)
    // ---------------------------------------------------------
    // parseTrajectoryData : 한 손의 타격 상태/시작·목표 악기 판정 (lookahead)
    ParsedHand parse_hand(const std::vector<DrumEvent>& rds,
                          bool is_right,
                          const std::array<double, 4>& prev_state);
 
    // checkOpenHihat : 하이햇 open/closed 에 따라 악기번호 보정 (HH=5 -> open이면 9)
    int check_open_hihat(int inst_num, bool is_closed_hihat) const;
 
    // getTargetPosition : 악기 번호 -> (x,y,z) + 손목각
    void get_target_position(int inst_num, bool is_right,
                             std::array<double, 3>& position_out,
                             double& wrist_angle_out) const;
 
    // calTimeScaling : ti..tf 에서 t -> s [0,1] (3차, 양끝 속도 0)
    double cal_time_scaling(double ti, double tf, double t) const;
 
    // makeTaskSpacePath : 3차 Bezier task-space 경로
    std::array<double, 3> make_task_space_path(const std::array<double, 3>& Pi,
                                               const std::array<double, 3>& Pf,
                                               int initial_offset,
                                               int final_offset,
                                               double s) const;
 
    // getWaistParams : the0 스캔하며 IK 풀어 최적 허리각 산출
    //   반환: optimized_q0 (해 없으면 0)
    double get_waist_params(const std::array<double, 3>& pR,
                            const std::array<double, 3>& pL,
                            double theta7, double theta8,
                            bool& solved_out) const;
};