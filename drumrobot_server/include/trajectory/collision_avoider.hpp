#pragma once

#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "common/motion_queue.hpp"

// legacy DrumRobot2 PathManager의 충돌 예측/회피(avoidCollision 계열) 이식.
// make_score_windows가 window를 복사하기 직전에 공유 rds를 in-place 수정한다.
// TABLE.bin이 없으면 전체가 no-op (fail-open).
class CollisionAvoider {
public:
    CollisionAvoider();

    void initialize();                              // drum_coordinate.json + TABLE.bin 로드
    // 연주/세션 시작 시 손 상태 시드. t0: 시드 시점의 누적 시간 (이어치기 전환은 절단 시각).
    void reset(int init_note_r, int init_note_l, double t0 = 0.0);
    // rds[start_idx..end_idx]를 검사/수정 후 손 상태를 한 줄 전진. window 복사 전에 호출.
    void process_window(std::vector<DrumEvent>& rds, int start_idx, int end_idx, double bpm);

    // 이어치기 전환이 실패했을 때 되돌리기 위한 손 상태 스냅샷
    std::pair<Eigen::VectorXd, Eigen::VectorXd> hand_state() const {
        return {measureStateR, measureStateL};
    }
    void set_hand_state(const std::pair<Eigen::VectorXd, Eigen::VectorXd>& state) {
        measureStateR = state.first;
        measureStateL = state.second;
    }

private:
    // ===== 이식 경계 (rds <-> measureMatrix 어댑터) =====
    bool table_ready = false;
    bool coords_ready = false;
    std::ifstream table_file;

    int note_to_column(int note_num) const;         // 악기 번호 -> 좌표 행렬 열 (1~8 -> 0~7, 9 -> 8)
    Eigen::MatrixXd build_matrix(const std::vector<DrumEvent>& rds, int start_idx, int end_idx) const;
    void write_back(const Eigen::MatrixXd& measureMatrix, std::vector<DrumEvent>& rds, int start_idx) const;

    // ===== 이하 legacy PathManager 이식 (함수명/본문 원본 유지) =====
    typedef struct {
        double t1, t2;    // 궤적 생성 시간

        double initialTimeR, finalTimeR;       // 전체 궤적에서 출발 시간, 도착 시간
        double initialTimeL, finalTimeL;

        Eigen::VectorXd initialPositionR, finalPositionR;  // 전체 궤적에서 출발 위치
        Eigen::VectorXd initialPositionL, finalPositionL;  // 전체 궤적에서 도착 위치

        double initialWristAngleR, finalWristAngleR;    // 손목 각도 출발 위치
        double initialWristAngleL, finalWristAngleL;    // 손목 각도 도착 위치

        Eigen::VectorXd nextStateR;            // 이전 시간, 이전 악기, 상태
        Eigen::VectorXd nextStateL;
    } TrajectoryData;

    // TABLE.bin은 legacy 좌표계(바닥 기준 z) 기준. 새 좌표계는 어깨 기준 z라 오프셋을 더한다.
    const double tableZOffset = 1.020 - 0.0605;     // legacy PartLength.height [m]

    std::string tablePath = "drumrobot_server/include/table/TABLE.bin";    // 테이블 위치

    Eigen::MatrixXd drumCoordinateR;        // 3x10, 악기별 위치 (TABLE 좌표계)
    Eigen::MatrixXd drumCoordinateL;
    Eigen::MatrixXd wristAngleOnImpactR;    // 1x10, 악기별 타격 시 손목 각도 [rad]
    Eigen::MatrixXd wristAngleOnImpactL;

    Eigen::VectorXd measureStateR, measureStateL;  ///< [이전 시간, 이전 악기, 상태(0:rest->rest 1:hit->rest 2:rest->hit 3:hit->hit)]
    double bpmOfScore = 100.0;

    std::map<int, std::string> modificationMethods = { ///< 악보 수정 방법 중 우선 순위
        { 0, "Crash"},
        { 1, "WaitAndMove"},
        { 2, "MoveAndWait"},
        { 3, "Switch"},
        { 4, "Delete"}
    };

    void avoidCollision(Eigen::MatrixXd &measureMatrix);

    //////////////////////////////////// Detect Collision
    bool detectCollision(Eigen::MatrixXd &measureMatrix);
    int findDetectionRange(Eigen::MatrixXd &measureMatrix);
    bool checkTable(Eigen::VectorXd PR, Eigen::VectorXd PL, double hitR, double hitL);
    size_t getFlattenIndex(const std::vector<size_t>& indices, const std::vector<size_t>& dims);
    std::pair<size_t, size_t> getBitIndex(size_t offsetIndex);

    //////////////////////////////////// Avoid Collision
    bool modifyMeasure(Eigen::MatrixXd &measureMatrix, int priority);
    std::pair<int, int> findModificationRange(Eigen::VectorXd t, Eigen::VectorXd instR, Eigen::VectorXd instL);
    bool modifyCrash(Eigen::MatrixXd &measureMatrix, int num);
    bool switchHands(Eigen::MatrixXd &measureMatrix, int num);
    bool waitAndMove(Eigen::MatrixXd &measureMatrix, int num);
    bool moveAndWait(Eigen::MatrixXd &measureMatrix, int num);
    bool deleteInst(Eigen::MatrixXd &measureMatrix, int num);

    //////////////////////////////////// Task Space Trajectory (예측용)
    TrajectoryData getTrajectoryData(Eigen::MatrixXd &measureMatrix, Eigen::VectorXd &stateR, Eigen::VectorXd &stateL);
    std::pair<Eigen::VectorXd, Eigen::VectorXd> parseTrajectoryData(Eigen::VectorXd &t, Eigen::VectorXd &inst, Eigen::VectorXd &hihat, Eigen::VectorXd &stateVector);
    int checkOpenHihat(int instNum, int isHihat);
    std::pair<Eigen::VectorXd, double> getTargetPosition(Eigen::VectorXd &inst, char RL);
    double calTimeScaling(double ti, double tf, double t);
    Eigen::VectorXd makeTaskSpacePath(Eigen::VectorXd &Pi, Eigen::VectorXd &Pf, double s);
};
