#include "trajectory/collision_avoider.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

#include "nlohmann/json.hpp"

#include "common/robot_config.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::map;
using std::make_pair;
using std::pair;
using std::vector;

CollisionAvoider::CollisionAvoider() {
    reset(1, 1);
}

void CollisionAvoider::initialize() {
    using json = nlohmann::json;

    drumCoordinateR = MatrixXd::Zero(3, 10);
    drumCoordinateL = MatrixXd::Zero(3, 10);
    wristAngleOnImpactR = MatrixXd::Zero(1, 10);
    wristAngleOnImpactL = MatrixXd::Zero(1, 10);

    const std::string config_path = "drumrobot_server/config/drum_coordinate.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[CollisionAvoider] Failed to open config file: " << config_path << "\n";
        return;
    }

    json root;
    try {
        config_file >> root;
    } catch (const json::parse_error& e) {
        std::cerr << "[CollisionAvoider] JSON parse error in " << config_path << ": " << e.what() << "\n";
        return;
    }

    for (const auto& inst : root.at("instruments")) {
        std::string name = inst.at("name");
        int id = instrument_name_to_id.at(name);
        int col = note_to_column(id);
        if (col < 0) {
            continue;   // bass(0)는 손 악기 아님
        }

        const auto& right = inst.at("right");
        const auto& left  = inst.at("left");
        auto right_pos = right.at("position");
        auto left_pos  = left.at("position");

        drumCoordinateR(0, col) = right_pos.at(0).get<double>();
        drumCoordinateR(1, col) = right_pos.at(1).get<double>();
        drumCoordinateR(2, col) = right_pos.at(2).get<double>() + tableZOffset;

        drumCoordinateL(0, col) = left_pos.at(0).get<double>();
        drumCoordinateL(1, col) = left_pos.at(1).get<double>();
        drumCoordinateL(2, col) = left_pos.at(2).get<double>() + tableZOffset;

        wristAngleOnImpactR(0, col) = right.at("wrist_angle_deg").get<double>() * M_PI / 180.0;
        wristAngleOnImpactL(0, col) = left.at("wrist_angle_deg").get<double>() * M_PI / 180.0;
    }

    // 악기 10(라이드 벨)은 json에 없어 ride(6) 좌표로 대체
    drumCoordinateR.col(9) = drumCoordinateR.col(5);
    drumCoordinateL.col(9) = drumCoordinateL.col(5);
    wristAngleOnImpactR(0, 9) = wristAngleOnImpactR(0, 5);
    wristAngleOnImpactL(0, 9) = wristAngleOnImpactL(0, 5);

    coords_ready = true;

    table_file.open(tablePath, std::ifstream::binary);
    if (!table_file) {
        std::cerr << "[CollisionAvoider] 테이블 열기 실패: " << tablePath << " (충돌 회피 비활성)\n";
        return;
    }

    // 크기 검증: 11*11*19*10*12*19*10*12 셀 x 2bit
    const long long expect_size = 157251600LL;
    table_file.seekg(0, std::ios::end);
    long long file_size = static_cast<long long>(table_file.tellg());
    if (file_size != expect_size) {
        std::cerr << "[CollisionAvoider] 테이블 크기 불일치: " << file_size
                  << " != " << expect_size << " (충돌 회피 비활성)\n";
        table_file.close();
        return;
    }

    table_ready = true;
    std::cerr << "[CollisionAvoider] TABLE.bin 로드 완료 (" << file_size << " bytes)\n";
}

void CollisionAvoider::reset(int init_note_r, int init_note_l, double t0) {
    measureStateR = VectorXd::Zero(3);
    measureStateR(0) = t0;
    measureStateR(1) = (init_note_r > 0) ? static_cast<double>(init_note_r) : 1.0;  // 시작 위치 기본값: 스네어
    measureStateL = VectorXd::Zero(3);
    measureStateL(0) = t0;
    measureStateL(1) = (init_note_l > 0) ? static_cast<double>(init_note_l) : 1.0;
}

void CollisionAvoider::process_window(std::vector<DrumEvent>& rds, int start_idx, int end_idx, double bpm) {
    if (!table_ready || !coords_ready) {
        return;
    }
    if (start_idx < 0 || end_idx <= start_idx || end_idx >= static_cast<int>(rds.size())) {
        return;
    }
    if (bpm > 0.0) {
        bpmOfScore = bpm;
    }

    MatrixXd measureMatrix = build_matrix(rds, start_idx, end_idx);
    avoidCollision(measureMatrix); // 충돌 감지 및 회피
    write_back(measureMatrix, rds, start_idx);

    // 이번 window의 첫 구간만큼 팔 상태를 앞으로 넘긴다.
    TrajectoryData data = getTrajectoryData(measureMatrix, measureStateR, measureStateL);
    measureStateR = data.nextStateR;
    measureStateL = data.nextStateL;
}

// legacy measureMatrix 열 배치: [마디, 박자, 악기R, 악기L, 세기R, 세기L, 킥, 클로즈드햇, 누적시간]
int CollisionAvoider::note_to_column(int note_num) const {
    if (note_num >= 1 && note_num <= 8) {
        return note_num - 1;
    }
    if (note_num == 9) {
        return 8;
    }
    return -1;
}

Eigen::MatrixXd CollisionAvoider::build_matrix(const std::vector<DrumEvent>& rds, int start_idx, int end_idx) const {
    int row_count = end_idx - start_idx + 1;
    MatrixXd measureMatrix(row_count, 9);

    for (int i = 0; i < row_count; i++) {
        const DrumEvent& event = rds[start_idx + i];
        measureMatrix(i, 0) = static_cast<double>(event.bar);
        measureMatrix(i, 1) = event.beat;
        measureMatrix(i, 2) = static_cast<double>(event.note_num_R);
        measureMatrix(i, 3) = static_cast<double>(event.note_num_L);
        measureMatrix(i, 4) = static_cast<double>(event.velocity_R);
        measureMatrix(i, 5) = static_cast<double>(event.velocity_L);
        measureMatrix(i, 6) = event.is_kick ? 1.0 : 0.0;
        measureMatrix(i, 7) = event.is_closed_hihat ? 1.0 : 0.0;
        measureMatrix(i, 8) = event.t;
    }

    return measureMatrix;
}

void CollisionAvoider::write_back(const Eigen::MatrixXd& measureMatrix, std::vector<DrumEvent>& rds, int start_idx) const {
    int row_count = static_cast<int>(measureMatrix.rows());

    for (int i = 0; i < row_count; i++) {
        DrumEvent& event = rds[start_idx + i];
        event.note_num_R = static_cast<int>(std::lround(measureMatrix(i, 2)));
        event.note_num_L = static_cast<int>(std::lround(measureMatrix(i, 3)));
        event.velocity_R = static_cast<int>(std::lround(measureMatrix(i, 4)));
        event.velocity_L = static_cast<int>(std::lround(measureMatrix(i, 5)));
    }
}

////////////////////////////////////////////////////////////////////////////////
/*              이하 legacy DrumRobot2 PathManager.cpp 이식 원본               */
////////////////////////////////////////////////////////////////////////////////

void CollisionAvoider::avoidCollision(MatrixXd &measureMatrix)
{
    if (detectCollision(measureMatrix))    // 충돌 예측
    {
        for (int priority = 0; priority < 5; priority++)    // 수정방법 중 우선순위 높은 것부터 시도
        {
            if (modifyMeasure(measureMatrix, priority))     // 주어진 방법으로 회피되면 measureMatrix를 바꾸고 True 반환
            {
                std::cout << measureMatrix;
                std::cout << "\n 충돌 회피 성공 \n";
                break;
            }
        }
    }
    else
    {
        // std::cout << "\n 충돌 안함 \n";
    }
}

////////////////////////////////////////////////////////////////////////////////
/*                              Detect Collision                              */
////////////////////////////////////////////////////////////////////////////////

bool CollisionAvoider::detectCollision(MatrixXd &measureMatrix)
{
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureIntensityR = measureMatrix.col(4);
    VectorXd measureIntensityL = measureMatrix.col(5);

    VectorXd stateDCR = measureStateR;
    VectorXd stateDCL = measureStateL;

    // 충돌 예측을 위한 악보 범위 (목표위치 없는 부분 제거)
    int endIndex = findDetectionRange(measureMatrix);

    // 충돌 예측
    bool isColli = false;
    double stepSize = 5;
    for (int i = 0; i < endIndex-1; i++)
    {
        MatrixXd tmpMatrix = measureMatrix.block(i,0,measureMatrix.rows()-i,measureMatrix.cols());

        TrajectoryData data = getTrajectoryData(tmpMatrix, stateDCR, stateDCL);
        stateDCR = data.nextStateR;
        stateDCL = data.nextStateL;

        double dt = (data.t2 - data.t1)/stepSize;

        for (int j = 0; j < stepSize+1; j++)
        {
            double tR = dt * j + data.t1 - data.initialTimeR;
            double tL = dt * j + data.t1 - data.initialTimeL;

            double sR = calTimeScaling(0.0, data.finalTimeR - data.initialTimeR, tR);
            double sL = calTimeScaling(0.0, data.finalTimeL - data.initialTimeL, tL);

            VectorXd PR = makeTaskSpacePath(data.initialPositionR, data.finalPositionR, sR);
            VectorXd PL = makeTaskSpacePath(data.initialPositionL, data.finalPositionL, sL);

            double Tr = 1.0, hitR, hitL;
            if (measureTime(i+1) - measureTime(i) < 0.5)
            {
                Tr = (measureTime(i+1) - measureTime(i))/0.5;
            }

            if (measureIntensityR(i+1) == 0)
            {
                hitR = 10.0 * M_PI / 180.0;
            }
            else
            {
                hitR = measureIntensityR(i+1)*Tr*15.0*sin(M_PI*j/stepSize) * M_PI / 180.0;
            }

            if (measureIntensityL(i+1) == 0)
            {
                hitL = 10.0 * M_PI / 180.0;
            }
            else
            {
                hitL = measureIntensityL(i+1)*Tr*15.0*sin(M_PI*j/stepSize) * M_PI / 180.0;
            }

            if (checkTable(PR, PL, hitR, hitL))
            {
                isColli = true;
            }
        }

        if (isColli)
        {
            return true;
        }
    }

    return false;
}

int CollisionAvoider::findDetectionRange(MatrixXd &measureMatrix)
{
    // 뒤쪽 목표위치 없는 부분 제거
    // endIndex 까지 탐색
    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);

    bool endR = false, endL = false;
    int endIndex = measureTime.rows();
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore;

    // 뒤에서부터 읽으면서 양 팔 다 목표위치가 있는 인덱스 반환
    for (int i = 0; i < measureTime.rows(); i++)
    {
        if (measureInstrumentR(measureTime.rows() - 1 - i) != 0)
        {
            endR = true;
        }

        if (!endR)
        {
            if (std::round(10000 * hitDetectionThreshold) < std::round(10000 * (measureTime(measureTime.rows() - 1) - measureTime(measureTime.rows() - 1 - i))))
            {
                endR = true;
            }
        }

        if (measureInstrumentL(measureTime.rows() - 1 - i) != 0)
        {
            endL = true;
        }

        if (!endL)
        {
            if (std::round(10000 * hitDetectionThreshold) < std::round(10000 * (measureTime(measureTime.rows() - 1) - measureTime(measureTime.rows() - 1 - i))))
            {
                endL = true;
            }
        }

        if (endR && endL)
        {
            endIndex = measureTime.rows() - i;
            break;
        }
    }

    return endIndex;
}

bool CollisionAvoider::checkTable(VectorXd PR, VectorXd PL, double hitR, double hitL)
{
    double rangeMin[8] = {0.0, 0.0, -0.3, 0.3, 0.5, -0.3, 0.3, 0.5};
    double rangeMax[8] = {50.0*M_PI/180.0, 50.0*M_PI/180.0, 0.5, 0.7, 1.0, 0.5, 0.7, 1.0};

    std::vector<double> target = {hitR, hitL, PR(0), PR(1), PR(2), PL(0), PL(1), PL(2)};

    std::vector<size_t> dims = {11, 11, 19, 10, 12, 19, 10, 12};    // Rw Lw Rx Ry Rz Lx Ly Lz
    std::vector<size_t> targetIndex;

    // 인덱스 공간으로 변환
    for (int i = 0; i < 8; i++)
    {
        double indexValue = std::round((dims[i]-1)*(target[i] - rangeMin[i])/(rangeMax[i] - rangeMin[i]));

        size_t index;
        if (indexValue > static_cast<double>(dims[i]-1))
        {
            index = dims[i]-1;
        }
        else if (indexValue < 0.0)
        {
            index = 0;
        }
        else
        {
            index = static_cast<size_t>(indexValue);
        }

        targetIndex.push_back(index);
    }

    // 테이블 확인
    if (!table_ready)
    {
        return false;
    }

    std::size_t offsetIndex = getFlattenIndex(targetIndex, dims);

    std::pair<size_t, size_t> bitIndex = getBitIndex(offsetIndex);

    table_file.clear();
    table_file.seekg(bitIndex.first, std::ios::beg);

    char byte;
    table_file.read(&byte, 1);
    if (!table_file) {
        std::cerr << " 바이트 읽기 실패 \n";
        return false;
    }

    // byte에서 원하는 2비트 추출 (LSB 기준)
    uint8_t value = (byte >> bitIndex.second) & 0b11;

    if (value == 0b00)
    {
        return false;   // 충돌 안함
    }
    else
    {
        return true;    // 충돌 위험 or IK 안풀림
    }
}

size_t CollisionAvoider::getFlattenIndex(const std::vector<size_t>& indices, const std::vector<size_t>& dims)
{
    size_t flatIndex = 0;
    size_t multiplier = 1;

    for (int i = 0; i < 8; i++)
    {
        flatIndex += indices[7-i] * multiplier;
        multiplier *= dims[7-i];
    }

    return flatIndex;
}

std::pair<size_t, size_t> CollisionAvoider::getBitIndex(size_t offsetIndex)
{
    size_t bitNum = offsetIndex * 2;
    size_t byteNum = bitNum / 8;
    size_t bitIndex = bitNum - 8 * byteNum;

    return std::make_pair(byteNum, bitIndex);
}

////////////////////////////////////////////////////////////////////////////////
/*                              Avoid Collision                               */
////////////////////////////////////////////////////////////////////////////////

bool CollisionAvoider::modifyMeasure(MatrixXd &measureMatrix, int priority)
{
    // 주어진 방법으로 회피되면 measureMatrix를 바꾸고 True 반환

    std::string method = modificationMethods[priority];
    MatrixXd modifedMatrix = measureMatrix;
    bool modificationSuccess = true;
    int nModification = 0;  // 주어진 방법을 사용한 횟수

    while (modificationSuccess)
    {
        if (method == modificationMethods[0])
        {
            modificationSuccess = modifyCrash(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[1])
        {
            modificationSuccess = waitAndMove(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[2])
        {
            modificationSuccess = moveAndWait(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[3])
        {
            modificationSuccess = switchHands(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else if (method == modificationMethods[4])
        {
            modificationSuccess = deleteInst(modifedMatrix, nModification);     // 주어진 방법으로 수정하면 True 반환
        }
        else
        {
            modificationSuccess = false;
        }

        if (!detectCollision(modifedMatrix))   // 충돌 예측
        {
            measureMatrix = modifedMatrix;
            return true;
        }
        else
        {
            modifedMatrix = measureMatrix;
        }

        nModification++;
    }
    return false;
}

pair<int, int> CollisionAvoider::findModificationRange(VectorXd t, VectorXd instR, VectorXd instL)
{
    // 수정하면 안되는 부분 제외
    // detectLine 부터 수정 가능
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore; // 일단 이렇게 하면 1줄만 읽는 일 없음

    int detectLineR = 1;
    for (int i = 1; i < t.rows(); i++)
    {
        if (std::round(10000 * hitDetectionThreshold) < std::round(10000 * (t(i) - t(0))))
        {
            break;
        }

        if (instR(i) != 0)
        {
            detectLineR = i + 1;
            break;
        }
    }

    int detectLineL = 1;
    for (int i = 1; i < t.rows(); i++)
    {
        if (std::round(10000 * hitDetectionThreshold) < std::round(10000 * (t(i) - t(0))))
        {
            break;
        }

        if (instL(i) != 0)
        {
            detectLineL = i + 1;
            break;
        }
    }

    return make_pair(detectLineR, detectLineL);
}

bool CollisionAvoider::modifyCrash(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Modify Crash
    int cnt = 0;

    for (int i = detectLineR; i < t.rows(); i++)
    {
        if (instR(i) == 7)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 8;
                return true;
            }
            cnt++;
        }
        else if (instR(i) == 8)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 7;
                return true;
            }
            cnt++;
        }
    }

    for (int i = detectLineL; i < t.rows(); i++)
    {
        if (instL(i) == 7)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 8;
                return true;
            }
            cnt++;
        }
        else if (instL(i) == 8)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 7;
                return true;
            }
            cnt++;
        }
    }

    return false;
}

bool CollisionAvoider::switchHands(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    // 뒤쪽 목표위치 없는 부분 수정하면 안됨
    int endIndex = findDetectionRange(measureMatrix);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Modify Arm
    int cnt = 0;
    int maxDetectLine = 1;

    if (detectLineR > detectLineL)
    {
        maxDetectLine = detectLineR;
    }
    else
    {
        maxDetectLine = detectLineL;
    }

    for (int i = maxDetectLine; i < t.rows(); i++)
    {
        if ((instR(i) != 0) && (instL(i) != 0))
        {
            if (cnt == num)
            {
                int tmp = measureMatrix(i, 2);
                measureMatrix(i, 2) = measureMatrix(i, 3);
                measureMatrix(i, 3) = tmp;

                tmp = measureMatrix(i, 4);
                measureMatrix(i, 4) = measureMatrix(i, 5);
                measureMatrix(i, 5) = tmp;

                return true;
            }
            cnt++;
        }
    }

    for (int i = detectLineR; i < endIndex; i++)
    {
        if (instR(i) != 0)
        {
            if (instL(i) == 0)
            {
                if (cnt == num)
                {
                    measureMatrix(i, 3) = measureMatrix(i, 2);
                    measureMatrix(i, 5) = measureMatrix(i, 4);

                    measureMatrix(i, 2) = 0;
                    measureMatrix(i, 4) = 0;
                    return true;
                }
                cnt++;
            }
        }
    }

    for (int i = detectLineL; i < endIndex; i++)
    {
        if (instL(i) != 0)
        {
            if (instR(i) == 0)
            {
                if (cnt == num)
                {
                    measureMatrix(i, 2) = measureMatrix(i, 3);
                    measureMatrix(i, 4) = measureMatrix(i, 5);

                    measureMatrix(i, 3) = 0;
                    measureMatrix(i, 5) = 0;
                    return true;
                }
                cnt++;
            }
        }
    }

    return false;
}

bool CollisionAvoider::waitAndMove(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Wait and Move
    int cnt = 0;

    bool isStart = false;
    int startInst, startIndex;
    for (int i = detectLineL-1; i < t.rows(); i++)
    {
        if (instL(i) != 0)
        {
            if (isStart)
            {
                if (startInst != instL(i))
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(startIndex+j, 3) = startInst;
                            return true;
                        }
                        cnt++;
                    }
                }

                startIndex = i;
                startInst = instL(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instL(i);
            }
        }
    }

    isStart = false;
    for (int i = detectLineR-1; i < t.rows(); i++)
    {
        if (instR(i) != 0)
        {
            if (isStart)
            {
                if (startInst != instR(i))
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(startIndex+j, 2) = startInst;
                            return true;
                        }
                        cnt++;
                    }
                }

                startIndex = i;
                startInst = instR(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instR(i);
            }
        }
    }

    return false;
}

bool CollisionAvoider::moveAndWait(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    // Move and Wait
    int cnt = 0;

    bool isStart = false;
    int startInst, endInst, startIndex;
    for (int i = detectLineL-1; i < t.rows(); i++)
    {
        if (instL(i) != 0)
        {
            if (isStart)
            {
                endInst = instL(i);
                if (endInst != startInst)
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(i-j, 3) = endInst;
                            return true;
                        }
                        cnt++;
                    }
                }

                startIndex = i;
                startInst = instL(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instL(i);
            }
        }
    }

    isStart = false;
    for (int i = detectLineR-1; i < t.rows(); i++)
    {
        if (instR(i) != 0)
        {
            if (isStart)
            {
                endInst = instR(i);
                if (endInst != startInst)
                {
                    for (int j = 1; j < i-startIndex; j++)
                    {
                        if (cnt == num)
                        {
                            measureMatrix(i-j, 2) = endInst;
                            return true;
                        }
                        cnt++;
                    }
                }

                startIndex = i;
                startInst = instR(i);
            }
            else
            {
                isStart = true;
                startIndex = i;
                startInst = instR(i);
            }
        }
    }

    return false;
}

bool CollisionAvoider::deleteInst(MatrixXd &measureMatrix, int num)
{
    // 주어진 방법으로 수정하면 True 반환
    VectorXd t = measureMatrix.col(8);
    VectorXd instR = measureMatrix.col(2);
    VectorXd instL = measureMatrix.col(3);

    // 수정하면 안되는 부분 제외
    pair<int, int> detectLine = findModificationRange(t, instR, instL);

    // 뒤쪽 목표위치 없는 부분 수정하면 안됨
    int endIndex = findDetectionRange(measureMatrix);

    int detectLineR = detectLine.first;
    int detectLineL = detectLine.second;

    int cnt = 0;

    for (int i = detectLineR; i < endIndex; i++)
    {
        if (instR(i) != 0)
        {
            if (cnt == num)
            {
                measureMatrix(i, 2) = 0;
                measureMatrix(i, 4) = 0;
                return true;
            }
            cnt++;
        }
    }

    for (int i = detectLineL; i < endIndex; i++)
    {
        if (instL(i) != 0)
        {
            if (cnt == num)
            {
                measureMatrix(i, 3) = 0;
                measureMatrix(i, 5) = 0;
                return true;
            }
            cnt++;
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////
/*                       Task Space Trajectory (예측용)                        */
////////////////////////////////////////////////////////////////////////////////

CollisionAvoider::TrajectoryData CollisionAvoider::getTrajectoryData(MatrixXd &measureMatrix, VectorXd &stateR, VectorXd &stateL)
{
    TrajectoryData data;

    VectorXd measureTime = measureMatrix.col(8);
    VectorXd measureInstrumentR = measureMatrix.col(2);
    VectorXd measureInstrumentL = measureMatrix.col(3);
    VectorXd measureHihat = measureMatrix.col(7);

    data.t1 = measureMatrix(0, 8);
    data.t2 = measureMatrix(1, 8);

    // parse
    pair<VectorXd, VectorXd> dataR = parseTrajectoryData(measureTime, measureInstrumentR, measureHihat, stateR);
    pair<VectorXd, VectorXd> dataL = parseTrajectoryData(measureTime, measureInstrumentL, measureHihat, stateL);

    // state 업데이트
    data.nextStateR = dataR.second;
    data.nextStateL = dataL.second;

    // 시간
    data.initialTimeR = dataR.first(0);
    data.initialTimeL = dataL.first(0);

    data.finalTimeR = dataR.first(11);
    data.finalTimeL = dataL.first(11);

    // 악기
    VectorXd initialInstrumentR = dataR.first.block(1, 0, 10, 1);
    VectorXd initialInstrumentL = dataL.first.block(1, 0, 10, 1);

    VectorXd finalInstrumentR = dataR.first.block(12, 0, 10, 1);
    VectorXd finalInstrumentL = dataL.first.block(12, 0, 10, 1);

    pair<VectorXd, double> initialTagetR = getTargetPosition(initialInstrumentR, 'R');
    pair<VectorXd, double> initialTagetL = getTargetPosition(initialInstrumentL, 'L');

    pair<VectorXd, double> finalTagetR = getTargetPosition(finalInstrumentR, 'R');
    pair<VectorXd, double> finalTagetL = getTargetPosition(finalInstrumentL, 'L');

    // position
    data.initialPositionR = initialTagetR.first;
    data.initialPositionL = initialTagetL.first;

    data.finalPositionR = finalTagetR.first;
    data.finalPositionL = finalTagetL.first;

    // 타격 시 손목 각도
    data.initialWristAngleR = initialTagetR.second;
    data.initialWristAngleL = initialTagetL.second;

    data.finalWristAngleR = finalTagetR.second;
    data.finalWristAngleL = finalTagetL.second;

    return data;
}

pair<VectorXd, VectorXd> CollisionAvoider::parseTrajectoryData(VectorXd &t, VectorXd &inst, VectorXd &hihat, VectorXd &stateVector)
{
    map<int, int> instrumentMapping = {
        {1, 0}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}, {8, 7}, {11, 0}, {51, 0}, {61, 0}, {71, 0}, {81, 0}, {91, 0}, {9, 8}, {10, 9}};
    //    S       FT      MT      HT      HH       R      RC      LC       S        S        S        S        S        S     Open HH   RB

    VectorXd initialInstrument = VectorXd::Zero(10), finalInstrument = VectorXd::Zero(10);
    VectorXd outputVector = VectorXd::Zero(22);

    VectorXd nextStateVector;

    bool detectHit = false;
    double detectTime = 0, initialT, finalT;
    int detectInst = 0, initialInstNum, finalInstNum;
    int preState, nextState;
    const double e = 0.00001;
    double hitDetectionThreshold = 1.2 * 100.0 / bpmOfScore;

    // 타격 감지
    for (int i = 1; i < t.rows(); i++)
    {
        if (std::round(10000 * (hitDetectionThreshold + e)) < std::round(10000 * (t(i) - t(0))))
        {
            break;
        }

        if (inst(i) != 0)
        {
            detectHit = true;
            detectTime = t(i);
            detectInst = checkOpenHihat(inst(i), hihat(i));

            break;
        }
    }

    // inst
    preState = stateVector(2);

    // 타격으로 끝나지 않음
    if (inst(0) == 0)
    {
        // 궤적 생성 중
        if (preState == 2 || preState == 3)
        {
            nextState = preState;

            initialInstNum = stateVector(1);
            finalInstNum = detectInst;

            initialT = stateVector(0);
            finalT = detectTime;
        }
        else
        {
            // 다음 타격 감지
            if (detectHit)
            {
                nextState = 2;

                initialInstNum = stateVector(1);
                finalInstNum = detectInst;

                initialT = t(0);
                finalT = detectTime;
            }
            // 다음 타격 감지 못함
            else
            {
                nextState = 0;

                initialInstNum = stateVector(1);
                finalInstNum = stateVector(1);

                initialT = t(0);
                finalT = t(1);
            }
        }
    }
    // 타격으로 끝남
    else
    {
        // 다음 타격 감지
        if (detectHit)
        {
            nextState = 3;

            initialInstNum = checkOpenHihat(inst(0), hihat(0));
            finalInstNum = detectInst;

            initialT = t(0);
            finalT = detectTime;
        }
        // 다음 타격 감지 못함
        else
        {
            nextState = 1;

            initialInstNum = checkOpenHihat(inst(0), hihat(0));
            finalInstNum = checkOpenHihat(inst(0), hihat(0));

            initialT = t(0);
            finalT = t(1);
        }
    }

    initialInstrument(instrumentMapping[initialInstNum]) = 1.0;
    finalInstrument(instrumentMapping[finalInstNum]) = 1.0;
    outputVector << initialT, initialInstrument, finalT, finalInstrument;

    nextStateVector.resize(3);
    nextStateVector << initialT, initialInstNum, nextState;

    return std::make_pair(outputVector, nextStateVector);
}

int CollisionAvoider::checkOpenHihat(int instNum, int isHihat)
{
    if (instNum == 5)       // 하이햇인 경우
    {
        if (isHihat == 0)  // 오픈
        {
            return 9;
        }
        else            // 클로즈
        {
            return instNum;
        }
    }
    else                // 하이햇이 아니면 그냥 반환
    {
        return instNum;
    }
}

pair<VectorXd, double> CollisionAvoider::getTargetPosition(VectorXd &inst, char RL)
{
    // inst 벡터를 (x,y,z) position으로 변환
    double angle = 0.0;
    VectorXd p(3);

    if (inst.sum() == 0)
    {
        std::cout << "Instrument Vector Error!! : " << inst << "\n";
    }

    if (RL == 'R' || RL == 'r')
    {
        MatrixXd productP = drumCoordinateR * inst;
        p = productP.block(0, 0, 3, 1);

        MatrixXd productA = wristAngleOnImpactR * inst;
        angle = productA(0, 0);
    }
    else if (RL == 'L' || RL == 'l')
    {
        MatrixXd productP = drumCoordinateL * inst;
        p = productP.block(0, 0, 3, 1);

        MatrixXd productA = wristAngleOnImpactL * inst;
        angle = productA(0, 0);
    }
    else
    {
        std::cout << "RL Error!! : " << RL << "\n";
    }

    return std::make_pair(p, angle);
}

double CollisionAvoider::calTimeScaling(double ti, double tf, double t)
{
    // 3차 다항식
    float s;

    MatrixXd A;
    MatrixXd b;
    MatrixXd A_1;
    MatrixXd sol;

    A.resize(4, 4);
    b.resize(4, 1);

    A << 1, ti, ti * ti, ti * ti * ti,
        1, tf, tf * tf, tf * tf * tf,
        0, 1, 2 * ti, 3 * ti * ti,
        0, 1, 2 * tf, 3 * tf * tf;

    b << 0, 1, 0, 0;

    A_1 = A.inverse();
    sol = A_1 * b;

    s = sol(0, 0) + sol(1, 0) * t + sol(2, 0) * t * t + sol(3, 0) * t * t * t;

    return s;
}

VectorXd CollisionAvoider::makeTaskSpacePath(VectorXd &Pi, VectorXd &Pf, double s)
{
    float degree = 2.0;

    float xi = Pi(0), xf = Pf(0);
    float yi = Pi(1), yf = Pf(1);
    float zi = Pi(2), zf = Pf(2);

    VectorXd Ps;
    Ps.resize(3);

    if (Pi == Pf)
    {
        Ps(0) = xi;
        Ps(1) = yi;
        Ps(2) = zi;
    }
    else
    {
        Ps(0) = xi + s * (xf - xi);
        Ps(1) = yi + s * (yf - yi);

        if (zi > zf)
        {
            float a = zf - zi;
            float b = zi;

            Ps(2) = a * std::pow(s, degree) + b;
        }
        else
        {
            float amp = std::abs(zi - zf) / 2;    // sin 함수 amplitude
            float a = (zi - zf) * std::pow(-1, degree);
            float b = zf;

            Ps(2) = a * std::pow(s - 1, degree) + b + (amp * sin(M_PI * s));    // sin 궤적 추가
        }
    }

    return Ps;
}
