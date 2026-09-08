#pragma once

#include <array>
#include <deque>
#include <random>
#include <string>
#include <vector>

// 즉흥 연주 곡 선택기. 목록 범위는 load()의 genre가 정한다 ("" = 전체 장르).
// 체이닝 선곡: 팔이 제때 도달 가능한 곡 중에서 cost(이동 거리, bpm 차, 최근 재생) 가중 랜덤.
//             전부 탈락하면 cost 최소 곡 + delay_first_hit.
// 순환 선곡: 이름순. 세션 첫 곡에 쓴다.
class ImprovSelector {
public:
    // 체이닝 문맥: 직전 파일이 끝난 시점의 상태.
    struct ChainContext {
        int cur_note_r = 1;         // 오른손 마지막 타격 악기 (5/9 변환 후)
        int cur_note_l = 1;         // 왼손 마지막 타격 악기
        double session_bpm = 0.0;   // override bpm (0 = 각 파일 bpm)
        double last_bpm = 100.0;    // 직전 파일에 적용된 bpm
    };

    // 악보 머리(첫 타격 행부터)의 한 행. 경계 충돌 예측에 쓴다.
    struct HeadRow {
        double beat = 0.0;          // 직전 행 이후 대기 (도착 기준)
        int note_r = 0;             // 5/9 변환 후 (0 = 그 손 안 침)
        int note_l = 0;
    };

    // 악보 목록 로드 (root 기준 상대 경로). 메타/좌표도 캐시. 악보가 없으면 false.
    bool load(const std::string& root_path, const std::string& genre);

    // 순환 선곡: 다음 악보 전체 경로 (끝나면 처음으로). 비어 있으면 "".
    std::string next_score();

    // 체이닝 선곡. 없으면 "". delay_first_hit가 서면 호출부가 첫 타격을 한 박 늦춰야 한다.
    std::string next_score(const ChainContext& chain, bool& delay_first_hit);

    // 좌표만 필요할 때(이어치기 gap 계산) load() 없이 좌표 캐시를 보장. 실패 시 false.
    bool ensure_coords();

    // 손 이동 시간 [s] (= 거리 / ARM_SPEED). 좌표 없거나 악기 번호가 무효면 0.
    double travel_sec(int from_note, int to_note, bool is_right) const;

    // 마지막으로 선택된 악보의 root 기준 상대 경로. 선택 전이면 "".
    std::string current_name() const;

    int score_count() const;

    void clear();

private:
    // 파일별 선곡용 메타데이터 (load()에서 1회 파싱).
    struct ScoreMeta {
        bool playable = false;      // 타격 행이 하나라도 있는지
        double file_bpm = 100.0;    // bpm 헤더 값 (없으면 100)
        double first_beat = 0.6;    // 첫 타격 행의 beat (= 이동 가용 시간)
        int first_note_r = 0;       // 첫 타격 악기 (0 = 그 손 안 침, 5/9 변환 후)
        int first_note_l = 0;
        std::vector<HeadRow> head_rows;     // 첫 타격 행부터 양손이 각각 한 번 칠 때까지
    };

    bool load_coordinates();
    void parse_meta(const std::string& file_path, ScoreMeta& meta);
    double travel_dist(int from_note, int to_note, bool is_right) const;
    double score_cost(int idx, const ChainContext& chain) const;

    // 경계 충돌 예측: 파킹->머리 타격 경로 샘플링, 팔 교차+근접이면 true(후보 제외).
    bool boundary_collision(const ChainContext& chain, const ScoreMeta& meta) const;

    void mark_played(int idx);

    std::string root_path;
    std::vector<std::string> score_files;   // root 기준 상대 경로 목록 (이름순 정렬)
    std::vector<ScoreMeta> score_metas;     // score_files와 같은 index
    std::deque<int> recent_picks;           // 최근 재생 index (뒤쪽이 최신)
    std::array<std::array<double, 3>, 10> right_pos{};  // 악기 1..9 좌표 (0은 미사용)
    std::array<std::array<double, 3>, 10> left_pos{};
    bool coords_ready = false;
    std::mt19937 rng;
    int cur_idx = -1;                       // 마지막으로 반환한 위치 (-1: 선택 전)
    int next_idx = 0;                       // 순환 정책이 반환할 위치
};
