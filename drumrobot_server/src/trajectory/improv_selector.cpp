#include "trajectory/improv_selector.hpp"

#include "common/robot_config.hpp"
#include "util/score_row.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>

namespace {

// ===== 선곡 파라미터 =====
const double ARM_SPEED = 1.38;      // 팔 이동 속도 [m/s] (legacy 손 선택 모델과 같은 스케일)
const double DIST_MAX = 0.754;      // 킷에서 가장 먼 악기 쌍 거리 [m] (정규화용)
const double W_RIGHT = 1.0;         // 오른손 이동 거리 가중치
const double W_LEFT = 1.0;          // 왼손 이동 거리 가중치
const double W_BPM = 0.5;           // bpm 연속성 가중치
const double W_RECENT = 1.5;        // 최근 재생 반복 페널티 가중치
const double BPM_SCALE = 60.0;      // |Δbpm| 정규화 스케일
const double SOFTMAX_TEMP = 0.5;    // 가중 랜덤 온도 (낮을수록 cost 최소에 집중)
const int RECENT_KEEP = 8;          // 반복 페널티를 줄 최근 곡 개수

// ===== 경계 충돌 예측 파라미터 =====
const double ARM_LOOKAHEAD = 1.2;   // 이동 시작 lookahead [s] (HIT_DETECTION_THRESHOLD와 같은 값 유지)
const double CROSS_EPS = 0.0;       // 팔 교차 판정 여유 [m] (RH.x < LH.x + eps 이면 교차)
const double CLEAR_MARGIN = 0.30;   // 교차 상태에서 요구하는 최소 손 간 거리 [m]
const double SAMPLE_DT = 0.05;      // 경계 경로 샘플 간격 [s]
const size_t HEAD_ROW_MAX = 64;     // ScoreMeta에 캐시할 머리 행 수 상한

// 한 손의 경계 구간 직선 이동 계획 (time scaling 대신 선형 근사 — 경계 판정엔 충분)
struct MoveSegment {
    double move_start = 0.0;
    double hit_time = 0.0;
    std::array<double, 3> start_pos{};
    std::array<double, 3> end_pos{};
};

// 파킹 위치부터 자기 손 타격을 따라가는 계획 (row_time: 절대 시각, t=0 = 직전 이벤트)
std::vector<MoveSegment> build_hand_plan(bool is_right, int parked_note,
                                         const std::vector<ImprovSelector::HeadRow>& head_rows,
                                         const std::vector<double>& row_time,
                                         const std::array<std::array<double, 3>, 10>& pos_table) {
    std::vector<MoveSegment> plan;

    // 파킹 자리 표시용 퇴화 구간 (이후 구간이 없으면 계속 이 위치)
    MoveSegment parked_seg;
    if (parked_note >= 1 && parked_note <= 9) {
        parked_seg.start_pos = pos_table[parked_note];
    } else {
        parked_seg.start_pos = pos_table[1];    // 방어: 알 수 없는 파킹은 스네어로 간주
    }
    parked_seg.end_pos = parked_seg.start_pos;
    plan.push_back(parked_seg);

    size_t row_count = head_rows.size();
    for (size_t i = 0; i < row_count; i++) {
        int own_note = is_right ? head_rows[i].note_r : head_rows[i].note_l;
        if (own_note < 1 || own_note > 9) {
            continue;
        }

        double hit_time = row_time[i];

        // 이동 시작 = 타격이 lookahead 안으로 들어오는 첫 이벤트 시각 (직전 타격보다 앞설 수 없음)
        double move_start = hit_time;
        if (hit_time <= ARM_LOOKAHEAD) {
            move_start = 0.0;
        } else {
            for (size_t k = 0; k < i; k++) {
                if (hit_time - row_time[k] <= ARM_LOOKAHEAD) {
                    move_start = row_time[k];
                    break;
                }
            }
        }
        double prev_hit = plan.back().hit_time;
        if (move_start < prev_hit) {
            move_start = prev_hit;
        }

        MoveSegment seg;
        seg.move_start = move_start;
        seg.hit_time = hit_time;
        seg.start_pos = plan.back().end_pos;
        seg.end_pos = pos_table[own_note];
        plan.push_back(seg);
    }
    return plan;
}

// 계획상 시각 t의 손 위치.
std::array<double, 3> plan_position(const std::vector<MoveSegment>& plan, double t) {
    std::array<double, 3> pos = plan.back().end_pos;
    size_t seg_count = plan.size();
    for (size_t i = 0; i < seg_count; i++) {
        const MoveSegment& seg = plan[i];
        if (t < seg.move_start) {
            pos = seg.start_pos;
            break;
        }
        if (t <= seg.hit_time) {
            double span = seg.hit_time - seg.move_start;
            double ratio = (span > 0.0) ? (t - seg.move_start) / span : 1.0;
            for (int axis = 0; axis < 3; axis++) {
                pos[axis] = seg.start_pos[axis]
                          + ratio * (seg.end_pos[axis] - seg.start_pos[axis]);
            }
            break;
        }
        pos = seg.end_pos;  // 이 구간은 지났음 — 다음 구간이 없으면 여기 유지
    }
    return pos;
}

// 한 장르 디렉터리의 .txt 파일명을 "<dir_name>/<file>" 형태로 out_files에 모은다.
bool collect_scores(const std::string& root_path, const std::string& dir_name,
                    std::vector<std::string>& out_files) {
    std::error_code dir_error;
    std::filesystem::directory_iterator dir_iter(root_path + "/" + dir_name, dir_error);
    if (dir_error) {
        std::cerr << "[ImprovSelector] 디렉터리를 열 수 없습니다: " << root_path << "/" << dir_name
                  << " (" << dir_error.message() << ")\n";
        return false;
    }

    for (const std::filesystem::directory_entry& entry : dir_iter) {
        std::error_code file_error;
        if (!entry.is_regular_file(file_error) || file_error) {
            continue;
        }
        if (entry.path().extension() != ".txt") {
            continue;
        }
        out_files.push_back(dir_name + "/" + entry.path().filename().string());
    }
    return true;
}

}   // namespace

bool ImprovSelector::load(const std::string& root_path_arg, const std::string& genre) {
    clear();

    if (!genre.empty()) {
        // 지정 장르만
        if (!collect_scores(root_path_arg, genre, score_files)) {
            return false;
        }
    } else {
        // 장르 미지정: 모든 장르 하위 디렉터리 (root 바로 아래 단일 곡 .txt는 제외)
        std::error_code root_error;
        std::filesystem::directory_iterator root_iter(root_path_arg, root_error);
        if (root_error) {
            std::cerr << "[ImprovSelector] 악보 루트를 열 수 없습니다: " << root_path_arg
                      << " (" << root_error.message() << ")\n";
            return false;
        }
        for (const std::filesystem::directory_entry& entry : root_iter) {
            std::error_code sub_error;
            if (!entry.is_directory(sub_error) || sub_error) {
                continue;
            }
            collect_scores(root_path_arg, entry.path().filename().string(), score_files);
        }
    }

    if (score_files.empty()) {
        std::cerr << "[ImprovSelector] .txt 악보가 없습니다: " << root_path_arg
                  << (genre.empty() ? "" : "/" + genre) << "\n";
        return false;
    }

    std::sort(score_files.begin(), score_files.end());
    root_path = root_path_arg;

    // 파일별 선곡 메타데이터 캐시
    int file_count = static_cast<int>(score_files.size());
    score_metas.assign(file_count, ScoreMeta());
    int playable_count = 0;
    for (int i = 0; i < file_count; i++) {
        parse_meta(root_path + "/" + score_files[i], score_metas[i]);
        if (score_metas[i].playable) {
            playable_count++;
        }
    }

    coords_ready = load_coordinates();
    if (!coords_ready) {
        std::cerr << "[ImprovSelector] 악기 좌표 없이 동작합니다 (이동 거리 필터 비활성)\n";
    }

    std::random_device seed_gen;
    rng.seed(seed_gen());

    std::cerr << "[ImprovSelector] 악보 " << file_count << "개 로드 (연주 가능 "
              << playable_count << "개)\n";
    return true;
}

// drum_coordinate.json에서 악기 1..9의 양손 타격 좌표를 읽는다.
bool ImprovSelector::load_coordinates() {
    const std::string config_path = "drumrobot_server/config/drum_coordinate.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[ImprovSelector] 좌표 파일을 열 수 없습니다: " << config_path << "\n";
        return false;
    }

    nlohmann::json root;
    try {
        config_file >> root;
    } catch (const std::exception& parse_error) {
        std::cerr << "[ImprovSelector] 좌표 JSON 파싱 실패: " << parse_error.what() << "\n";
        return false;
    }

    try {
        for (const nlohmann::json& inst : root.at("instruments")) {
            std::string name = inst.at("name").get<std::string>();
            if (instrument_name_to_id.count(name) == 0) {
                continue;
            }
            int note_id = instrument_name_to_id.at(name);
            if (note_id < 1 || note_id > 9) {
                continue;   // bass(0)는 손 이동과 무관
            }
            const nlohmann::json& right_arr = inst.at("right").at("position");
            const nlohmann::json& left_arr = inst.at("left").at("position");
            for (int axis = 0; axis < 3; axis++) {
                right_pos[note_id][axis] = right_arr.at(axis).get<double>();
                left_pos[note_id][axis] = left_arr.at(axis).get<double>();
            }
        }
    } catch (const std::exception& field_error) {
        std::cerr << "[ImprovSelector] 좌표 형식 오류: " << field_error.what() << "\n";
        return false;
    }
    return true;
}

// bpm 헤더/첫 타격과, 양손이 한 번씩 칠 때까지의 머리 행을 캐시.
void ImprovSelector::parse_meta(const std::string& file_path, ScoreMeta& meta) {
    std::ifstream input_file(file_path);
    if (!input_file.is_open()) {
        std::cerr << "[ImprovSelector] 악보를 열 수 없습니다: " << file_path << "\n";
        return;
    }

    bool head_seen_r = false;
    bool head_seen_l = false;

    std::string row;
    while (std::getline(input_file, row)) {
        std::vector<std::string> items = split_score_row(row);
        if (items.empty() || items[0].empty()) {
            continue;
        }
        if (items[0] == "bpm") {
            if (items.size() >= 2) {
                try {
                    meta.file_bpm = std::stod(items[1]);
                } catch (...) {
                }
            }
            continue;
        }
        if (items[0] == "end") {
            break;
        }
        if (items.size() < 8) {
            break;
        }

        double beat = 0.0;
        int note_r = 0;
        int note_l = 0;
        int kick = 0;
        int hihat = 0;
        try {
            beat = std::stod(items[1]);
            note_r = std::stoi(items[2]);
            note_l = std::stoi(items[3]);
            kick = std::stoi(items[6]);
            hihat = std::stoi(items[7]);
        } catch (...) {
            break;
        }

        if (!meta.playable && note_r == 0 && note_l == 0 && kick != 1) {
            continue;   // 머리 무음 행 (trim 대상과 동일 판정)
        }

        note_r = open_hihat_note(note_r, hihat == 1);
        note_l = open_hihat_note(note_l, hihat == 1);

        if (!meta.playable) {
            // 첫 타격 발견
            meta.playable = true;
            meta.first_beat = beat;
            meta.first_note_r = note_r;
            meta.first_note_l = note_l;
        }

        // 머리 행 캐시 (중간 무음 행도 시간 누적을 위해 포함)
        HeadRow head_row;
        head_row.beat = beat;
        head_row.note_r = note_r;
        head_row.note_l = note_l;
        meta.head_rows.push_back(head_row);

        if (note_r != 0) {
            head_seen_r = true;
        }
        if (note_l != 0) {
            head_seen_l = true;
        }
        if ((head_seen_r && head_seen_l) || meta.head_rows.size() >= HEAD_ROW_MAX) {
            break;
        }
    }
}

double ImprovSelector::travel_dist(int from_note, int to_note, bool is_right) const {
    if (!coords_ready) {
        return 0.0;     // 좌표 없으면 이동 제약 없이 동작
    }
    if (from_note < 1 || from_note > 9 || to_note < 1 || to_note > 9) {
        return 0.0;
    }
    const std::array<std::array<double, 3>, 10>& pos = is_right ? right_pos : left_pos;
    double dx = pos[to_note][0] - pos[from_note][0];
    double dy = pos[to_note][1] - pos[from_note][1];
    double dz = pos[to_note][2] - pos[from_note][2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 후보 악보의 선곡 비용. 각 항은 0~1로 정규화 후 가중.
double ImprovSelector::score_cost(int idx, const ChainContext& chain) const {
    const ScoreMeta& meta = score_metas[idx];

    double dist_r = 0.0;
    double dist_l = 0.0;
    if (meta.first_note_r != 0) {
        dist_r = travel_dist(chain.cur_note_r, meta.first_note_r, true);
    }
    if (meta.first_note_l != 0) {
        dist_l = travel_dist(chain.cur_note_l, meta.first_note_l, false);
    }

    double bpm_eff = (chain.session_bpm > 0.0) ? chain.session_bpm : meta.file_bpm;
    double bpm_gap = std::abs(bpm_eff - chain.last_bpm) / BPM_SCALE;

    // 최근에 재생했을수록 1에 가까운 페널티 (deque 뒤쪽이 최신)
    double recent_pen = 0.0;
    int recent_size = static_cast<int>(recent_picks.size());
    for (int i = 0; i < recent_size; i++) {
        if (recent_picks[i] == idx) {
            recent_pen = static_cast<double>(i + 1) / static_cast<double>(RECENT_KEEP);
        }
    }

    return W_RIGHT * std::min(dist_r / DIST_MAX, 1.0)
         + W_LEFT * std::min(dist_l / DIST_MAX, 1.0)
         + W_BPM * std::min(bpm_gap, 1.0)
         + W_RECENT * recent_pen;
}

// 파킹->머리 타격 경로를 샘플링해 팔 교차(RH.x < LH.x) + 근접이면 true.
bool ImprovSelector::boundary_collision(const ChainContext& chain, const ScoreMeta& meta) const {
    if (!coords_ready || meta.head_rows.empty()) {
        return false;
    }

    double bpm_eff = (chain.session_bpm > 0.0) ? chain.session_bpm : meta.file_bpm;
    double beat_scale = 100.0 / bpm_eff;

    // 머리 행의 절대 시각 (t=0 = 직전 파일 마지막 이벤트)
    std::vector<double> row_time;
    double time_acc = 0.0;
    size_t row_count = meta.head_rows.size();
    for (size_t i = 0; i < row_count; i++) {
        time_acc += meta.head_rows[i].beat * beat_scale;
        row_time.push_back(time_acc);
    }

    std::vector<MoveSegment> plan_r = build_hand_plan(true, chain.cur_note_r,
                                                      meta.head_rows, row_time, right_pos);
    std::vector<MoveSegment> plan_l = build_hand_plan(false, chain.cur_note_l,
                                                      meta.head_rows, row_time, left_pos);

    // 검사 창: 두 계획의 마지막 도착 시각 중 늦은 쪽까지
    double window_end = plan_r.back().hit_time;
    if (plan_l.back().hit_time > window_end) {
        window_end = plan_l.back().hit_time;
    }
    if (window_end <= 0.0) {
        return false;   // 머리 구간에 손 타격 없음 (킥 전용 등)
    }

    for (double t = 0.0; t <= window_end + 1e-9; t += SAMPLE_DT) {
        std::array<double, 3> pos_r = plan_position(plan_r, t);
        std::array<double, 3> pos_l = plan_position(plan_l, t);

        bool crossed = pos_r[0] < pos_l[0] + CROSS_EPS;
        if (!crossed) {
            continue;
        }

        double dx = pos_r[0] - pos_l[0];
        double dy = pos_r[1] - pos_l[1];
        double dz = pos_r[2] - pos_l[2];
        double hand_gap = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (hand_gap < CLEAR_MARGIN) {
            return true;
        }
    }
    return false;
}

void ImprovSelector::mark_played(int idx) {
    recent_picks.push_back(idx);
    while (static_cast<int>(recent_picks.size()) > RECENT_KEEP) {
        recent_picks.pop_front();
    }
}

std::string ImprovSelector::next_score() {
    if (score_files.empty()) {
        return "";
    }

    cur_idx = next_idx;
    next_idx = (next_idx + 1) % static_cast<int>(score_files.size());
    mark_played(cur_idx);
    return root_path + "/" + score_files[cur_idx];
}

std::string ImprovSelector::next_score(const ChainContext& chain, bool& delay_first_hit) {
    delay_first_hit = false;
    if (score_files.empty()) {
        return "";
    }

    int file_count = static_cast<int>(score_files.size());
    std::vector<int> playable;
    std::vector<int> feasible;

    for (int i = 0; i < file_count; i++) {
        const ScoreMeta& meta = score_metas[i];
        if (!meta.playable) {
            continue;
        }
        if (i == cur_idx && file_count > 1) {
            continue;   // 같은 곡 즉시 반복 방지
        }
        playable.push_back(i);

        // 하드 필터: 첫 타격 beat(가용 시간) 안에 양손 도달 가능한가
        double bpm_eff = (chain.session_bpm > 0.0) ? chain.session_bpm : meta.file_bpm;
        double gap_sec = meta.first_beat * 100.0 / bpm_eff;
        double need_r = 0.0;
        double need_l = 0.0;
        if (meta.first_note_r != 0) {
            need_r = travel_dist(chain.cur_note_r, meta.first_note_r, true) / ARM_SPEED;
        }
        if (meta.first_note_l != 0) {
            need_l = travel_dist(chain.cur_note_l, meta.first_note_l, false) / ARM_SPEED;
        }
        if (need_r <= gap_sec && need_l <= gap_sec) {
            feasible.push_back(i);
        }
    }

    if (playable.empty()) {
        return "";
    }

    int picked = -1;
    if (!feasible.empty()) {
        // cost 가중 랜덤. 경계 충돌 예측에 걸리면 빼고 재추첨.
        std::vector<int> candidates = feasible;
        std::vector<double> weights;
        int candidate_count = static_cast<int>(candidates.size());
        for (int i = 0; i < candidate_count; i++) {
            double cost = score_cost(candidates[i], chain);
            weights.push_back(std::exp(-cost / SOFTMAX_TEMP));
        }

        while (!candidates.empty()) {
            std::discrete_distribution<int> picker(weights.begin(), weights.end());
            int pick_pos = picker(rng);
            int candidate_idx = candidates[pick_pos];

            if (!boundary_collision(chain, score_metas[candidate_idx])) {
                picked = candidate_idx;
                break;
            }

            std::cerr << "[ImprovSelector] 경계 충돌 예측으로 제외: "
                      << score_files[candidate_idx] << "\n";
            candidates.erase(candidates.begin() + pick_pos);
            weights.erase(weights.begin() + pick_pos);
        }

        if (picked < 0) {
            std::cerr << "[ImprovSelector] 모든 후보가 경계 충돌 예측으로 제외되어 "
                         "cost 최소 곡 + gap 연장으로 진행합니다\n";
        }
    }

    if (picked < 0) {
        // 전멸: cost 최소 곡을 뽑고 첫 타격을 늦추게 한다
        double best_cost = 0.0;
        int playable_count = static_cast<int>(playable.size());
        for (int i = 0; i < playable_count; i++) {
            double cost = score_cost(playable[i], chain);
            if (picked < 0 || cost < best_cost) {
                picked = playable[i];
                best_cost = cost;
            }
        }
        delay_first_hit = true;
        std::cerr << "[ImprovSelector] 이어치기 가능한 악보가 없어 gap을 한 박으로 늘립니다: "
                  << score_files[picked] << "\n";
    }

    cur_idx = picked;
    next_idx = (picked + 1) % file_count;
    mark_played(picked);
    return root_path + "/" + score_files[picked];
}

bool ImprovSelector::ensure_coords() {
    if (!coords_ready) {
        coords_ready = load_coordinates();
    }
    return coords_ready;
}

double ImprovSelector::travel_sec(int from_note, int to_note, bool is_right) const {
    return travel_dist(from_note, to_note, is_right) / ARM_SPEED;
}

std::string ImprovSelector::current_name() const {
    if (cur_idx < 0 || cur_idx >= static_cast<int>(score_files.size())) {
        return "";
    }
    return score_files[cur_idx];
}

int ImprovSelector::score_count() const {
    return static_cast<int>(score_files.size());
}

void ImprovSelector::clear() {
    root_path.clear();
    score_files.clear();
    score_metas.clear();
    recent_picks.clear();
    coords_ready = false;
    cur_idx = -1;
    next_idx = 0;
}
