#pragma once

#include <string>

// 연주 중단 시점의 재개 지점 (마디는 중단 시 MotionQueue 맨 앞 이벤트 기준).
// play_id: 일반 곡은 play_list 조회 키, improv는 표시용 라벨(ctx.play_id 그대로 보존).
struct PausePoint {
    std::string play_id;    // 재개할 곡 id
    int bar = 0;            // 재개 시작 마디
    int row = 0;            // 마지막으로 친 악보 행 (이어치기 전환은 row+1부터 재개, 0 = 행 정보 없음)
    bool valid = false;     // 저장된 재개 지점이 있는지

    // improv 재개 정보 (GET_STATUS에는 내보내지 않는다 — wire 포맷 불변)
    bool improv = false;        // true면 improv 세션의 재개 지점
    std::string genre;          // 장르 디렉터리명 ("" = 전체 장르)
    double bpm = 0.0;           // override bpm (0 = 각 파일의 bpm 사용)

    // 일반 곡 재개 지점 저장
    void save(const std::string& id, int bar_num, int row_num) {
        play_id = id;
        bar = bar_num;
        row = row_num;
        improv = false;
        genre.clear();
        bpm = 0.0;
        valid = true;
    }

    // improv 재개 지점 저장 (재개는 genre/bpm로 새 세션, id는 라벨)
    void save(const std::string& id, const std::string& genre_dir, double bpm_val) {
        play_id = id;
        bar = 0;
        row = 0;
        improv = true;
        genre = genre_dir;
        bpm = bpm_val;
        valid = true;
    }

    // 재개 지점 폐기 (RESUME 불가 상태로)
    void clear() {
        valid = false;
    }
};
