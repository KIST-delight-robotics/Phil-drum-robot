#pragma once

#include <sstream>
#include <string>
#include <vector>

// 악보 txt 행 파싱/악기 번호 규칙 공용 (BehaviorPlanner, ImprovSelector).
// inline 전용: selector의 단독 링크를 깨지 않기 위해 cpp를 두지 않는다.

// 양끝 공백 제거. 전부 공백이면 "".
inline std::string trim_whitespace(const std::string& text) {
    size_t begin_pos = text.find_first_not_of(" \t\r\n");
    if (begin_pos == std::string::npos) {
        return "";
    }
    size_t end_pos = text.find_last_not_of(" \t\r\n");
    return text.substr(begin_pos, end_pos - begin_pos + 1);
}

// 한 행을 탭으로 나누고 각 셀을 trim.
inline std::vector<std::string> split_score_row(const std::string& row) {
    std::istringstream row_stream(row);
    std::string item;
    std::vector<std::string> items;
    while (std::getline(row_stream, item, '\t')) {
        items.push_back(trim_whitespace(item));
    }
    return items;
}

// 하이햇(5)은 페달(왼발 열)을 밟지 않았으면 오픈 하이햇(9)이다.
inline int open_hihat_note(int note_num, bool pedal_closed) {
    if (note_num == 5 && !pedal_closed) {
        return 9;
    }
    return note_num;
}
