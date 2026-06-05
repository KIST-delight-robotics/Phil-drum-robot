#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <sstream>

// =============================================================
// 패킷 형식: OPCODE|arg1|arg2|...\n
// 필드 구분자: '|'   패킷 구분자: '\n'
//
// 예시:
//   LOOK|0|90\n               -> 정면 응시
//   GESTURE|nod\n             -> 끄덕임
//   MOVE|right_wrist|45|1.0\n -> right_wrist를 45도로 1.0초에 이동
//   POSE|home\n               -> home 포즈
//   QUIT\n                    -> 종료
// =============================================================

enum class Opcode {
    LOOK,       // 시선 제어       args: pan(deg), tilt(deg)
    GESTURE,    // 행동           args: type (nod / shake / wave / hi / hurray / happy)
    MOVE,       // 개별 관절 이동   args: motorName, angleDeg, [moveTime=2.0]
    POSE,       // 사전 정의 포즈   args: poseName (home / ready / shutdown)
    HIT,        // 드럼 타격       args: target (snare / ride / bass ...)
    PLAY,       // 악보 연주       args: scoreName
    
    // args 없음
    START,      // 시작
    READY,      // 상태 변경
    QUIT,       // 종료
    UNKNOWN
};

struct ParsedCommand {
    bool valid = false;
    Opcode opcode = Opcode::UNKNOWN;
    std::vector<std::string> args;
};

class CommandParser {
public:
    CommandParser();
    ~CommandParser();

    ParsedCommand parse(const std::string& cmd);
 
private:
    // '|' 구분자로 토큰 분리
    std::vector<std::string> split(const std::string& str, char delimiter) const;
 
    // 앞뒤 공백 및 개행 제거
    std::string trim(const std::string& str) const;
 
    // 문자열 → Opcode 변환
    Opcode to_opcode(const std::string& token) const;
 
    // Opcode별 최소 인자 수 검증
    bool validate_args(Opcode opcode, const std::vector<std::string>& args) const;
};