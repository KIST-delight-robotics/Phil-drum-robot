#pragma once

#include <string>
#include <vector>

enum class Opcode
{
    MOVE,
    TURN,
    PICK,
    SPEAK,
    UNKNOWN
};

struct ParsedCommand
{
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
};