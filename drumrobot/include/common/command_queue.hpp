#pragma once

#include <queue>
#include <mutex>
#include <string>
#include <optional>

struct Command {
    std::string s;
};

class CommandQueue {
public:
    CommandQueue();
    ~CommandQueue();

    void push(const std::string& cmd);
    bool empty();
    std::optional<std::string> try_pop();

private:
    std::queue<std::string> queue_;
    std::mutex mutex_;
};