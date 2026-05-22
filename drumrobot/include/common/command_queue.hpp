#pragma once

#include <queue>
#include <mutex>
#include <string>

struct Command {
    std::string s;
};

class CommandQueue {
public:
    CommandQueue();
    ~CommandQueue();

    void push(std::string cmd);
    std::string pop();
    bool empty();

private:
    std::queue<std::string> queue_;
    std::mutex mutex_;
};