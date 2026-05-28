#pragma once

#include <iostream>
#include <string>
#include <unistd.h>
#include <algorithm>
#include <cctype>

#include "common/app_context.hpp"
#include "common/command_queue.hpp"
 
class KeyboardHandler {
public:
    KeyboardHandler(AppContext &ctxRef, CommandQueue &commandQueueRef);
    ~KeyboardHandler();
 
    void run();
 
private:
    AppContext &ctx;
    CommandQueue &command_queue;

    bool quitting = false;
};