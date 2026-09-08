#pragma once

#include <string>

// miniaudio 전방 선언용 타입 (헤더에 miniaudio.h를 노출하지 않기 위함)
struct ma_engine;
struct ma_context;

// 음악 재생 유틸 (PulseAudio, fire-and-forget).
// set_track()으로 곡을 미리 정해 두면 send_loop에서 play() 호출만으로 바로 재생된다.
// 주의: set_track은 이번 연주 1회용. 연주가 끝나거나 중단되면 clear_track()으로
//       리셋해야 다음 무음 연주에서 이전 곡이 다시 재생되지 않는다.
class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;

    // PulseAudio 백엔드로 엔진 초기화. 성공 시 true.
    bool initialize();

    // 재생할 곡명(확장자 제외)을 미리 설정. 예: "TIM" -> data/audio/TIM.wav
    void set_track(const std::string& name);

    // 설정된 곡명 초기화.
    void clear_track();

    // 설정된 곡을 즉시 재생. 곡이 없으면 재생 없이 false (정상 동작).
    bool play();

    // 곡명을 즉석에서 지정하면서 바로 재생.
    bool play(const std::string& name);

    // 재생 중인 모든 소리 정지.
    void stop();

    bool is_ready() const { return ready_; }

private:
    std::string make_path(const std::string& name) const;

    ma_context* context_ = nullptr;
    ma_engine*  engine_  = nullptr;
    bool ready_ = false;

    std::string base_dir_ = "drumrobot_server/data/audio";
    std::string track_;     // 미리 설정된 곡명 (확장자 제외)
};