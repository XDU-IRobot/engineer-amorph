
#pragma once

#include "tim.h"

/**
 * @brief   无源蜂鸣器驱动
 */
class Buzzer {
 public:
  Buzzer(TIM_HandleTypeDef& htim, uint32_t channel) : htim_(&htim), channel_(channel) {}

  void Begin() const { HAL_TIM_PWM_Start(htim_, channel_); }

  void SetFrequency(uint32_t frequency) const {
    if (frequency == 0) {
      __HAL_TIM_SET_COMPARE(htim_, channel_, 0);  // 关闭蜂鸣器
      return;
    }
    frequency *= 2;
    __HAL_TIM_SET_AUTORELOAD(htim_, 4000000 / frequency - 1);
    __HAL_TIM_SET_COMPARE(htim_, channel_, (4000000 / frequency) / 2);  // 50%占空比
  }

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
};

namespace buzzer_melody {

using rm::modules::BuzzerNote;

class EnterManualMode : public rm::modules::BuzzerMelody {
 public:
  EnterManualMode() = default;

  BuzzerNote Update(TimePoint now) override {
    using Freq = rm::modules::NoteFreqStandard;
    using Duration = rm::modules::NoteDuration160;

    constexpr std::array kMelody = {
        BuzzerNote(Freq::kC6, Duration::kSixteenth),
        BuzzerNote(Freq::kE6, Duration::kSixteenth),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);  // 播放完毕
    }

    auto note = kMelody[note_index_];

    auto elapsed = ElapsedMs(note_start_time_, now);
    if (elapsed >= note.duration) {
      note_index_++;
      note_start_time_ = now;
    }

    return note;
  }

  void Reset(TimePoint now) override {
    note_index_ = 0;
    note_start_time_ = now;
  }

 private:
  size_t note_index_{0};
  TimePoint note_start_time_;
};

template <size_t BeepsCount>
class HiBeeps : public rm::modules::BuzzerMelody {
 public:
  HiBeeps() = default;

  BuzzerNote Update(TimePoint now) override {
    using Freq = rm::modules::NoteFreqStandard;
    using Duration = rm::modules::NoteDuration<210>;

    if (note_index_ >= BeepsCount * 2) {
      return BuzzerNote(0, 0);  // 播放完毕
    }

    BuzzerNote note;
    if (note_index_ % 2 == 0) {
      note = BuzzerNote(Freq::kF6, Duration::kThirtySecond);  // 响声
    } else {
      note = BuzzerNote(Freq::kRest, Duration::kThirtySecond);  // 静音
    }

    auto elapsed = ElapsedMs(note_start_time_, now);
    if (elapsed >= note.duration) {
      note_index_++;
      note_start_time_ = now;
    }

    return note;
  }

  void Reset(TimePoint now) override {
    note_index_ = 0;
    note_start_time_ = now;
  }

 private:
  size_t note_index_{0};
  TimePoint note_start_time_;
};
}  // namespace buzzer_melody