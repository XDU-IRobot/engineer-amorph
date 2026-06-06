#pragma once

#include <librm.hpp>

struct __attribute__((__packed__)) CustomControllerCommandCompressed {
  bool ok : 1;
  bool left_btn_a : 1;
  bool left_btn_b : 1;
  bool left_btn_c : 1;
  bool right_btn_a : 1;
  bool right_btn_b : 1;
  bool right_btn_c : 1;

  rm::modules::f16 q_left[6];
  rm::modules::f16 q_right[6];
};

struct CustomControllerCommand {
  bool ok;
  bool left_btn_a;
  bool left_btn_b;
  bool left_btn_c;
  bool right_btn_a;
  bool right_btn_b;
  bool right_btn_c;
  float q_left[6]{};
  float q_right[6]{};

  CustomControllerCommand() = default;
  explicit CustomControllerCommand(const CustomControllerCommandCompressed &compressed)
      : ok(compressed.ok),
        left_btn_a(compressed.left_btn_a),
        left_btn_b(compressed.left_btn_b),
        left_btn_c(compressed.left_btn_c),
        right_btn_a(compressed.right_btn_a),
        right_btn_b(compressed.right_btn_b),
        right_btn_c(compressed.right_btn_c) {
    for (int i = 0; i < 6; i++) {
      this->q_left[i] = rm::modules::F16ToF32(compressed.q_left[i]);
      this->q_right[i] = rm::modules::F16ToF32(compressed.q_right[i]);
    }
  }

  CustomControllerCommandCompressed ToCompressed() const {
    CustomControllerCommandCompressed ret;
    ret.ok = this->ok;
    ret.left_btn_a = this->left_btn_a;
    ret.left_btn_b = this->left_btn_b;
    ret.left_btn_c = this->left_btn_c;
    ret.right_btn_a = this->right_btn_a;
    ret.right_btn_b = this->right_btn_b;
    ret.right_btn_c = this->right_btn_c;
    for (int i = 0; i < 6; i++) {
      ret.q_left[i] = rm::modules::F32ToF16(this->q_left[i]);
      ret.q_right[i] = rm::modules::F32ToF16(this->q_right[i]);
    }
    return ret;
  }
};

struct SingleCustomCtrlCommand {
  SingleCustomCtrlCommand(CustomControllerCommand cmd, bool is_left)
      : arm{is_left ? kLeft : kRight},
        btn_a{is_left ? cmd.left_btn_a : cmd.right_btn_a},
        btn_b{is_left ? cmd.left_btn_b : cmd.right_btn_b},
        btn_c{is_left ? cmd.left_btn_c : cmd.right_btn_c} {
    for (int i = 0; i < 6; i++) {
      q[i] = is_left ? cmd.q_left[i] : cmd.q_right[i];
    }
  }
  enum {
    kLeft,
    kRight,
  } arm;
  bool btn_a;
  bool btn_b;
  bool btn_c;
  float q[6]{};
};

static_assert(sizeof(CustomControllerCommandCompressed) <= 30, "Payload struct too large");

class CustomController : public rm::device::Device {
 public:
  template <rm::device::RefereeRevision RefereeRev>
  explicit CustomController(rm::device::Referee<RefereeRev> &ref) {
    ref.AttachCallback([&](uint16_t cmd_id, uint8_t seq) {
      UNUSED(seq);
      if (cmd_id == 0x302) {  // custom_robot_data
        ReportStatus(kOk);
        std::memcpy(&cc_data_compressed_, &ref.data().custom_robot_data, sizeof(CustomControllerCommandCompressed));
        cc_data_ = CustomControllerCommand{cc_data_compressed_};
        for (auto &cb : on_recv_) {
          cb(*this);
        }
      }
    });
  }

  void OnRecv(std::function<void(const CustomController &)> cb) { on_recv_.emplace_back(std::move(cb)); }

  void ResetData() { std::memset(&cc_data_, 0, sizeof(CustomControllerCommand)); }

  const auto &data() const { return cc_data_; }

 private:
  etl::vector<std::function<void(const CustomController &)>, 5> on_recv_;
  CustomControllerCommand cc_data_;
  CustomControllerCommandCompressed cc_data_compressed_;
};

struct __attribute__((__packed__)) SlaveDataCan1 {
  bool ok : 1;
  bool btn_a;
  bool btn_b;
  bool btn_c;
};

struct __attribute__((__packed__)) SlaveDataCan2 {
  float q_12[2];
};

struct __attribute__((__packed__)) SlaveDataCan3 {
  float q_34[2];
};

struct __attribute__((__packed__)) SlaveDataCan4 {
  float q_56[2];
};

struct __attribute__((__packed__)) SlaveData {
  const uint8_t frame_head[2] = {0xa5, 0xa5};

  bool ok : 1;
  bool btn_a;
  bool btn_b;
  bool btn_c;
  float q[6];

  uint16_t crc16;

  SlaveData() = default;

  SlaveData(const SlaveData &other)
      : ok(other.ok), btn_a(other.btn_a), btn_b(other.btn_b), btn_c(other.btn_c), crc16(other.crc16) {
    std::memcpy(q, other.q, sizeof(q));
  }

  SlaveData(SlaveDataCan1 frag_1, SlaveDataCan2 frag_2, SlaveDataCan3 frag_3, SlaveDataCan4 frag_4) {
    ok = frag_1.ok;
    btn_a = frag_1.btn_a;
    btn_b = frag_1.btn_b;
    btn_c = frag_1.btn_c;
    q[0] = frag_2.q_12[0];
    q[1] = frag_2.q_12[1];
    q[2] = frag_3.q_34[0];
    q[3] = frag_3.q_34[1];
    q[4] = frag_4.q_56[0];
    q[5] = frag_4.q_56[1];
    crc16 = 0;
  }

  SlaveData &operator=(const SlaveData &other) {
    if (this != &other) {
      ok = other.ok;
      btn_a = other.btn_a;
      btn_b = other.btn_b;
      btn_c = other.btn_c;
      crc16 = other.crc16;
      std::memcpy(q, other.q, sizeof(q));
    }
    return *this;
  }
};
