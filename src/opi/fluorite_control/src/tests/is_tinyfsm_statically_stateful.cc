
// yes it is
// output:
//
// construct
// construct
// state1
// state2
// destruct
// destruct

#include "tinyfsm.hpp"

#include <iostream>

class ffssmm : public tinyfsm::MooreMachine<ffssmm> {
 public:
  ffssmm() { std::cout << "construct" << std::endl; }
  virtual ~ffssmm() { std::cout << "destruct" << std::endl; }

  virtual void entry() {}
  virtual void react(const tinyfsm::Event &) { std::cout << "react" << std::endl; }
};

class state1 : public ffssmm {
 public:
  void entry() override {}
  void react(const tinyfsm::Event &) override;
};

class state2 : public ffssmm {
 public:
  void entry() override {}
  void react(const tinyfsm::Event &) override;
};

void state1::react(const tinyfsm::Event &) { transit<state2>(); }
void state2::react(const tinyfsm::Event &) {}

void TestState() {
  if (ffssmm::is_in_state<state1>()) {
    std::cout << "state1" << std::endl;
  } else if (ffssmm::is_in_state<state2>()) {
    std::cout << "state2" << std::endl;
  } else {
    std::cout << "none" << std::endl;
  }
}

FSM_INITIAL_STATE(ffssmm, state1);

int main() {
  ffssmm::start();
  TestState();
  ffssmm::dispatch<tinyfsm::Event>(tinyfsm::Event{});
  TestState();
  return 0;
}
