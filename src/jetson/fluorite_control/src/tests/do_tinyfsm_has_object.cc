
// yes it does
// output:
// construct
// construct_derive
// 1
// 2
// 3
// 4
// 5
// destruct_derive
// destruct

#include "tinyfsm.hpp"

#include <iostream>

class a : public tinyfsm::Event {};

class ffssmm : public tinyfsm::Fsm<ffssmm> {
 public:
  ffssmm() { std::cout << "construct" << std::endl; }
  virtual ~ffssmm() { std::cout << "destruct" << std::endl; }

  void entry() {}
  virtual void react(const tinyfsm::Event &) { std::cout << "react" << std::endl; }
};

class state1 : public ffssmm {
 public:
  state1() : ffssmm() { std::cout << "construct_derive" << std::endl; }
  ~state1() override { std::cout << "destruct_derive" << std::endl; }

  void react(const tinyfsm::Event &) override {
    std::cout << eee_ << std::endl;
    ++eee_;
  }

 private:
  int eee_{1};
};

FSM_INITIAL_STATE(ffssmm, state1);

using list = tinyfsm::FsmList<ffssmm>;

int main() {
  ffssmm::start();
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  return 0;
}