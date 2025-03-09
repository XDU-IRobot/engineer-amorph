
// so the output is:
//
// construct
// construct_derive
// [ros2run]: Segmentation fault
//
// 对象是有实例化的，但是不知道为什么在react()里面访问不到非静态成员 :)...
// 有空还是得啃一啃源码

#include "tinyfsm.hpp"

#include <iostream>

class a : public tinyfsm::Event {};

class ffssmm : public tinyfsm::Fsm<ffssmm> {
 public:
  ffssmm() { std::cout << "construct" << std::endl; }
  virtual ~ffssmm() { std::cout << "destruct" << std::endl; }

  void entry();
  virtual void react(const tinyfsm::Event &) { std::cout << "react" << std::endl; }
};

class state1 : public ffssmm {
 public:
  state1() : ffssmm() { std::cout << "construct_derive" << std::endl; }
  ~state1() override { std::cout << "destruct_derive" << std::endl; }

  void react(const tinyfsm::Event &) override { std::cout << eee_ << std::endl; }

 private:
  int eee_{1};
};

FSM_INITIAL_STATE(ffssmm, state1);

using list = tinyfsm::FsmList<ffssmm>;

int main() {
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  list::dispatch<a>(a{});
  return 0;
}