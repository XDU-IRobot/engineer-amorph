#pragma once

#include <librm.hpp>
#include <etl/debounce.h>

class EnergyUnitStorage {
 public:
  EnergyUnitStorage(rm::hal::Pin slot_1, rm::hal::Pin slot_2)
      : slot_1_(std::move(slot_1)), slot_2_(std::move(slot_2)) {}

  void Update() {
    slot_1_.debounce.add(!slot_1_.pin.Read());
    slot_2_.debounce.add(!slot_2_.pin.Read());
  }

  bool slot_1() const { return slot_1_.debounce.is_set(); }
  bool slot_2() const { return slot_2_.debounce.is_set(); }

 private:
  struct Slot {
    rm::hal::Pin pin;
    etl::debounce<10> debounce;
  } slot_1_, slot_2_;
};