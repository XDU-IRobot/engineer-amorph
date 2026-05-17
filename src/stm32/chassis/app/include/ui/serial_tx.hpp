#pragma once

#include <cstdint>

extern "C" void UiEnqueue(const uint8_t *pData, uint16_t Size);

void UiProcessTx();