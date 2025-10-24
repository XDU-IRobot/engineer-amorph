
#include "main.h"

#include <librm.hpp>

#include "rpc/usbcdc_transport.hpp"

extern "C" {

extern USBD_HandleTypeDef hUsbDeviceHS;

void AppMain(void) { rm::hal::Can can; }
}