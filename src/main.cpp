/*
 * Fridge Light Monitor for Thingy:53 (nRF5340)
 *
 * Wakes from System OFF on button press or BH1749 light threshold,
 * monitors door open/close events, transmits data over BLE NUS to gateway,
 * then returns to System OFF.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main()
{
  LOG_INF("ALC Fridge Light Monitor - Starting.");

  fridge::App app;

  if (!app.Init()) {
    LOG_ERR("Application init failed!");
    return -1;
  }

  app.Run();

  // Run() enters System OFF, so we should never reach here.
  LOG_ERR("Application returned unexpectedly!");
  return -1;
}
