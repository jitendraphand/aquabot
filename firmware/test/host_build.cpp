// Compiles the complete sketch on the host (CI smoke build) and runs a few control ticks.
#define HOST_BUILD 1
#include "../aquabot/aquabot.ino"
int main() {
  setup();
  for (int i = 0; i < 300; ++i) loop();
  return 0;
}
