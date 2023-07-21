#include "nturt_integration_test/user_interface.hpp"

// stl include
#include <string>
#include <vector>

// ftxui include
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"

int main(int /*argc*/, char** /*argv*/) {
  UserInterface ui;
  ui.mainloop();
  return 0;
}
