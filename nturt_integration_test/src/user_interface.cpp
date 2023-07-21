#include "nturt_integration_test/user_interface.hpp"

// stl include
#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

// ftxui include
#include "ftxui/component/component.hpp"
#include "ftxui/component/loop.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/dom/elements.hpp"

using namespace ::ftxui;

void UserInterface::mainloop() {
  /* overview ----------------------------------------------------------------*/
  const Component overview_renderer = Renderer([&] { return vbox({}); });

  /* vcu ---------------------------------------------------------------------*/
  const Component vcu_renderer = Renderer([&] {
    const Element error_code = window(
        text("error_code") | bold,
        text(std::to_string(
            std::chrono::system_clock::now().time_since_epoch().count())));

    return vbox({error_code});
  });

  /* front_sensor ------------------------------------------------------------*/
  const Component front_sensor_renderer = Renderer([&] { return vbox({}); });

  /* rear_box ----------------------------------------------------------------*/
  const Component rear_box_renderer = Renderer([&] { return vbox({}); });

  /* invereter ---------------------------------------------------------------*/
  const Component inverter_renderer = Renderer([&] { return vbox({}); });

  /* bms ---------------------------------------------------------------------*/
  const Component bms_renderer = Renderer([&] { return vbox({}); });

  /* tab ---------------------------------------------------------------------*/
  int tab_index = 0;
  const std::vector<std::string> tab_entries{
      "overview", "vcu", "front_sensor", "rear_box", "inverter", "bms",
  };

  // create a tab menu
  const Component tab_selection = [&]() {
    auto option = MenuOption::HorizontalAnimated();
    option.underline.SetAnimation(std::chrono::seconds(1),
                                  animation::easing::ElasticOut);
    option.entries_option.transform = [](EntryState state) {
      Element e = text(state.label) | hcenter | flex;
      if (state.active && state.focused) e = e | bold;
      if (!state.focused && !state.active) e = e | dim;
      return e;
    };
    option.underline.color_inactive = Color::Default;
    option.underline.color_active = Color::BlueLight;
    return Menu(&tab_entries, &tab_index, option);
  }();

  const Component tab_content =
      Container::Tab({overview_renderer, vcu_renderer, front_sensor_renderer,
                      rear_box_renderer, inverter_renderer, bms_renderer},
                     &tab_index);

  /* screen ------------------------------------------------------------------*/
  const Element screen_title = text("NTURT Integration Test") | bold | hcenter;

  // combine all components that needs to be updated into one component
  const Component main_component =
      Container::Vertical({tab_selection, tab_content});

  const Component main_renderer = Renderer(main_component, [&] {
    return vbox({screen_title, separator(), tab_selection->Render(),
                 tab_content->Render() | flex}) |
           borderRounded;
  });

  ScreenInteractive screen = ScreenInteractive::Fullscreen();

  /* main loop ---------------------------------------------------------------*/
  Loop loop(&screen, main_renderer);

  while (!loop.HasQuitted()) {
    // TODO: update display data
    screen.Post(Event::Custom);
    loop.RunOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
