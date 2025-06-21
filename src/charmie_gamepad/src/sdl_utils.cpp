#include "charmie_gamepad/sdl_utils.hpp"
#include <SDL2/SDL.h>
#include <iostream>

namespace sdl_utils
{

std::string get_controller_name()
{
  if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
    std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
    return "";
  }

  int num_joysticks = SDL_NumJoysticks();
  if (num_joysticks < 1) {
    SDL_Quit();
    return "No joystick connected";
  }

  const char *name = SDL_JoystickNameForIndex(0);  // Safe alternative
  std::string controller_name = name ? name : "Unknown controller";

  SDL_Quit();
  return controller_name;
}

}  // namespace sdl_utils
