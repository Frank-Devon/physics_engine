#ifndef SDL_INIT_HPP
#define SDL_INIT_HPP

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include "vector.hpp"
//#include "imgui.h"
//#include "imgui_impl_sdl2.h"
//#include "imgui_impl_sdlrenderer2.h"

class SDL_Main {
public:
    SDL_Window *window;
    SDL_Renderer *renderer;
    TTF_Font *font;
    var_type window_width;
    var_type window_height;
    bool created_ok;
    
    SDL_Main(int screen_wdith, int screen_height);
};

extern SDL_Main gsdl;

#endif



