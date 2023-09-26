#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include "vector.hpp"
#include "sdl_init.hpp"

SDL_Main gsdl(800, 600);

SDL_Main::SDL_Main(int screen_width, int screen_height) {
    window = nullptr;
    renderer = nullptr;
    font = nullptr;
    window_width = screen_width;
    window_height = screen_height;
    std::cout << "trying to initialize gsdl\n"; 
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        //printf("SDL failed to initialize. SDL Error: %s\n", SDL_GetError() );
        std::cout << "SDL failed to initialize. SDL Error: " << SDL_GetError();
        created_ok = false;
        return;
    }
    if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
    {
        printf("Linear texture filtering not enabled!" );
    }
    window = SDL_CreateWindow( "Physics simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, window_width, window_height, SDL_WINDOW_SHOWN );
    if( window == NULL)
    {
        printf("SDL_CreateWindow failed! SDL Error: %s\n", SDL_GetError() );
        created_ok = false;
        return;
    }
    renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
    if( renderer == NULL )
    {
        printf( "SDL_CreateRenderer failed! SDL Error: %s\n", SDL_GetError() );
        created_ok = false;
        return;
    }
    SDL_SetRenderDrawColor( renderer, 0xff, 0xff, 0xff, 0xff );
    //Initialize PNG loading
    int img_flags = IMG_INIT_PNG;
    if( !( IMG_Init( img_flags ) & img_flags ) )
    {
        printf(  "SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError() );
        created_ok = false;
        return;
    }
    if(TTF_Init() == -1)
    {
        printf("Couldn't initialize SDL2_ttf. Error: %s", TTF_GetError());
        created_ok = false;
        return;
    }
    //g_font = TTF_OpenFont("./fonts/8bitOperatorPlus8-Regular.ttf",32);
    font = TTF_OpenFont("./fonts/Justmore - Personal Use.ttf",32);
    if(font == NULL)
    {
        printf("Couldn't create font! SDL_Error: %s\n", SDL_GetError());
        created_ok = false;
        return;
    }
    std::cout << "SDL_Main object successfully created.\n";
}
