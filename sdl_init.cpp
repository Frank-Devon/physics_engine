#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include "vector.hpp"
#include "sdl_init.hpp"
//#include "imgui/imgui.h"
//#include "imgui_impl_sdl2.h"
//#include "imgui_impl_sdlrenderer2.h"

//SDL_Main gsdl(800, 600);
SDL_Main gsdl(1280, 720);

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
    //8bitOperatorPlus8-Regular
    //font = TTF_OpenFont("./fonts/Justmore - Personal Use.ttf",32);
    font = TTF_OpenFont("./fonts/8bitOperatorPlus8-Regular.ttf", 32);
    
    if(font == NULL)
    {
        printf("Couldn't create font! SDL_Error: %s\n", SDL_GetError());
        created_ok = false;
        return;
    }
    std::cout << "SDL_Main object successfully created.\n";
	
//	// Setup Dear ImGui context
//    IMGUI_CHECKVERSION();
//    ImGui::CreateContext();
//    //ImGuiIO* io = ImGui::GetIO(); //(void)io;
//	io = new ImGui::GetIO();
//    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
//    io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
//
//    // Setup Dear ImGui style
//    ImGui::StyleColorsDark();
//    //ImGui::StyleColorsLight();
//
//    // Setup Platform/Renderer backends
//    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
//    ImGui_ImplSDLRenderer2_Init(renderer);
//
//    // Load Fonts
//    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
//    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
//    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
//    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
//    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
//    // - Read 'docs/FONTS.md' for more instructions and details.
//    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
//    //io.Fonts->AddFontDefault();
//    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
//    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
//    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
//    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
//    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
//    //IM_ASSERT(font != nullptr);
//
//    // Our state
//    show_demo_window = true;
//    show_another_window = false;
//    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

}
