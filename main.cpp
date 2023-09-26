#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include "vector.hpp"
#include "physics.hpp"
#include "sdl_init.hpp"

const int SCREEN_FPS = 60;
const int SCREEN_TICK_PER_FRAME = 1000/ SCREEN_FPS;

std::vector<Ball> balls = {};
std::vector<Edge> edges = {};
SDL_Texture* ball_texture;

    void
update(var_type duration){
    for(auto& ball : balls){
        ball.integrate(duration);
    }
    // serial collision.
    // resolve velocity, then position maybe
    //std::cout << "size: " << balls.size() << std::endl;
    for(int i = 0; i < balls.size(); i++){
        for(int j = i + 1; j < balls.size(); j++) {
            balls[i].collides_ball(balls[j]);          
        }
        for(int k = 0; k < edges.size(); k++) {
            balls[i].collides_edge(edges[k]);          
        }
    }
}

    void
render(){
    
    SDL_SetRenderDrawColor(gsdl.renderer, 0x00,0x00,0x18,SDL_ALPHA_OPAQUE);
    SDL_RenderClear(gsdl.renderer);
    SDL_FRect r;
    for(auto& ball : balls)
    {
        r.h = ball.radius * 2;
        r.w = r.h;
        r.x = ball.pos.x - ball.radius;
        r.y = ball.pos.y - ball.radius;
        SDL_RenderCopyF(gsdl.renderer, ball_texture, NULL, &r);
        //std::cout << "atleast one ball" << std::endl;
    }


    // render edges
    //for( int i = 0; i < g_edge_count; i++)
    //{
    //    SDL_SetRenderDrawColor(gsdl.renderer, 100, 100, 100, 50);
    //    //int test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].start.x, g_edges[i].start.y, g_edges[i].end.x, g_edges[i].end.y);
    //    int test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_0_start.x, 
    //        g_edges[i].line_0_start.y, g_edges[i].line_0_end.x, g_edges[i].line_0_end.y);
    //    test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_1_start.x, 
    //        g_edges[i].line_1_start.y, g_edges[i].line_1_end.x, g_edges[i].line_1_end.y);
    //    r.h = g_edges[i].radius * 2.0;
    //    r.w = r.h;
    //    r.x = g_edges[i].start.x - g_edges[i].radius;
    //    r.y = g_edges[i].start.y - g_edges[i].radius;
    //    SDL_RenderCopyF(gsdl.renderer, ball_texture, NULL, &r);
    //    r.x = g_edges[i].end.x - g_edges[i].radius;
    //    r.y = g_edges[i].end.y - g_edges[i].radius;
    //    SDL_RenderCopyF(gsdl.renderer, ball_texture, NULL, &r);

    //    if( test != 0 ) // something failed
    //    {
    //        //printf("Drawline failed! SDL_ERROR: %s\n", SDL_GetError());
    //        std::cout << "Drawline failed! SDL_ERROR: \n" << SDL_GetError() << std::endl;
    //    }
    //}
    
    // render edges
    for(Edge& edge : edges) {
        
        SDL_SetRenderDrawColor(gsdl.renderer, 100, 100, 100, 50);
        int test = SDL_RenderDrawLine(gsdl.renderer, edge.start.x, edge.start.y, edge.end.x, edge.end.y);
        //int test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_0_start.x, 
        //    g_edges[i].line_0_start.y, g_edges[i].line_0_end.x, g_edges[i].line_0_end.y);
        //test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_1_start.x, 
        //    g_edges[i].line_1_start.y, g_edges[i].line_1_end.x, g_edges[i].line_1_end.y);
    }

    // print total energy to screen
    static int skip_frames;
    skip_frames++;
    if(skip_frames % 15 == 0)  // make new surface/texture showing energy
    {
        char buffer[64];
        var_type energy_potential;
        var_type energy_kinematic; 
        var_type energy_total = 0;
        // sum up all energies of all particles
        //for( int i = 0; i < g_ball_count; i++)
        //{
        //    energy_kinematic = 0.5 * pow(Vec2_magnitude(&g_balls[i].vel), 2.0);
        //    energy_potential = g_balls[i].acc.y * (gsdl.window_height - g_balls[i].pos.y 
        //        - g_balls[i].radius);
        //    energy_total += energy_kinematic + energy_potential;
        //}
    }
    
    //SDL_RenderCopyF(gsdl.renderer, texture_text, NULL, &r);
    SDL_RenderPresent(gsdl.renderer);
}

int main() {
    std::cout << "hi" << std::endl;
    Vector2 a(1.0, 2.0);
    Vector2 b(10.0, 20.0);
    var_type scalar = 2.0;
    std::cout << "vector a: " << a.x << ", " << a.y << std::endl;
    std::cout << "vector b: " << b.x << ", " << b.y << std::endl;
    Vector2 c = a + b;
    std::cout << "a + b : " << c.x << ", " << c.y << std::endl;
    c = a - b;
    std::cout << "a - b : " << c.x << ", " << c.y << std::endl;
    //c = a * scalar; // reverse this?
    c = scalar * a;
    std::cout << c.x << ", " << c.y << std::endl;
    c = a.unit(); 
    std::cout << "a unit vector : " << c.x << ", " << c.y << std::endl;
    var_type magnitude = a.magnitude();
    std::cout << "a magnitude: " << magnitude << std::endl;
    c = a.perpendicular();
    std::cout << "a perpendicular vector : " << c.x << ", " << c.y << std::endl;
    
    // ball test
    //Ball ball(Vector2(3.0, 3.0), Vector2(1.0, 1.0), Vector2(-0.1, -0.1),
    //        1.0, 1.0, 1.0);
    //std::cout << "Ball position t = 0: " << ball.pos.x << ", " << ball.pos.y << std::endl;
    //ball.integrate(1.0);
    //std::cout << "Ball position t = 1.0: " << ball.pos.x << ", " << ball.pos.y << std::endl;
    //ball.integrate(1.0);
    //std::cout << "Ball position t = 2.0: " << ball.pos.x << ", " << ball.pos.y << std::endl;

    std::cout << "gsdl status: " << gsdl.created_ok << std::endl; 

    bool exit = false; 
    SDL_Event e;
    Uint64 time_frame_start;
    Uint64 time_frame_duration=0;
    
    // create some initial balls and edges
    const float pi = 3.14159;
    balls.push_back(Ball(Vector2(100, 100), Vector2(1.0, 1.0), Vector2(0,0), 8, 0.005, 1));
    balls.push_back(Ball(Vector2(400, 550), Vector2(-1.0, 1.5), Vector2(0,0), 8, 0.005, 1));
    edges.push_back(Edge(Vector2(1, 1), Vector2(800, 1)));  // top edge
    edges.push_back(Edge(Vector2(1, 599), Vector2(799, 599))); // bottom edge
    edges.push_back(Edge(Vector2(200, 300), Vector2(400, 320))); 
    edges.push_back(Edge(Vector2(799, 1), Vector2(799, 599))); //right edge
    edges.push_back(Edge(Vector2(1, 1), Vector2(1, 599))); // left edge
    edges.push_back(Edge(Vector2(500, 200), Vector2(200, 120))); 
    edges.push_back(Edge(Vector2(700, 100), Vector2(500, 420))); 
    SDL_Surface* image = IMG_Load("./ball.png");
    
    if(!image){
        std::cout << "Image not loaded..." << std::endl;
    }
    ball_texture = SDL_CreateTextureFromSurface(gsdl.renderer, image);
    
    while( !exit )
    {
        //start timer to keep track of time taken for this frame.
        time_frame_start = SDL_GetTicks64();

        //Handle all events on queue
        while( SDL_PollEvent( &e ) != 0 )
        {
            if( e.type == SDL_QUIT)
            {
                exit = true;
            }
            //if( e.type == SDL_MOUSEMOTION )
            //{
            //   mouse_x = e.motion.x;
            //   mouse_y = e.motion.y;
            //   //printf("%d, %d\n", mouse_x, mouse_y);
            //}
            if( e.type == SDL_MOUSEBUTTONDOWN ) 
            {
                if( e.button.button == SDL_BUTTON_LEFT)
                {
                }
                if( e.button.button == SDL_BUTTON_RIGHT)
                {
                    //g_ball_count++;
                    //int x, y;
                    //SDL_GetMouseState(&x, &y);
                    ////printf("Ball @ %d, %d\n", mouse_x, mouse_y);
                    //g_balls[g_ball_count - 1].pos.x = x;
                    //g_balls[g_ball_count - 1].pos.y = y;
                    //g_balls[g_ball_count - 1].vel.x = 0.0; //(float)(rand() % 40 - 20) * 0.3;
                    ////3.0 - g_ball_count;
                    //g_balls[g_ball_count - 1].vel.y = 0.0; //(float)(rand() % 40 - 20) * 0.3;
                    ////3.0 + g_ball_count;
                    //g_balls[g_ball_count - 1].acc.x = 0.0;
                    //g_balls[g_ball_count - 1].acc.y = 0.9;//0.25;     //0.8;
                    //float f = (float)(rand() % 20) / 2.0;
                    //g_balls[g_ball_count - 1].radius = 2.5 + f + 10.5;
                    //g_balls[g_ball_count - 1].mass_inverse = 1.0/(2.5 + f + 10.5);
                    ////0.5 * pow(Vec2_magnitude(&g_balls[0].vel), 2.0);
                    //var_type energy_k = 0.5 * pow(Vec2_magnitude(&g_balls[0].vel), 2.0);
                    //var_type energy_p = g_balls[g_ball_count - 1].acc.y * (gsdl.window_height - g_balls[g_ball_count - 1].pos.y - g_balls[g_ball_count - 1].radius);
                    //var_type energy_total = energy_k + energy_p;
                    //printf("Ball created with %f TE\n", energy_k + energy_p);
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    Ball ball;
                    ball.pos = Vector2(x, y);
                    ball.vel = Vector2((float)(rand() % 40 - 20) * 0.3, 
                        (float)(rand() % 40 - 20) * 0.3);
                    ball.acc = Vector2(0, 0);
                    ball.radius = 10.0 +  float(rand() % 10);
                    ball.mass_inverse = 1.0/ (3.14159 * pow(ball.radius, 2));
                    ball.elasticity = 1.0;
                    balls.push_back(ball);
                    std::cout << "ball created!\n";

                }
            }
            if ( e.type == SDL_KEYDOWN)
            {
                if(e.key.keysym.sym == SDLK_d)
                {
                    if (!balls.empty()) {
                        balls.pop_back(); 
                    }
                    //if(g_ball_count > 0)
                    //{
                    //    //g_ball_count--;
                    //    printf("Ball deleted.\n");
                    //    element_remove(g_balls, sizeof(Ball), g_ball_count - 1, &g_ball_count);
                    //    printf("g_ball_count = %d.\n", g_ball_count);
                    //}
                    //else
                    //{
                    //    printf("No more balls to delete.\n");
                    //}
                }
            } 
        }

        int num_steps_per_loop = 1;
        for( int i = 0; i < num_steps_per_loop; i++)
        {
            update(1.0/(float)num_steps_per_loop);
        }
        render();
        time_frame_duration = SDL_GetTicks64() - time_frame_start;
        if( time_frame_duration < SCREEN_TICK_PER_FRAME )
        {
            //Wait
            SDL_Delay( SCREEN_TICK_PER_FRAME - time_frame_duration );
        }
    }
    SDL_DestroyWindow(gsdl.window);
    TTF_CloseFont(gsdl.font);
    SDL_FreeSurface(image);
    SDL_DestroyTexture(ball_texture);
    IMG_Quit();
    SDL_Quit();
}

