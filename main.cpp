#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <array>
#include <vector>
#include "vector.hpp"
#include "physics.hpp"
#include "sdl_init.hpp"

const int SCREEN_FPS = 30;
const int SCREEN_TICK_PER_FRAME = 1000/ SCREEN_FPS;

std::vector<Ball> balls = {};
std::vector<Edge> edges = {};
Ball* ball_mouseover = NULL;
Ball* ball_selected = NULL;
ParticleForceRegistry particle_force_registry;
SDL_Texture* ball_texture;
SDL_Texture* ball_mouseover_texture;
SDL_Texture* ball_selected_texture;
SDL_Texture* energy_readout_texture = NULL;  //texture_text = NULL;
Vector2 mouse_pos = Vector2(0, 0);

void
update(var_type duration)
{
    particle_force_registry.update_all(duration);
    for (auto& ball : balls) {
        ball.integrate(duration);
    }
    // if a ball is selected, make sure it moves with the cursor
    // better to do it here than checking each iteration of balls
    // TODO crash if ball draged into edge
    if (ball_selected) {
        ball_selected->pos = mouse_pos;
        ball_selected->vel = Vector2(0, 0);
    }
    // detect collisions with other balls and edges
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
render(var_type duration)
{
    SDL_SetRenderDrawColor(gsdl.renderer, 0x00,0x00,0x18,SDL_ALPHA_OPAQUE);
    SDL_RenderClear(gsdl.renderer);
    SDL_FRect r;
    for (auto& ball : balls) {
        r.h = ball.radius * 2;
        r.w = r.h;
        r.x = ball.pos.x - ball.radius;
        r.y = ball.pos.y - ball.radius;
        SDL_RenderCopyF(gsdl.renderer, ball_texture, NULL, &r);
        //std::cout << "atleast one ball" << std::endl;
    }
    // draw mouseover ball differently
    if (ball_mouseover) { 
        r.h = ball_mouseover->radius * 2;
        r.w = r.h;
        r.x = ball_mouseover->pos.x - ball_mouseover->radius;
        r.y = ball_mouseover->pos.y - ball_mouseover->radius;
        SDL_RenderCopyF(gsdl.renderer, ball_mouseover_texture, NULL, &r);
    }
    // draw selected ball differently
    if (ball_selected) { 
        r.h = ball_selected->radius * 2;
        r.w = r.h;
        r.x = ball_selected->pos.x - ball_selected->radius;
        r.y = ball_selected->pos.y - ball_selected->radius;
        SDL_RenderCopyF(gsdl.renderer, ball_selected_texture, NULL, &r);
    }
    // render edges
    for (Edge& edge : edges) {
        SDL_SetRenderDrawColor(gsdl.renderer, 100, 100, 100, 50);
        int test = SDL_RenderDrawLine(gsdl.renderer, edge.start.x, edge.start.y, edge.end.x, edge.end.y);
        //int test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_0_start.x, 
        //    g_edges[i].line_0_start.y, g_edges[i].line_0_end.x, g_edges[i].line_0_end.y);
        //test = SDL_RenderDrawLine(gsdl.renderer, g_edges[i].line_1_start.x, 
        //    g_edges[i].line_1_start.y, g_edges[i].line_1_end.x, g_edges[i].line_1_end.y);
    }

    //
    // render force generators in the "registry"
    //
    particle_force_registry.draw_all(duration);

    // print total energy to screen
    static int skip_frames;
    skip_frames++;
    if(skip_frames % 15 == 0) {  // make new surface/texture showing energy
        SDL_Surface* energy_readout_surface = NULL;
        char buffer[64];
        var_type energy_potential = 0.0;
        var_type energy_kinematic = 0.0; 
        var_type energy_total = 0.0;
        // new way of calculating energy
        for(Ball& ball : balls)
        {
            energy_kinematic += 0.5 * pow(ball.vel.magnitude() / duration, 2.0);  
        }
        
        //var_type spring_length = (anchor - balls[0].pos).magnitude();
        //var_type spring_displacement = spring_length - 250.0;
        //energy_potential = 0.5 * 40.0 * spring_displacement * spring_displacement;
        //energy_total = energy_kinematic + energy_potential;
        //// old way
        // sum up all energies of all particles
        //for( int i = 0; i < g_ball_count; i++)
        //{
        //    energy_kinematic = 0.5 * pow(Vec2_magnitude(&g_balls[i].vel), 2.0);
        //    energy_potential = g_balls[i].acc.y * (gsdl.window_height - g_balls[i].pos.y 
        //        - g_balls[i].radius);
        //    energy_total += energy_kinematic + energy_potential;
        //}
        //int ret = snprintf(buffer, sizeof buffer, "%f J", energy_total);
        int ret = snprintf(buffer, sizeof buffer, "%E J: %E, %E", energy_total, energy_kinematic, energy_potential);
        if (energy_readout_texture != NULL)
        { SDL_DestroyTexture(energy_readout_texture); }
        energy_readout_surface = TTF_RenderUTF8_Solid(gsdl.font, buffer, {0, 0, 100});
        energy_readout_texture = SDL_CreateTextureFromSurface(gsdl.renderer, energy_readout_surface);
        SDL_FreeSurface(energy_readout_surface);
        skip_frames = 0;
    }
    
    r.x = 10.0; r.y = 10.0; r.w = 550.0; r.h = 80.0;
    //SDL_RenderCopyF(gsdl.renderer, energy_readout_texture, NULL, &r);
    SDL_RenderPresent(gsdl.renderer);
}

int main()
{
    std::cout << "gsdl status: " << gsdl.created_ok << std::endl; 
    // testing array type
    std::array<int, 3> test{};
    int index = 0;
    test[index++] = 7;
    test[index++] = 10;
    for (const auto& e : test) {
        std::cout << e << std::endl;
    }

    bool exit = false; 
    SDL_Event e;
    Uint64 time_frame_start;
    Uint64 time_frame_duration=0;
    
    // create some initial balls and edges
    const float pi = 3.14159;
    //balls.push_back(Ball(Vector2(100, 100), Vector2(1.0, 1.0), Vector2(0,0), 8, 0.005, 1));
    // this ball collides with edge
    //balls.push_back(Ball(Vector2(180, 300), Vector2(1.0, 0.1), Vector2(0,0), 5, 0.005, 1));
    //// these 2 balls hit each other off center
    //balls.push_back(Ball(Vector2(480, 240), Vector2(0.0, 0), Vector2(0,0), 15, 0.05, 1));
    //balls.push_back(Ball(Vector2(472, 300), Vector2(0.0, -2.0), Vector2(0,0), 15, 0.005, 1));
   
    // TODO make balls an array
    balls.reserve(100);
    // testing springs 
    balls.push_back(Ball(Vector2(200, 450), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    balls.push_back(Ball(Vector2(600, 150), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    //balls.push_back(Ball(Vector2(450, 300), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    // border edges
    edges.push_back(Edge(Vector2(1, 1), Vector2(800, 1)));  // top edge
    edges.push_back(Edge(Vector2(1, 599), Vector2(799, 599))); // bottom edge
    //edges.push_back(Edge(Vector2(200, 300), Vector2(400, 320))); 
    edges.push_back(Edge(Vector2(799, 1), Vector2(799, 599))); //right edge
    edges.push_back(Edge(Vector2(1, 1), Vector2(1, 599))); // left edge
    
    //// edges in the middle
    // TODO causes seg fault. Balls starting out on top of the this edge causes the seg fault?
    //edges.push_back(Edge(Vector2(200, 300), Vector2(400, 300)));
    edges.push_back(Edge(Vector2(500, 200), Vector2(200, 120))); 
    //edges.push_back(Edge(Vector2(700, 100), Vector2(500, 420))); 
    //// force generators ( springs)
    //Spring spring_a(&balls[1], 200.0, 40.0);//0.0005);
    //Spring spring_b(&balls[0], 200.0, 40.0);//0.0005);
    // force generators ( anchored springs)
    Vector2 anchor0(425, 325);
    Vector2 anchor1(150, 300);
    SpringAnchoredForceGenerator spring0(&anchor1, 250.0, 20.0);
    SpringAnchoredForceGenerator spring1(&anchor0, 250.0, 20.0);
    particle_force_registry.add(&balls[0], &spring0);
    particle_force_registry.add(&balls[0], &spring1);
    // add bungee
    Vector2 anchor2(400, 200);
    BungieForceGenerator bungie(&anchor2, 250.0, 28.0);
    particle_force_registry.add(&balls[1], &bungie);
   
    // Bungie bungie(anchor_point, rest_length, spr_constant, force_registry, ball)
    // AncSpring as(anchor_point, rest_length, spr_constant, force_registry, ball)
    // Spring spring(ball0, ball1 rest_length, spr_constant, force_registry)
    

    GravityForceGenerator gravity_force_generator(Vector2(0.0, 250.1));
    //particle_force_registry.add(&balls[0], &gravity_force_generator);
    particle_force_registry.add(&balls[1], &gravity_force_generator);


    //// register force generators to particles/balls
    //particle_force_registry.add(&balls[0], &spring_a);
    //particle_force_registry.add(&balls[1], &spring_b);

    SDL_Surface* image = IMG_Load("./ball.png");
    SDL_Surface* ball_selected_surface = IMG_Load("./ball_select.png");
    SDL_Surface* ball_mouseover_surface = IMG_Load("./ball_hover.png");
    
    if(!image || !ball_selected_surface || ! ball_mouseover_surface){
        std::cout << "Image not loaded..." << std::endl;
    }
    ball_texture = SDL_CreateTextureFromSurface(gsdl.renderer, image);
    ball_selected_texture = SDL_CreateTextureFromSurface(gsdl.renderer, ball_selected_surface);
    ball_mouseover_texture = SDL_CreateTextureFromSurface(gsdl.renderer, ball_mouseover_surface);
    
    SDL_FreeSurface(image);
    SDL_FreeSurface(ball_selected_surface);
    SDL_FreeSurface(ball_mouseover_surface);
    while (!exit) {
        //start timer to keep track of time taken for this frame.
        time_frame_start = SDL_GetTicks64();
        int x, y; // mouse position 
        //Handle all events on queue
        while (SDL_PollEvent( &e ) != 0) {
            SDL_GetMouseState(&x, &y);
            mouse_pos = Vector2(x, y);
            if (e.type == SDL_QUIT) exit = true;
            if (e.type == SDL_MOUSEBUTTONDOWN) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    ball_selected = NULL;
                    for (auto& ball : balls) {
                        Vector2 dist_vector = ball.pos - mouse_pos;
                        var_type dist = dist_vector.magnitude(); 
                        if (dist < ball.radius) ball_selected = &ball;
                    }
                }
                if (e.button.button == SDL_BUTTON_RIGHT) {
                    Ball ball;
                    ball.pos = mouse_pos;
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
            if (e.type == SDL_MOUSEBUTTONUP) {
                if (e.button.button == SDL_BUTTON_LEFT) {
                    std::cout << "lmouse up\n";
                    ball_selected = NULL;
                }
            }
            if (e.type == SDL_KEYDOWN) {
                if(e.key.keysym.sym == SDLK_d) {
                    if (!balls.empty()) {
                        balls.pop_back(); 
                    }
                }
            } 
        }

        // check if mouse is over any balls
        ball_mouseover = NULL;  // clear previous selection
        for (auto& ball : balls) {
            Vector2 dist_vector = ball.pos - mouse_pos;
            var_type dist = dist_vector.magnitude(); 
            if (dist < ball.radius) {
                ball_mouseover = &ball; 
            }
        }
        // update state of the simulation. Using 'mini-steps' for better resolution.
        int num_steps_per_loop = 20;
        var_type duration = 1.0/(float)num_steps_per_loop;
        for (int i = 0; i < num_steps_per_loop; i++) {
            //TODO ball-ball collisions cause change in energy
            update(duration);
        }
        // render the simulation
        render(duration * num_steps_per_loop);
        // ensure frame rate is steady
        //printf("ball[1] velocity: %f, %f\n", balls[1].vel.x, balls[1].vel.y);
        time_frame_duration = SDL_GetTicks64() - time_frame_start;
        if (time_frame_duration < SCREEN_TICK_PER_FRAME) {
            //Wait
            SDL_Delay( SCREEN_TICK_PER_FRAME - time_frame_duration );
        }
    }
    // clean up
    SDL_DestroyWindow(gsdl.window);
    TTF_CloseFont(gsdl.font);
    SDL_DestroyTexture(ball_texture);
    SDL_DestroyTexture(ball_selected_texture);
    SDL_DestroyTexture(energy_readout_texture);
    IMG_Quit();
    SDL_Quit();
}

