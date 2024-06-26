#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <array>
#include <vector>
#include "vector.hpp"
#include "physics.hpp"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "sdl_init.hpp"
#include "space_partition.hpp"
#include "draw.hpp"
  
const int SCREEN_FPS = 60;
const int SCREEN_TICK_PER_FRAME = 1000 / SCREEN_FPS;

enum class RunMode { Run, Pause, RunNumFrames };
//const int BALLPTR_GRID_X_MAX = 20;
//const int BALLPTR_GRID_Y_MAX = 20; 
//using ballptr_array = std::array<std::array<std::array<Ball*, 30>, 20>, 20>; 

//static int num_hard_constraints = 0;

// variables that are the most important for the simulation
std::vector<Ball<var_type>> balls = {};
std::vector<Edge<var_type>> edges = {};
std::vector<Contact<var_type>> contacts = {};  
std::vector<ContactGenerator<var_type>*> contact_generators = {};
ParticleForceRegistry particle_force_registry;
ContactResolver contact_resolver(100);
SpacePartition<var_type> space_partition(balls, 800.0, 600.0);

// variables that help handle user input
var_type ball_selected_mass_inverse = 0.000;  // when ball selected, it becomes immovable
Vector2 ball_selected_acc = Vector2(0.0f, 0.0f);
Vector2 ball_selected_mouse_to_center = Vector2(0.0f, 0.0f);

// variables dealing with display
Ball<var_type>* ball_mouseover = nullptr;
Ball<var_type>* ball_selected = nullptr;
Ball<var_type>* ball_tracked = nullptr;
SDL_Texture* ball_texture;
SDL_Texture* ball_mouseover_texture;
SDL_Texture* ball_selected_texture;
SDL_Texture* energy_readout_texture = NULL;  //texture_text = NULL;
Vector2 mouse_pos = Vector2(0, 0);
Vector2 mouse_pos_float = Vector2<float>(0, 0);
Vector2 mouse_pos_old = Vector2(0, 0);
int display_contact_index = 0;

// ball emitter
var_type emitter_time = 00.0f;
var_type emitter_time_per_ball = 0.025;
var_type emitter_time_current = 0.05f;  // decremented every frame by amount of time passed. 
                                        // when less than 0, emit ball. reset this variable to
                                        // emitter_time_per_ball

// configure physics simulation
var_type restitution_ball_edge = 1.0f;
var_type restitution_ball_ball = 1.0f;
var_type gravity = 400.0f;//0.050f;
var_type dampening_vel = 1.0f;  //0.99998f;
var_type new_ball_avg_size = 10.0;
var_type ball_new_min_radius = 6.0;
var_type ball_new_max_radius = 16.0;
var_type ball_new_fixed_radius = 15.0;
bool ball_fixed_radius = true;
bool ball_new_speed_zero = false;
RunMode run_mode = RunMode::Run;
int num_frames = 0;  // if run_mode == RunNumFrames, then execute this # of frames till 0, then pause
int num_frames_ui = 30;  // sets num_frames when RunNumFrames entered.
int collision_recheck_count = 2;
int num_steps_per_loop = 40;

// commands. bools checked every frame, if true, do some job, then turn the flag off automatically
bool command_gravity = false;  // accel of every particle set to global gravity variable
bool command_restitution_ball_ball = false;
bool command_restitution_ball_edge = false;
bool command_dampening_vel = false;
bool command_set_frames_to_simulate = false;



//// imgui
//// Setup Dear ImGui context
//IMGUI_CHECKVERSION();
//ImGui::CreateContext();
//ImGuiIO& io = ImGui::GetIO(); (void)io;
//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
//
//// Setup Dear ImGui style
//ImGui::StyleColorsDark();
////ImGui::StyleColorsLight();
//
//// Setup Platform/Renderer backends
//ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
//ImGui_ImplSDLRenderer2_Init(renderer);
// Our state

bool show_demo_window = true;
bool show_another_window = false;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

double
kinetic_energy_calculate()
{
    double energy = 0.0;
    for (auto& ball : balls) {
       energy += 0.5 * (1.0/ball.mass_inverse) * pow(ball.vel.magnitude(), 2.0); 
    }
    return energy;
}
double
potential_energy_calculate()
{
    double energy = 0.0;
    for (auto& ball : balls) {
       energy += (1.0/ball.mass_inverse) * gravity * (599.0 - (ball.pos.y + ball.radius));
    }
    return energy;
}

double
total_energy_calculate()
{
    return potential_energy_calculate() + kinetic_energy_calculate();
}

void erase_scene() 
{
    balls.clear();
    edges.clear();
    contacts.clear();
    emitter_time = 0;
    for (ContactGenerator<var_type>* cg : contact_generators) delete cg;
    contact_generators.clear();
    particle_force_registry.clear();
}

void
update(var_type duration)
{
    for (auto& ball : balls) ball.force_accumulator = Vector2<var_type>();
    particle_force_registry.update_all(duration);
    // integrate balls with chosen integration method 
    switch (Ball<var_type>::integration_method) {
        case Ball<var_type>::IntegrationMethod::explicit_euler:
            for (auto& ball : balls) 
            { if (&ball != ball_selected) ball.integrate_explicit_euler(duration); }
            break;
        case Ball<var_type>::IntegrationMethod::implicit_euler:
            for (auto& ball : balls) 
            { if (&ball != ball_selected) ball.integrate_implicit_euler(duration); }
            break;
        case Ball<var_type>::IntegrationMethod::explicit_midpoint:
            for (auto& ball : balls) 
            { if (&ball != ball_selected) ball.integrate_explicit_midpoint(duration); }
            break;
    }
    
    // if a ball is selected, make sure it moves with the cursor
    // better to do it here than checking each iteration of balls
    if (ball_selected) {
        // the selected ball get's reset here (integration is overwritten).
        ball_selected->pos_old = ball_selected->pos;
        ball_selected->pos = Vector2<var_type>(mouse_pos.x, mouse_pos.y) + ball_selected_mouse_to_center;
        ball_selected->vel = (ball_selected->pos - ball_selected->pos_old);//  / duration;
        // TODO should acceleration be calculated?
    }
    for (int i = 0; i < collision_recheck_count; i++) {
        contacts.clear(); // clear contacts... might not be necessary
        // detect collisions with other balls and edges
        space_partition.update();
        // TODO is this causing more contacts than necessary?
        for (Ball<var_type>& ball : balls) {
            auto& balls_nearby = space_partition.get_nearby_balls(ball.pos);
            for (Ball<var_type>* ball_nearby : balls_nearby) {
                if (ball_nearby == nullptr) break;
                //if (ball_nearby->pos == ball.pos) continue;
                if (ball_nearby <= &ball) continue;
                Vector2 pos_relative = ball.pos - ball_nearby->pos; 
                var_type penetration = ball.radius + ball_nearby->radius - pos_relative.magnitude();
                if (penetration > 0.0) {
                    Contact<var_type> c;
                    c.ball[0] = &ball;
                    c.ball[1] = ball_nearby;
                    c.penetration = penetration;
                    c.restitution = restitution_ball_ball;
                    c.contact_normal = (c.ball[0]->pos - c.ball[1]->pos).unit();
                    c.movement[0] =  Vector2();
                    c.movement[1] =  Vector2();
                    c.seperating_speed = c.seperating_speed_calculate();
                    contacts.emplace_back(c);
                }
            }
        }
        for (size_t i = 0; i < balls.size(); i++) {
            for (unsigned int k = 0; k < edges.size(); k++) {
                Edge edge = edges[k];
                Vector2 PS = balls[i].pos - edge.start;   
                Vector2 SE = edge.end - edge.start; 
                // will find the SE.x, SE.SE.x, SE.y point on the edge to the ball
                var_type dot_product = PS.dot(SE);  
                var_type SE_magnitude = SE.magnitude();  
                // normalize t between 0 and 1
                var_type t = fmax(0.0, fmin(SE_magnitude, dot_product / SE_magnitude));
                t = t / SE_magnitude;
                Vector2 closest_point = t * SE; 
                closest_point = closest_point + edge.start;  
                //printf("closest circ = %f, %f", closest_point.x, closest_point.y);
                Vector2 penetration_vector = closest_point - balls[i].pos;
                var_type penetration = balls[i].radius - penetration_vector.magnitude();
                if (penetration > 0.0) {
                    Contact<var_type> c;
                    c.ball[0] = &balls[i];
                    c.ball[1] = NULL;
                    c.penetration = penetration;
                    c.restitution = restitution_ball_edge;
                    c.contact_normal = (c.ball[0]->pos - closest_point).unit();
                    c.movement[0] =  Vector2();
                    c.movement[1] =  Vector2();
                    c.seperating_speed = c.seperating_speed_calculate();
                    contacts.emplace_back(c);
                }
            }
        } 
        
        // get additional contacts from contact_generators (rods and cables)
        // TODO: should only use on Anchor style contact generators?
        for (ContactGenerator<var_type>* cg : contact_generators) {
            if ( cg->disable_generation(ball_selected) ) {
                // contacts from constraints violated by user movement causes resolver to freeze
                continue; 
            }

            cg->generate_contact(contacts, 10);  // second parameter doesn't matter
        }
        // now, finally resolve collisions
        if (contacts.size() > 0) {
            contact_resolver.resolve_contacts(contacts, duration);
        } 
    }
}

void
render(ImGuiIO& io)
{
    //SDL_SetRenderDrawColor(gsdl.renderer, 0x00,0x00,0x1F, SDL_ALPHA_OPAQUE);
    SDL_SetRenderDrawColor(gsdl.renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
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
    // draw marker on first ball
    if (balls.size() > 0) {
        //std::cout << "hi\n";
        SDL_SetRenderDrawColor(gsdl.renderer, 50,150,100,100);
        SDL_Rect z;
        z.h = 8; z.w = 8;
        z.x = (int)balls[0].pos.x - 4; z.y = (int)balls[0].pos.y - 4;
        SDL_RenderFillRect(gsdl.renderer, &z);
    }
    // draw marker on tracked ball 
    if (ball_tracked) { 
        //r.h = ball_tracked->radius * 2;
        //r.w = r.h;
        //r.x = ball_tracked->pos.x - ball_tracked->radius;
        //r.y = ball_tracked->pos.y - ball_tracked->radius;
        //SDL_RenderCopyF(gsdl.renderer, ball_selected_texture, NULL, &r);
        SDL_SetRenderDrawColor(gsdl.renderer, 150, 150, 50, 100);
        SDL_Rect z;
        z.h = 8; z.w = 8;
        z.x = (int)ball_tracked->pos.x - 4; z.y = (int)ball_tracked->pos.y - 4;
        SDL_RenderFillRect(gsdl.renderer, &z);
    }
    // render edges
    for (Edge<var_type>& edge : edges) {
        SDL_SetRenderDrawColor(gsdl.renderer, 0, 0, 0, 0);
        SDL_RenderDrawLine(gsdl.renderer, edge.start.x, edge.start.y, edge.end.x, edge.end.y);
    }
    
    particle_force_registry.draw_all();

    for (ContactGenerator<var_type>* contact_gen : contact_generators) {
        contact_gen->draw();        
    }

    // print total energy to screen
    static int skip_frames;
    static int ui_num_chars;
    skip_frames++;
    if(skip_frames % 5 == 0) {  // make new surface/texture showing energy
        SDL_Surface* energy_readout_surface = NULL;
        char buffer[128];

        //get out of bounds balls
        int balls_out_of_bounds = 0;
        for (Ball<var_type>& ball : balls) {
            if (ball.pos.x < 0.0 || ball.pos.x > 800.0 || ball.pos.y < 0.0 
                    || ball.pos.y > 600.0) {
                balls_out_of_bounds++;
            }
        }
        snprintf(buffer, sizeof buffer,"RB %.2f RE %.2f Grv %.2f Dmp %.2f B# %ld BM# %d",
            restitution_ball_ball, restitution_ball_edge, gravity, dampening_vel, 
            balls.size(), balls_out_of_bounds);
        ui_num_chars = strlen(buffer);
        if (energy_readout_texture != NULL)
        { SDL_DestroyTexture(energy_readout_texture); }
        energy_readout_surface = TTF_RenderUTF8_Solid(gsdl.font, buffer, {50, 0, 100});
        energy_readout_texture = SDL_CreateTextureFromSurface(gsdl.renderer, energy_readout_surface);
        SDL_FreeSurface(energy_readout_surface);
        skip_frames = 0;
    }
    
    //r.x = 10.0; r.y = 10.0; r.w = 550.0; r.h = 32.0;
    r.x = 10.0; r.y = 610.0; r.w = ui_num_chars * 16.0; r.h = 16.0;
    SDL_RenderCopyF(gsdl.renderer, energy_readout_texture, NULL, &r);


	// Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
    {
        //static float f = 0.0f;
        //static int counter = 0;
        static int timer_show_contact_count = 10;

        ImGui::Begin("Control Panel");                          // Create a window called "Hello, world!" and append into it.

        //ImGui::Text("Select demo (press 1-5)");
        //if (ImGui::Button("main demo")); //set_up_scene();
        //ImGui::SameLine();
        //if (ImGui::Button("simple rod"));// set_up_scene_rod_test();
        //ImGui::SameLine();
        //if (ImGui::Button("springs and bungees"));// set_up_scene_springs_bungees();
        ////ImGui::SameLine();
        //if (ImGui::Button("newton's cradle")); //set_up_scene();
        //ImGui::SameLine();
        //if (ImGui::Button("crate"));// set_up_scene_rod_test();
        ////ImGui::SameLine();
        ////if (ImGui::Button("springs and bungees"));// set_up_scene_springs_bungees();

        //ImGui::Text(" ");
        //if (ImGui::Button("delete balls"));// set_up_scene_rod_test();

        //static int contacts_count = 0;
        if (timer_show_contact_count <= 0) {
            timer_show_contact_count = 10;
            //contacts_count = contacts.size();
        }
        //if (contacts.size() > 0 && contacts[0].ball[1]) {
        if (false) {
            ImGui::Text("contacts[0].contact_normal = %f, %f", contacts[0].contact_normal.x, 
                    contacts[0].contact_normal.y);
            ImGui::Text("contacts[0].ball[0].pos = %f, %f", contacts[0].ball[0]->pos.x, 
                    contacts[0].ball[0]->pos.y);
            ImGui::Text("contacts[0].ball[1].pos = %f, %f", contacts[0].ball[1]->pos.x, 
                    contacts[0].ball[1]->pos.y);
            ImGui::Text("contacts[0] distance balls = %f", distance(contacts[0].ball[0]->pos,
                    contacts[0].ball[1]->pos));
            ImGui::Text("Penetration: %f", contacts[0].penetration);            
        }
        
        //ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
        //ImGui::Checkbox("Another Window", &show_another_window);
//collision_recheck_count
        
        if (run_mode == RunMode::Pause) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.7f, 0.4f, 1.0f));
            if (ImGui::Button(" Run ")) {
               run_mode = RunMode::Run;
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
        } else if(run_mode == RunMode::Run) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.4f, 0.4f, 1.0f));
            if (ImGui::Button("Pause")) {
               run_mode = RunMode::Pause;
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
        } else {  /* run_mode == RunMode::RunNumFrames */
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.4f, 0.7f, 1.0f));
            if (ImGui::Button(" Run ")) {
                run_mode = RunMode::Run;
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        ImGui::InputInt("# Frames", &num_frames_ui, 1, 300);
        ImGui::SameLine();
        if (ImGui::Button("Run # Frames")) {
            run_mode = RunMode::RunNumFrames;
            num_frames = num_frames_ui;
        }


        ImGui::Text("   PRESS 1 - 5 TO SELECT DEMO   ");            
        //ImGui::Text("Lower Collision Re-Check to improve speed,\nat the cost of stability");
        ImGui::SliderInt("Collision Re-Check #", &collision_recheck_count, 1, 6);
        //ImGui::SliderFloat("float", &f, -1.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::SliderInt("physics updates per frame", &num_steps_per_loop, 1, 200);
        ImGui::SliderInt("Max Iterations over contact list", 
                &contact_resolver.iterate_over_list_count, 1, 20);
        switch (Ball<var_type>::integration_method) {
            case Ball<var_type>::IntegrationMethod::explicit_euler:
                ImGui::Text("Method: Explicit Euler");
                break;
            case Ball<var_type>::IntegrationMethod::implicit_euler:
                ImGui::Text("Method: Implicit Euler");
                break;
            case Ball<var_type>::IntegrationMethod::explicit_midpoint:
                ImGui::Text("Method: Explicit Midpoint");
                break;
        }
        ImGui::SameLine();
        if (ImGui::Button("Explicit Euler")) {
            Ball<var_type>::integration_method = Ball<var_type>::IntegrationMethod::explicit_euler;
        }
        ImGui::SameLine();
        if (ImGui::Button("Implicit Euler")) {
            Ball<var_type>::integration_method = Ball<var_type>::IntegrationMethod::implicit_euler;
        }
        //ImGui::SameLine();
        //if (ImGui::Button("Explicit Midpoint")) {
        //    Ball::integration_method = Ball::IntegrationMethod::explicit_midpoint;
        //}
        ImGui::Text("New ball properties");
        ImGui::SameLine();
        ImGui::Checkbox("fixed radius ", &ball_fixed_radius); 
        ImGui::SameLine();
        ImGui::Checkbox("initial speed = 0", &ball_new_speed_zero);
        //static float ball_new_fixed_radius_imgui = (float)ball_new_fixed_radius;
        ImGui::SliderFloat("fixed radius", &ball_new_fixed_radius, 9.0, 20.0);

        ImGui::SliderFloat("min radius", &ball_new_min_radius, 2.0, 9.0);
        ImGui::SliderFloat("max radius", &ball_new_max_radius, 12.0, 20.0);
        ImGui::Text(" ");
        ImGui::SliderFloat("gravity", &gravity, -400.0f, 400.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::SameLine();
        if (ImGui::Button("Zero G.")) {
            gravity = 0.0;
        }
        ImGui::Text("Coefficient of Restitution");
        ImGui::SliderFloat("ball to ball", &restitution_ball_ball, 0.0f, 1.0f); 
        ImGui::SliderFloat("ball to edge", &restitution_ball_edge, 0.0f, 1.0f);
        ImGui::Text(" ");
        ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

        //if (ImGui::Button("Button123")) // Buttons return true when clicked (most widgets return true when edited/activated)
        //    counter++;
        //ImGui::SameLine();
        //ImGui::Text("counter = %d", counter);
        ImGui::Text("Contact count = %d", (int)contacts.size());
        ImGui::Text("Edges count = %d", (int)edges.size());
        //ImGui::Text("Constraints count = %d", (int)num_hard_constraints);
        
        //if (balls.size() > 0) {
        //    ImGui::Text("balls[0].pos = %f, %f", balls[0].pos.x, balls[0].pos.y);
        //    ImGui::Text("balls[0].vel = %f, %f", balls[0].vel.x, balls[0].vel.y);
        //    ImGui::Text("balls[0].speed = %f", balls[0].vel.magnitude());
        //    ImGui::Text("balls[0].acc = %f, %f", balls[0].acc.x, balls[0].acc.y);

        //    ImGui::Text("balls[0] mass = %f", 1.0/balls[0].mass_inverse);

        //}
        static bool ball_tracked_draw_contacts = true;
        //static bool ball_tracked_list_contacts = true;
        static std::vector<Contact<var_type>*> ball_tracked_contacts = std::vector<Contact<var_type>*>();
        if (ball_tracked) {
            ball_tracked_contacts.reserve(30);
            ball_tracked_contacts.clear();
            ImGui::Text("Tracked ball");
            ImGui::Text("pos = %f, %f", ball_tracked->pos.x, ball_tracked->pos.y);
            ImGui::Text("vel = %f, %f", ball_tracked->vel.x, ball_tracked->vel.y);
            ImGui::Text("speed = %f", ball_tracked->vel.magnitude());
            ImGui::Text("acc = %f, %f", ball_tracked->acc.x, ball_tracked->acc.y);
            ImGui::Text("mass = %f", 1.0/ball_tracked->mass_inverse);
            ImGui::Text("force accum = %f, %f", ball_tracked->force_accumulator.x, 
                ball_tracked->force_accumulator.y);
            //find index of tracked ball
            int index_of_tracked_ball = 0;
            for (auto& ball : balls) {
               if (&ball == ball_tracked) {
                    break;
               }
               index_of_tracked_ball++;
            }
            ImGui::Text("index of tracked ball = %d", index_of_tracked_ball);
        
            if (ball_tracked_draw_contacts) {
                for (Contact<var_type>& contact : contacts) {
                    if (contact.ball[0]  == ball_tracked || contact.ball[1]  == ball_tracked ) {
                       ball_tracked_contacts.push_back(&contact); 
                    }
                }
                Contact<var_type>* contact_selected = nullptr;
                if ( display_contact_index < (int)ball_tracked_contacts.size() ) {
                    contact_selected = ball_tracked_contacts[display_contact_index];
                }
                ImGui::Text("num contacts containing ball = %ld", ball_tracked_contacts.size());
                for (Contact<var_type>* contact : ball_tracked_contacts) {
                    if ( contact == contact_selected ) {

                        SDL_SetRenderDrawColor(gsdl.renderer, 0, 100, 100, 0);
                    } else {

                        SDL_SetRenderDrawColor(gsdl.renderer, 0, 0, 0, 0);
                    }
                    draw_arrow<float>(gsdl.renderer,
                        contact->ball[0]->pos + 40.f * contact->contact_normal,
                        contact->ball[0]->pos, 
                        3);
                }

                if (contact_selected) {
                    ImGui::Text("Ball Contact list, contact # %d", display_contact_index);
                    ImGui::Text("contact normal = %f, %f", contact_selected->contact_normal.x,
                            contact_selected->contact_normal.y);
                    ImGui::Text("seperating speed saved = %f", contact_selected->seperating_speed);
                    ImGui::Text("seperating speed newly calc = %f", contact_selected->seperating_speed_calculate());
                    ImGui::Text("penetration = %f", contact_selected->penetration);
                }
            }
        }
        
        SDL_SetRenderDrawColor(gsdl.renderer, 0, 0, 0, 0);
        //            SDL_SetRenderDrawColor(gsdl.renderer, 0, 0, 0, 0);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(300, 300),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(200, 300),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(100, 300),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(100, 200),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(100, 100),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(200, 100),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(300, 100),
        //                Vector2(200, 200), 3);
        //            draw_arrow(gsdl.renderer,
        //                Vector2(300, 200),
        //                Vector2(200, 200), 3);

        //ImGui::Text("Potential Energy = %f", potential_energy_calculate());
        //ImGui::Text("Kinetic Energy = %f", kinetic_energy_calculate());
        //ImGui::Text("Total Energy = %f", total_energy_calculate());

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::End();
    }

    // 3. Show another simple window.
    if (show_another_window)
    {
        ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Text("Hello from another window!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;
        ImGui::End();
    }

    // Rendering
    ImGui::Render();
    SDL_RenderSetScale(gsdl.renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
    //SDL_SetRenderDrawColor(renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
    //SDL_RenderClear(gsdl.renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
    //SDL_RenderPresent(renderer);

    SDL_RenderPresent(gsdl.renderer);
}

void emit_ball(Vector2<var_type> mouse_pos) {
    Ball<var_type> ball{};
    ball.pos = Vector2<var_type>(mouse_pos.x, mouse_pos.y);
    ball.vel = !ball_new_speed_zero ? Vector2<var_type>((var_type)(rand() % 40 - 20) * 0.3, 
        (var_type)(rand() % 40 - 20) * 0.3) : Vector2<var_type>(0.0, 0.0);
    //ball.acc = Vector2(0, gravity);
    ball.acc = Vector2<var_type>(0.0, 0.0 ); // TODO, set initial acceleration to something else?
    ball.radius = ball_fixed_radius ? ball_new_fixed_radius : ball_new_min_radius 
        + float(rand() % (int)(ball_new_max_radius - ball_new_min_radius));
    ball.mass_inverse = 1.0/ (3.14159 * pow(ball.radius, 2));
    ball.elasticity = 1.0;
    std::cout << "ball pos " << ball.pos.x << ", " << ball.pos.y << std::endl;
    std::cout << "ball vel " << ball.vel.x << ", " << ball.vel.y << std::endl;
    std::cout << "ball acc " << ball.acc.x << ", " << ball.acc.y << std::endl;
    balls.push_back(ball);
}

void set_up_scene_springs_bungees(){
    erase_scene();
    // testing springs 
    balls.push_back(Ball<var_type>(Vector2(200.f, 450.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    balls.push_back(Ball<var_type>(Vector2(500.f, 350.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    balls.push_back(Ball<var_type>(Vector2(250.f, 450.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    balls.push_back(Ball<var_type>(Vector2(530.f, 350.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    balls.push_back(Ball<var_type>(Vector2(550.f, 150.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    balls.push_back(Ball<var_type>(Vector2(510.f, 150.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15, 0.001f, 1));
    // these two balls hit each other (2d collision test)
    //balls.push_back(Ball(Vector2(650, 310), Vector2(0.0, -0.6), Vector2(0,0), 15, 0.001, 1));
    //balls.push_back(Ball(Vector2(648, 240), Vector2(0.0, 0.6), Vector2(0,0), 15, 0.001, 1));
    // border edges
    edges.push_back(Edge<var_type>(Vector2(1.f, 1.f), Vector2(800.f, 1.f)));  // top edge
    edges.push_back(Edge<var_type>(Vector2(1.f, 599.f), Vector2(799.f, 599.f))); // bottom edge
    edges.push_back(Edge<var_type>(Vector2(799.f, 1.f), Vector2(799.f, 599.f))); //right edge
    edges.push_back(Edge<var_type>(Vector2(1.f, 1.f), Vector2(1.f, 599.f))); // left edge
    
    // TODO causes seg fault. Balls starting out on top of the this edge causes the seg fault?
    //edges.push_back(Edge(Vector2(200, 300), Vector2(400, 300)));
    edges.push_back(Edge(Vector2(500.f, 200.f), Vector2(200.f, 120.f))); 
    
    //// force generators springs and bungees to registry
    particle_force_registry.add(&balls[0], 
        new SpringAnchoredForceGenerator(Vector2<var_type>(150, 300), 250.f, 20.f));
    particle_force_registry.add(&balls[1], 
        new SpringAnchoredForceGenerator(Vector2<var_type>(425, 325), 250.f, 20.f));
    particle_force_registry.add(&balls[2], 
        new BungeeAnchoredForceGenerator(Vector2<var_type>(290, 270), 150.f, 28.f));
   
    // add contact generators (rods or cables), things that 'can' generate contacts
    contact_generators.push_back(new RodLink<var_type>(100, &balls[0], &balls[1])); 
    contact_generators.push_back(new RodLink<var_type>(100, &balls[2], &balls[3])); 
    contact_generators.push_back(new RodLink<var_type>(100, &balls[4], &balls[5])); 

}

void set_up_scene_main(){
    erase_scene();
    // border edges
    edges.push_back(Edge(Vector2(1.f, 1.f), Vector2(800.f, 1.f)));  // top edge
    edges.push_back(Edge(Vector2(1.f, 599.f), Vector2(799.f, 599.f))); // bottom edge
    //edges.push_back(Edge(Vector2(200, 300), Vector2(400, 320))); 
    edges.push_back(Edge(Vector2(799.f, 1.f), Vector2(799.f, 599.f))); //right edge
    edges.push_back(Edge(Vector2(1.f, 1.f), Vector2(1.f, 599.f))); // left edge
    
    // add balls
    float step = 40.0; 
    for (int i = 0; i < 10; i++) {
        balls.push_back(Ball(Vector2(100.f + i * step, 350.f), 
            Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15.f, 0.001f, 1.f));
    }

    particle_force_registry.add(&balls[0], 
        new BungeeAnchoredForceGenerator(Vector2<var_type>(100, 100), 150.0f, 28.0f));
    particle_force_registry.add(&balls[9], 
        new BungeeAnchoredForceGenerator(Vector2<var_type>(500, 100), 150.0f, 28.0f));

    for (int i = 0; i < 9; i++) {
        //contact_generators.push_back(new RodLink(step, &balls[i], &balls[i+1])); 
        contact_generators.push_back(new CableLink(step, &balls[i], &balls[i+1])); 
    }
   
    // add 2x2 crate
    balls.push_back(Ball<var_type>(Vector2(300.f, 150.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15.f, 0.001f, 1.f));
    balls.push_back(Ball<var_type>(Vector2(300.f + step, 150.f), Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15.f, 0.001f, 1.f));
    balls.push_back(Ball<var_type>(Vector2(300.f + step, 150.f + step), 
            Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(300, 150 + step), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    contact_generators.push_back(new RodLink (step, &balls[10], &balls[11]));
    contact_generators.push_back(new RodLink (step, &balls[11], &balls[12]));
    contact_generators.push_back(new RodLink (step, &balls[12], &balls[13]));
    contact_generators.push_back(new RodLink (step, &balls[13], &balls[10]));
    contact_generators.push_back(
        new RodLink<var_type> (pow((pow(step, 2) + pow(step, 2)), 0.5), &balls[10], &balls[12]));
    
    //add second chain of balls
    for (int i = 0; i < 10; i++) {
        balls.push_back(Ball<var_type>(Vector2(300.f + i * step, 450.f), 
            Vector2(0.0f, 0.0f), Vector2(0.f,0.f), 15.f, 0.001f, 1.f));
    }
    //connect second chain to itself via RodLink 
    for (int i = 14; i < 23; i++) {
        //contact_generators.push_back(new RodLink(step, &balls[i], &balls[i+1])); 
        contact_generators.push_back(new CableLink(step, &balls[i], &balls[i+1])); 
    }
    
    balls.push_back(Ball(Vector2<var_type>(550, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    particle_force_registry.add(&balls[24], 
        new SpringAnchoredForceGenerator(Vector2<var_type>(500, 300), 175.0f, 20.0f));
    
    //// add 3x2 crate
    //balls.push_back(Ball(Vector2(300, 400), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    //balls.push_back(Ball(Vector2(300 + step, 400), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    //balls.push_back(Ball(Vector2(300 + step, 400 + step), 
    //        Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    //balls.push_back(Ball(Vector2(300, 400 + step), Vector2(0.0, 0.0), Vector2(0,0), 15, 0.001, 1));
    //contact_generators.push_back(new RodLink (step, &balls[10], &balls[11]));
    //contact_generators.push_back(new RodLink (step, &balls[11], &balls[12]));
    //contact_generators.push_back(new RodLink (step, &balls[12], &balls[13]));
    //contact_generators.push_back(new RodLink (step, &balls[13], &balls[10]));
    //contact_generators.push_back(
    //    new RodLink (pow((pow(step, 2) + pow(step, 2)), 0.5), &balls[10], &balls[12]));
    

    //// add contact generators (rods or cables), things that 'can' generate contacts
    //contact_generators.push_back(new RodLink(100, &balls[0], &balls[1])); 
    //contact_generators.push_back(new RodLink(100, &balls[1], &balls[3])); 
    //contact_generators.push_back(new RodLink(100, &balls[4], &balls[5])); 
}

void set_up_scene_many_balls(){
    erase_scene();
    // border edges
    edges.push_back(Edge(Vector2<var_type>(1, 1), Vector2<var_type>(800, 1)));  // top edge
    edges.push_back(Edge(Vector2<var_type>(1, 599), Vector2<var_type>(799, 599))); // bottom edge
    edges.push_back(Edge(Vector2<var_type>(799, 1), Vector2<var_type>(799, 599))); //right edge
    edges.push_back(Edge(Vector2<var_type>(1, 1), Vector2<var_type>(1, 599))); // left edge
    edges.push_back(Edge(Vector2<var_type>(500, 200), Vector2<var_type>(200, 120))); 

    balls.push_back(Ball(Vector2<var_type>(350, 350), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(350, 250), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    ContactGenerator<var_type>* rod = new RodLink<var_type>(100, &balls[0], &balls[1]);
    contact_generators.push_back(rod); 
    emitter_time = 10.0f;
}

void set_up_scene_newton_cradle() {
    erase_scene();

    edges.push_back(Edge(Vector2<var_type>(1, 1), Vector2<var_type>(800, 1)));  // top edge
    edges.push_back(Edge(Vector2<var_type>(1, 599), Vector2<var_type>(799, 599))); // bottom edge
    edges.push_back(Edge(Vector2<var_type>(799, 1), Vector2<var_type>(799, 599))); //right edge
    edges.push_back(Edge(Vector2<var_type>(1, 1), Vector2<var_type>(1, 599))); // left edge

    balls.push_back(Ball(Vector2<var_type>(200, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(230, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(260, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(290, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(320, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    
    ContactGenerator<var_type>* newton_cable0 = new CableConstraint (200.0f, &balls[0], Vector2<var_type>(200, 200));
    ContactGenerator<var_type>* newton_cable1 = new CableConstraint (200.0f, &balls[1], Vector2<var_type>(230, 200));
    ContactGenerator<var_type>* newton_cable2 = new CableConstraint (200.0f, &balls[2], Vector2<var_type>(260, 200));
    ContactGenerator<var_type>* newton_cable3 = new CableConstraint (200.0f, &balls[3], Vector2<var_type>(290, 200));
    ContactGenerator<var_type>* newton_cable4 = new CableConstraint (200.0f, &balls[4], Vector2<var_type>(320, 200));

    contact_generators.push_back(newton_cable0);
    contact_generators.push_back(newton_cable1);
    contact_generators.push_back(newton_cable2);
    contact_generators.push_back(newton_cable3);
    contact_generators.push_back(newton_cable4);
}

void set_up_scene_crate() { 
    erase_scene();

    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(800, 1)));  // top edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 599), Vector2<var_type>(799, 599))); // bottom edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(799, 1), Vector2<var_type>(799, 599))); //right edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(1, 599))); // left edge
    int step = 50;
    balls.push_back(Ball<var_type>(Vector2<var_type>(300, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0),
    15.f, 0.001f, 1.f));
    balls.push_back(Ball<var_type>(Vector2<var_type>(300 + step, 400), Vector2<var_type>(0.0, 0.0),
    Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball<var_type>(Vector2<var_type>(300 + step, 400 + step), 
            Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(300, 400 + step), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    contact_generators.push_back(new RodLink<var_type> (step, &balls[0], &balls[1]));
    contact_generators.push_back(new RodLink<var_type> (step, &balls[1], &balls[2]));
    contact_generators.push_back(new RodLink<var_type> (step, &balls[2], &balls[3]));
    contact_generators.push_back(new RodLink<var_type> (step, &balls[3], &balls[0]));
    contact_generators.push_back(
        new RodLink<var_type> (pow((pow(step, 2) + pow(step, 2)), 0.5), &balls[0], &balls[2]));
}

void set_up_scene_double_pendulum()
{
    erase_scene();

    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(800, 1)));  // top edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 599), Vector2<var_type>(799, 599))); // bottom edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(799, 1), Vector2<var_type>(799, 599))); //right edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(1, 599))); // left edge

    balls.push_back(Ball<var_type>(Vector2<var_type>(200, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball<var_type>(Vector2<var_type>(200, 500), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));

}

void set_up_scene_energy_test()
{
    erase_scene();

    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(800, 1)));  // top edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 599), Vector2<var_type>(799, 599))); // bottom edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(799, 1), Vector2<var_type>(799, 599))); //right edge
    edges.push_back(Edge<var_type>(Vector2<var_type>(1, 1), Vector2<var_type>(1, 599))); // left edge

    float step = 40.0; 
    //add second chain of balls
    for (int i = 0; i < 10; i++) {
        balls.push_back(Ball(Vector2<var_type>(300 + i * step, 450), 
            Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    }
    //connect second chain to itself via RodLink 
    for (int i = 0; i < 9; i++) {
        //contact_generators.push_back(new RodLink(step, &balls[i], &balls[i+1])); 
        contact_generators.push_back(new CableLink(step, &balls[i], &balls[i+1])); 
    }
    step = 50;
    balls.push_back(Ball(Vector2<var_type>(150, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(150 + step, 400), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(150 + step, 400 + step), 
            Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    balls.push_back(Ball(Vector2<var_type>(150, 400 + step), Vector2<var_type>(0.0, 0.0), Vector2<var_type>(0,0), 15.f, 0.001f, 1.f));
    contact_generators.push_back(new RodLink (step, &balls[10], &balls[11]));
    contact_generators.push_back(new RodLink (step, &balls[11], &balls[12]));
    contact_generators.push_back(new RodLink (step, &balls[12], &balls[13]));
    contact_generators.push_back(new RodLink (step, &balls[13], &balls[10]));
    contact_generators.push_back(
        new RodLink<var_type>(pow((pow(step, 2) + pow(step, 2)), 0.5), &balls[10], &balls[12]));
}

#if !SDL_VERSION_ATLEAST(2,0,17)
#error This backend requires SDL 2.0.17+ because of SDL_RenderGeometry() function
#endif

int main()
{
    std::cout << "gsdl status: " << gsdl.created_ok << std::endl; 
    balls.reserve(1000);
    contacts.reserve(1000);
    contact_generators.reserve(1000);
    //set_up_scene_energy_test(); 
    set_up_scene_main();  
    //set_up_scene_rod_test();
    // set_up_scene_many_balls();
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
            SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); //(void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(gsdl.window, gsdl.renderer);
    ImGui_ImplSDLRenderer2_Init(gsdl.renderer);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    //// Our state
    //bool show_demo_window = true;
    //bool show_another_window = false;
    //ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f); 




    ////auto a = space_partition.get_index(Vector2(1200, 1100));
    //std::cout << "space " << a.first << ", " << a.second << std::endl;
    ////space_partition.get_neighbor_indexes(2, 0);
    ////std::cout << "get_neighbor_indexes(0, 0) : " << std::endl;
    //for (int i = 0; i < 9; i++) {
    //    std::cout << space_partition.neighbor_indexes[i].first 
    //            << ", " << space_partition.neighbor_indexes[i].second << "; ";
    //}
    //std::cout << std::endl;
   
    bool exit = false; 
    SDL_Event e;
    Uint64 time_frame_start;
    Uint64 time_frame_duration=0;

    SDL_Surface* image = IMG_Load("./ball.png");
    SDL_Surface* ball_selected_surface = IMG_Load("./ball_select.png");
    SDL_Surface* ball_mouseover_surface = IMG_Load("./ball_hover.png");
    
    if (!image || !ball_selected_surface || ! ball_mouseover_surface) {
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
        //SDL_GetMouseState(&x, &y);
        while (SDL_PollEvent( &e ) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            SDL_GetMouseState(&x, &y);
            mouse_pos = Vector2(x, y);  // only set this once? 
            mouse_pos_float = Vector2<var_type>(x, y);  // only set this once? 
            switch (e.type) {
                case SDL_QUIT:
                    exit = true;
                    break; 
                case SDL_MOUSEBUTTONDOWN:
                    if (run_mode == RunMode::Pause) break; 
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        ball_selected = NULL;
                        for (auto& ball : balls) {
                            Vector2 dist_vector = ball.pos - mouse_pos_float;
                            var_type dist = dist_vector.magnitude(); 
                            if (dist < ball.radius)  { 
                                ball_selected = &ball;
                                ball_selected_mass_inverse = ball.mass_inverse;
                                ball_selected_acc = ball.acc;
                                ball.mass_inverse = 0.0;  // ball has infinite weight
                                ball.vel = Vector2(0.0f, 0.0f);
                                ball.acc = Vector2(0.0f, 0.0f);
                                ball_selected_mouse_to_center = dist_vector;
                                // pinned down by mouse cursor
                            }
                        }
                    }
                    if (e.button.button == SDL_BUTTON_RIGHT) {
                        emit_ball(mouse_pos_float);
                    }
                    break;
                
                case SDL_MOUSEBUTTONUP:

                    if (e.button.button == SDL_BUTTON_LEFT) {
                        if(ball_selected) { 
                            ball_selected->mass_inverse = ball_selected_mass_inverse;
                            ball_selected = NULL;
                        }
                    }
                    break;

                case SDL_KEYDOWN:
                    if (e.key.keysym.sym == SDLK_p) {
                        if (run_mode != RunMode::Pause) {
                            run_mode = RunMode::Pause;
                        } else {
                            run_mode = RunMode::Run;
                        }
                        
                    }
                    if (e.key.keysym.sym == SDLK_z) {
                        ball_tracked = NULL;
                        for (auto& ball : balls) {
                            Vector2 dist_vector = ball.pos - mouse_pos_float;
                            var_type dist = dist_vector.magnitude(); 
                            if (dist < ball.radius)  { 
                                ball_tracked = &ball;
                                break;
                            }
                        }
                    }
                    if (e.key.keysym.sym == SDLK_F1) { display_contact_index = 0; }
                    if (e.key.keysym.sym == SDLK_F2) { display_contact_index = 1; }
                    if (e.key.keysym.sym == SDLK_F3) { display_contact_index = 2; }
                    if (e.key.keysym.sym == SDLK_F4) { display_contact_index = 3; }
                    if (e.key.keysym.sym == SDLK_F5) { display_contact_index = 4; }
                    if (e.key.keysym.sym == SDLK_F6) { display_contact_index = 5; }
                    if (e.key.keysym.sym == SDLK_F7) { display_contact_index = 6; }
                    // if paused, don't execute the follow commands
                    if (run_mode == RunMode::Pause) break;
                    if(e.key.keysym.sym == SDLK_d) {
                        if (!balls.empty()) { balls.pop_back(); }
                    }
                    if (e.key.keysym.sym == SDLK_1) { set_up_scene_main(); }
                    if (e.key.keysym.sym == SDLK_2) { set_up_scene_many_balls(); }
                    if (e.key.keysym.sym == SDLK_3) { set_up_scene_springs_bungees(); }
                    if (e.key.keysym.sym == SDLK_4) { set_up_scene_newton_cradle(); }
                    if (e.key.keysym.sym == SDLK_5) { set_up_scene_crate(); }
                    if (e.key.keysym.sym == SDLK_6) { set_up_scene_energy_test(); }
                    break;

                default:
                    break;
            }
        }
       
        if ( run_mode != RunMode::Pause) {
            if (run_mode == RunMode::RunNumFrames) {
                if (num_frames <= 0) {
                    run_mode = RunMode::Pause;
                } else {
                    num_frames--;
                }
            }
            // update state of the simulation. Using 'mini-steps' for better resolution.
            var_type duration = 1.0/(float)num_steps_per_loop;
            // ball emitter 
            emitter_time_current -= duration; 
            emitter_time -= duration;
            if (emitter_time_current <= 0.0f && emitter_time >= 0.0) {
                emitter_time_current = emitter_time_per_ball;
                emit_ball(Vector2(200.0f, 100.0f));
            }
            // apply gravity (should pretty much be constant)
            for (Ball<var_type>& ball : balls) {
                ball.acc.y = gravity * ball.mass_inverse;
            }
            for (int i = 0; i < num_steps_per_loop; i++) {
                update(duration);
            }
        }
        // check if mouse is over any balls
        ball_mouseover = NULL;  // clear previous selection
        for (auto& ball : balls) {
            Vector2 dist_vector = ball.pos - mouse_pos_float;
            var_type dist = dist_vector.magnitude(); 
            if (dist < ball.radius) { ball_mouseover = &ball; }
        }
        // render the simulation
        render(io);
        // ensure frame rate is steady
        time_frame_duration = SDL_GetTicks64() - time_frame_start;
        if (time_frame_duration < SCREEN_TICK_PER_FRAME) {
            SDL_Delay( SCREEN_TICK_PER_FRAME - time_frame_duration );
        }
    }
    // clean up
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyWindow(gsdl.window);
    TTF_CloseFont(gsdl.font);
    SDL_DestroyTexture(ball_texture);
    SDL_DestroyTexture(ball_selected_texture);
    SDL_DestroyTexture(energy_readout_texture);
    IMG_Quit();
    SDL_Quit();
}

