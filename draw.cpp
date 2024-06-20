#include <SDL2/SDL.h>
#include <cmath>
#include "vector.hpp"
#include "draw.hpp"

template <typename T>
void draw_line(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b) 
{
    SDL_RenderDrawLine(renderer, llround(a.x), llround(a.y), llround(b.x), llround(b.y));
}

template <typename T>
void draw_thick_line(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b, T width)
{
    //Vector2<float> dir = b - a;
    Vector2<float> dir{};
    dir.x = b.x - a.x;
    dir.y = b.y - a.y;
    Vector2<int> a_int{};
    Vector2<int> b_int{};
    a_int.x = round(a.x);
    a_int.y = round(a.y);
    b_int.x = round(b.x);
    b_int.y = round(b.y);
    Vector2<float> perp = dir.perpendicular().unit();
    Vector2<int> line_offset{};
    if (fabs(perp.x) < fabs(perp.y)) {
        line_offset.x = 0;
        line_offset.y = (perp.y > 0.0f) ? 1 : -1;
    } else {
        line_offset.x = (perp.x > 0.0f) ? 1 : -1;
        line_offset.y = 0; 
    }
   
    
    draw_line(renderer, a_int, b_int);
    draw_line(renderer, a_int + line_offset, b_int + line_offset);
    draw_line(renderer, a_int - line_offset, b_int - line_offset);

    //// assume thickness is 3
    //Vector2<T> norm_l = a.perpendicular().unit();
    //Vector2<T> norm_r =  T( -1.0f) * a.perpendicular().unit();
    //Vector2<T> a_l0 = a +T( 0.5f ) * norm_l;
    //Vector2<T> a_l1 = a +T( 1.0f ) * norm_l;
    //Vector2<T> a_r0 = a +T( 0.5f ) * norm_r;
    //Vector2<T> a_r1 = a +T( 1.0f ) * norm_r;
    //Vector2<T> b_l0 = b +T( 0.5f ) * norm_l;
    //Vector2<T> b_l1 = b +T( 1.0f ) * norm_l;
    //Vector2<T> b_r0 = b +T( 0.5f ) * norm_r;
    //Vector2<T> b_r1 = b +T( 1.0f ) * norm_r;
    //
    //draw_line(renderer, a_l0, b_l0);
    //draw_line(renderer, a_l1, b_l1);
    //draw_line(renderer, a, b);
    //draw_line(renderer, a_r0, b_r0);
    //draw_line(renderer, a_r1, b_r1);
}

void draw_thick_line(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width)
{
    draw_thick_line(renderer, Vector2<int>(x1, y1) , Vector2<int>(x2, y2), width);
}

template <typename T>
void draw_arrow(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b, T width) 
{
    // points to a from b
    Vector2 dir = (a - b).unit();
    Vector2 head_start = a + T(-8) * dir; // point on the shaft where the arrowhead begins
    Vector2 dir_p = dir.perpendicular();
    Vector2 head_left = head_start + T(8) * dir_p;
    Vector2 head_right = head_start - T(8) * dir_p;
   
    draw_thick_line(renderer, a, b, width); // draw shaft
    draw_thick_line(renderer, a, head_left, width); // draw shaft
    draw_thick_line(renderer, a, head_right, width); // draw shaft
    
}

void draw_arrow(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width) 
{
    draw_arrow(renderer, Vector2<int>(x1, y1), Vector2<int>(x2, y2), width);
}

//ints
template void draw_line(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b);
template void draw_thick_line(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b, int width);
//void draw_thick_line(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width);
template void draw_arrow(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b, int width);
//void draw_arrow(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width) 

//floats
template void draw_line(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b);
template void draw_thick_line(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b, float width);
template void draw_arrow(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b, float width);

//doubles
template void draw_line(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b);
template void draw_thick_line(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b, double width);
template void draw_arrow(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b, double width);
