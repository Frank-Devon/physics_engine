#ifndef DRAW_HPP
#define DRAW_HPP

template <typename T = float>
void draw_line(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b);
void draw_thick_line(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width);
template <typename T = float>
void draw_thick_line(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b, T width);
template <typename T = float>
void draw_arrow(SDL_Renderer* renderer, Vector2<T> a, Vector2<T> b, T width);
void draw_arrow(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, int width);

//ints
extern template void draw_line(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b);
extern template void draw_thick_line(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b, int width);
extern template void draw_arrow(SDL_Renderer* renderer, Vector2<int> a, Vector2<int> b, int width);

//floats
extern template void draw_line(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b);
extern template void draw_thick_line(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b, float width);
extern template void draw_arrow(SDL_Renderer* renderer, Vector2<float> a, Vector2<float> b, float width);

//doubles
extern template void draw_line(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b);
extern template void draw_thick_line(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b, double width);
extern template void draw_arrow(SDL_Renderer* renderer, Vector2<double> a, Vector2<double> b, double width);
#endif
