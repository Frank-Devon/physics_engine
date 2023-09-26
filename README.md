# physics_engine
This is a very rough physics simulation. Particles, mass, any-angle edges, and collisions are simulated. More to come soon.

Requires SDL2, SDL_image, SDL_ttf libraries. Static linking is used.
Compiles on linux with:
g++ *.cpp -lSDL2 -lSDL2_image -lSDL2_ttf -o a

Right click anywhere to place a particle/ball with random velocity. Press d to delete a ball.
