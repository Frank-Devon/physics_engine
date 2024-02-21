# physics_engine
This is a physics simulation. Springs, bungees, rods, and collisions are simulated. Here's a video: https://vimeo.com/877286187

Requires SDL2, SDL_image, SDL_ttf libraries.
Compiles on linux with:
g++ -g -Wall -Werror *.cpp -lSDL2 -lSDL2_image -lSDL2_ttf -ldl -o a

Right click anywhere to place a particle/ball with random velocity. Left click and hold on a ball to move it's position. Press d to delete a ball.

Bugs
- While manually moving a ball attached to a rod, the rod's length can change for several frames.
