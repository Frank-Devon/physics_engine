# physics_engine
This is a very rough physics simulation. Particles, mass, any-angle edges, and collisions are simulated. More to come soon.

Requires SDL2, SDL_image, SDL_ttf libraries. Static linking is used.
Compiles on linux with:
g++ *.cpp -lSDL2 -lSDL2_image -lSDL2_ttf -o a

Right click anywhere to place a particle/ball with random velocity. Left click and hold on a ball to move it's position. Press d to delete a ball.

Bugs
- Collision of a ball against the endpoint of an edge causes error in ball position
- Manually moving a ball into an edge causes a crash.
- Manually moving a ball into another ball can cause issues.

These bugs will be fixed with the collision detection/resolution overhaul.
