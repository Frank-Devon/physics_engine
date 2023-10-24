#include <cmath>
#include <iostream>
#include "vector.hpp"
#include "physics.hpp"
#include "sdl_init.hpp" // this allows draw functions to be called

var_type Ball::restitution = 1.0;

void spring_draw(Vector2 start, Vector2 end, var_type rest_length);

Ball::Ball(Vector2 pos, Vector2 vel, Vector2 acc,
        var_type radius, var_type mass_inverse, var_type elasticity) 
        : pos(pos), vel(vel), acc(acc), radius(radius), 
          mass_inverse(mass_inverse), elasticity(elasticity) 
{
    force_accumulator = Vector2(0.0, 0.0);
}

void Ball::integrate(var_type duration) 
{
    // implicit euler method
    // determine acceleration from all combining all the forces acting on this ball.
    vel = vel + duration * (acc + mass_inverse * force_accumulator);
    pos = pos + duration * vel;
    force_accumulator = Vector2(0.0, 0.0);
}

// checks if balls have collided then fixes position and velocity. 
void Ball::collides_ball(Ball& b) 
{
    Vector2 pos_relative = b.pos - pos;
    var_type penetration = radius + b.radius - pos_relative.magnitude();
    if (penetration > 0) {
        // objects are penetrating
        Vector2 new_vel(0.0, 0.0);
        Vector2 b_new_vel(0.0, 0.0);
        var_type mass = 1.0/mass_inverse;
        var_type b_mass = 1.0/b.mass_inverse;
        var_type mass_sum = mass + b_mass;
        var_type dot_0 = Vector2::dot( vel - b.vel, pos - b.pos);
        var_type dot_1 = Vector2::dot( b.vel - vel, b.pos - pos);
        var_type distance_squared = pow(pos_relative.magnitude(), 2.0);
        new_vel = vel - (2.0 * b_mass / mass_sum) * (dot_0 / distance_squared) * (pos - b.pos); 
        b_new_vel = b.vel - (2.0 * mass / mass_sum) * (dot_1 / distance_squared) * (b.pos - pos); 
        // assign new velocities
        vel = new_vel;
        b.vel = b_new_vel;
        // solve interpenetrations
        Vector2 delta   = b_mass * vel.unit() / mass_sum;
        Vector2 delta_b = mass * b.vel.unit() / mass_sum;
        pos += delta;
        b.pos += delta_b;
        //std::cout << "howdy" << penetration << std::endl;
    }
    else {
        //std::cout << "pen: " << penetration << std::endl;
    }
}

// checks if ball has collided with edge then fixes position and velocity
void Ball::collides_edge(const Edge& edge) 
{
    // P S E are 2d Vectors (Vec2)
    // P = center of ball
    // S = start of edge
    // E = end of edge
    Vector2 PS = this->pos - edge.start;   //Vec2_sub(&ball->pos, &edge->start); 
    Vector2 SE = edge.end - edge.start; //edge.start - edge.end;   //Vec2_sub(&edge->end, &edge->start);

    // will find the SE.x, SE.SE.x, SE.y point on the edge to the ball
    var_type dot_product = PS.dot(SE);  //Vec2_dot_product(&PS, &SE);
    var_type SE_magnitude = SE.magnitude();  //Vec2_magnitude(&SE);
    // normalize t between 0 and 1
    var_type t = fmax(0.0, fmin(SE_magnitude, dot_product / SE_magnitude));
    t = t / SE_magnitude;
    //std::cout << "t: " << t << std::endl;
    Vector2 closest_point = t * SE; //Vec2_scale(&SE, t);
    closest_point = closest_point + edge.start;  //Vec2_add(&closest_point, &edge->start); // get global position
    //printf("closest circ = %f, %f", closest_point.x, closest_point.y);
    Vector2 penetration_vector = closest_point - pos;
    var_type penetration = radius - penetration_vector.magnitude();
    if (penetration > 0) {
        // this will correct the velocity, but not position      
        //Ball_resolve_collision(ball, &ball_edge);  
        Vector2 v_reflected = Collision::reflect(vel, edge);
        vel = v_reflected;

        // correct position
        Vector2 delta = penetration * v_reflected.unit();
        pos = pos + delta;
        // this will correct the position.
        //Vec2 pos_rel = Vec2_sub(&ball->pos, &ball_edge.pos); 
        //var_type penetration_depth = Vec2_magnitude(&pos_rel) - ball->radius - ball_edge.radius;

        //Vec2 velocity_unit = Vec2_unit(&ball->vel);
        //Vec2 push_ball = Vec2_scale(&velocity_unit, penetration_depth);
        //ball->pos = Vec2_add(&push_ball, &ball->pos);
    }
}

Edge::Edge(Vector2 start, Vector2 end) : start(start), end(end)
{
    Vector2 temp = end - start;  // temp points from start to end
    tangent = temp.unit(); 
    normal = tangent.perpendicular();
}

Vector2 Collision::reflect(const Vector2& a, const Edge& edge)
{
    var_type v_dot_t = a.dot(edge.tangent); //(Vec2_dot_product(a, &edge->tangent));
    var_type v_dot_n = a.dot(edge.normal);//(Vec2_dot_product(a, &edge->normal));
    Vector2 normal = edge.normal;  // might be adjusted 
    Vector2 tangent = edge.tangent;  // might be adjusted
    if (v_dot_n > 0) {
        v_dot_n = -1.0 * v_dot_n;
        normal = - edge.normal; 
    }
    if (v_dot_t < 0) {
        v_dot_t = -1.0 * v_dot_t;
        tangent = -1.0 * tangent; 
    }
    Vector2 v_dot_t_vec = v_dot_t * tangent; 
    Vector2 v_dot_n_vec = v_dot_n * normal ;  
    Vector2 result = v_dot_t_vec - v_dot_n_vec; 
    return result;
}


SpringForceGenerator::SpringForceGenerator(Ball* other_ball, var_type rest_length, var_type spring_constant)
    : other_ball(other_ball), rest_length(rest_length), spring_constant(spring_constant)
{ }

void SpringForceGenerator::update(Ball* ball, var_type duration)
{
    // get vector representing relative position of string
    Vector2 spring_vector = ball->pos - other_ball->pos;
    // find current length of spring
    var_type spring_length = spring_vector.magnitude();
    // find force generated on ball
    Vector2 force = - spring_constant * ( spring_length - rest_length) * spring_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;
    // add opposite force to other ball
    other_ball->force_accumulator -= force;
}

void SpringForceGenerator::draw(Ball* ball, var_type duration)
{
   //void spring_draw(Vector2 start, Vector2 end, var_type rest_length);
   spring_draw(ball->pos, other_ball->pos, rest_length);
}

SpringAnchoredForceGenerator::SpringAnchoredForceGenerator(Vector2* anchor_position, var_type rest_length, var_type spring_constant) : anchor_position(anchor_position), rest_length(rest_length), spring_constant(spring_constant)
{ }

void SpringAnchoredForceGenerator::update(Ball* ball, var_type duration)
{
    // get vector representing relative position of string
    Vector2 spring_vector = ball->pos - *anchor_position;
    // find current length of spring
    var_type spring_length = spring_vector.magnitude();
    // find force generated on ball
    Vector2 force = - spring_constant * ( spring_length - rest_length) * spring_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;
}

void SpringAnchoredForceGenerator::draw(Ball* ball, var_type duration)
{
    // maybe draw square where anchor is
    spring_draw(ball->pos, *anchor_position, rest_length);
}

GravityForceGenerator::GravityForceGenerator(Vector2 force) : force(force) { }

void GravityForceGenerator::update(Ball* ball, var_type duration)
{
    ball->force_accumulator += force;
}

void GravityForceGenerator::draw(Ball* ball, var_type duration) { }

void ParticleForceRegistry::update_all(var_type duration)
{
    for(auto& registry : registrations) {
        registry.force_generator->update(registry.ball, duration);
    }
}

void ParticleForceRegistry::draw_all(var_type duration)
{
    for(auto& registry : registrations) {
        registry.force_generator->draw(registry.ball, duration);
    }
}

BungieForceGenerator::BungieForceGenerator(Vector2* anchor_position, var_type rest_length, var_type spring_constant)
: anchor_position(anchor_position), rest_length(rest_length), spring_constant(spring_constant)
{ }

void BungieForceGenerator::update(Ball* ball, var_type duration)
{
    // get vector representing relative position of strinunsigned
    Vector2 bungie_vector = ball->pos - *anchor_position;
    // find current length of spring
    var_type bungie_length = bungie_vector.magnitude();
    // 
    if (bungie_length < rest_length) return;  // no force applied when bungie isn't extended
    // find force generated on ball
    Vector2 force = - spring_constant * ( bungie_length - rest_length) * bungie_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;
}

void BungieForceGenerator::draw(Ball* ball, var_type duration) 
{
    SDL_SetRenderDrawColor(gsdl.renderer, 128, 200, 0, 0);
    SDL_RenderDrawLine(gsdl.renderer, ball->pos.x, ball->pos.y, anchor_position->x, anchor_position->y);
}

void ParticleForceRegistry::add(Ball* ball, ParticleForceGenerator* fg)
{
    ParticleForceRegistry::ForceRegistration entry;
    entry.ball = ball;
    entry.force_generator = fg;
    // minor hack to make sure dynamic array doesn't reallocate
    registrations.reserve(1000);
    // 
    //TODO check size, and print error message if dynamic array reallocates
    //
    registrations.push_back(entry);
}


void ParticleForceRegistry::remove(Ball* ball, ParticleForceGenerator* fg)
{
    // not needed now
    // TODO
}

void ParticleForceRegistry::clear()
{
    // not needed now
    // TODO
}

void Contact::resolve(var_type duration)
{
    resolve_interpenetration(duration);
    resolve_velocity(duration);
}

var_type Contact::seperating_speed_calculate() const
{
    //if (!ball[1]) return ball[0]->vel; // in case theres no 2nd ball
    //Vector2 pos_rel = ball[0]->pos - ball[1]->pos;
    //Vector2 v = ball[0]->vel - ball[1]->vel;    
    //if (Vector2.dot(pos_rel, v) < 0) {
    //    // objects are approaching
    //    return - v.magnitude;
    //} else { 
    //    // objects are seperating
    //    return v.magnitude;
    //}
    // alternatively 
    Vector2 vel_rel = ball[0]->vel;
    if (!ball[1]) vel_rel = vel_rel - ball[1]->vel;
    return Vector2::dot(contact_normal, vel_rel);
}

void Contact::resolve_interpenetration(var_type duration)
{
//TODO    
}

void Contact::resolve_velocity(var_type duration)
{
//TODO
}

ContactResolver::ContactResolver(unsigned int iterations_max) : iterations_max(iterations_max)
{ 
    iterations_count = 0;
}    

void ContactResolver::set_iterations(unsigned int iterations_max)
{
    this->iterations_max = iterations_max;
}

void ContactResolver::resolve_contacts(Contact* contacts, unsigned int num_contacts, var_type duration)
{
//TODO
}

// private function
void spring_draw(Vector2 start, Vector2 end, var_type rest_length)
{
    using std::max, std::min;
    // find how many "turns" are in the spring. depends only on rest_length
    int turns_count = min(10, (int)max(rest_length / 25.0, 3.0)); // max 10 turns, min 3
    // "turn" positions
    Vector2 turn_positions[10];
    // how far does the "turn" extend
    var_type turn_distance = 25.0;

    // relative position
    Vector2 pos_relative = end - start;
    // get current length
    var_type length = pos_relative.magnitude();
    // get direction
    Vector2 direction = pos_relative.unit();
    // 5% of the total length will be drawn as a straight wire. Instead of a turning wire.
    Vector2 start_turn, end_turn;
    start_turn = start + 0.025 * length * direction;
    end_turn   = end - 0.025 * length * direction;
    // get current length of turning wire
    var_type length_turn = length * 0.95;
    // get distance between each turn
    var_type length_per_turn = length_turn / ((float)turns_count + 1.0);
    // determine positions of "turns"
    var_type length_current = 0.0;
    Vector2 turn_vector = turn_distance * direction.perpendicular();
    for(int i = 0; i < turns_count; i++)
    {
       length_current += length_per_turn;
       turn_vector = -1.0 * turn_vector;
       // convert length of "walk", to Vector2 position
       Vector2 centered_pos = start_turn + length_current * direction;
       turn_positions[i] = centered_pos + turn_vector; 
    }
    

    //
    // draw spring from data determined above
    //
    int displacement = length - rest_length;
    if (displacement > 250) displacement = 250;
    else if (displacement < -250) displacement = -250;
    
    int green = 0; int red = 0;
    if (displacement > 0) green = green + displacement;
    else red = red - displacement;
    
    SDL_SetRenderDrawColor(gsdl.renderer, red, green, 30, 0);


    //// drawing the "spring" as a straight wire
    //int test = SDL_RenderDrawLine(gsdl.renderer, start.x, start.y, end.x, end.y);
    // drawing straight ends of the spring
    SDL_RenderDrawLine(gsdl.renderer, start.x, start.y, start_turn.x, start_turn.y);
    SDL_RenderDrawLine(gsdl.renderer, end.x, end.y, end_turn.x, end_turn.y);
    // drawing first and last turn of the spring
    SDL_RenderDrawLine(gsdl.renderer, start_turn.x, start_turn.y, 
                        turn_positions[0].x, turn_positions[0].y);
    SDL_RenderDrawLine(gsdl.renderer, end_turn.x, end_turn.y, 
                        turn_positions[turns_count - 1].x, turn_positions[turns_count - 1].y);
    for( int i = 0; i < turns_count - 1; i++)
    {
        SDL_RenderDrawLine(gsdl.renderer, turn_positions[i].x, turn_positions[i].y,
                        turn_positions[i + 1].x, turn_positions[i + 1].y);
    }
}




