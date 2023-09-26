#include <cmath>
#include <iostream>
#include "vector.hpp"
#include "physics.hpp"

var_type Ball::restitution = 1.0;

void Ball::integrate(var_type duration) {
    vel = vel + duration * acc;
    pos = pos + duration * vel;
}

void Ball::collides_ball(Ball& b) {
    // check if in collision, then immediately fix velocity and position
    //Ball& a = this;
    // ball b is the objecting
    Vector2 pos_relative = b.pos - pos;
    var_type penetration = radius + b.radius - pos_relative.magnitude();
     
    if (penetration > 0) {
        // objects are penetrating
        Vector2 new_vel(0.0, 0.0);
        Vector2 b_new_vel(0.0, 0.0);
        var_type mass = 1.0/mass_inverse;
        var_type b_mass = 1.0/b.mass_inverse;
        new_vel = (mass * vel + b_mass * b.vel - b_mass * restitution * vel
            + b_mass * restitution * b.vel) / ( mass + b_mass);
        b_new_vel = (mass * vel + b_mass * b.vel - mass * restitution * b.vel
            + mass * restitution * vel) / ( mass + b_mass);
        // assign new velocities
        vel = new_vel;
        b.vel = b_new_vel;
        // solve interpenetrations
        Vector2 normal_contact = pos_relative.unit();
        Vector2 delta = b_mass * normal_contact / (mass + b_mass);
        Vector2 delta_b = mass *  normal_contact / (mass + b_mass);
        pos += delta;
        b.pos += delta_b;

        //std::cout << "howdy" << penetration << std::endl;
    }
    else {
        //std::cout << "pen: " << penetration << std::endl;
    }
}

void Ball::collides_edge(const Edge& edge) {

    // P S E are 2d Vectors (Vec2)
    // P = center of ball
    // S = start of edge
    // E = end of edge
    //Vec2 PS = Vec2_sub(&edge->start, &ball->pos); 
    Vector2 PS = this->pos - edge.start;   //Vec2_sub(&ball->pos, &edge->start); 
    Vector2 SE = edge.end - edge.start; //edge.start - edge.end;   //Vec2_sub(&edge->end, &edge->start);

    // will find the SE.x, SE.SE.x, SE.y point on the edge to the ball
    var_type dot_product = PS.dot(SE);  //Vec2_dot_product(&PS, &SE);
    //printf("dot = %f\n", dot_product);
    var_type SE_magnitude = SE.magnitude();  //Vec2_magnitude(&SE);
    //printf("SE_mag = %f\n", SE_magnitude);
    // normalize t between 0 and 1
    var_type t = fmax(0.0, fmin(SE_magnitude, dot_product / SE_magnitude));
    t = t / SE_magnitude;
    //std::cout << "t: " << t << std::endl;
    Vector2 closest_point = t * SE; //Vec2_scale(&SE, t);
    closest_point = closest_point + edge.start;  //Vec2_add(&closest_point, &edge->start); // get global position
    //printf("t = %f\n", t);
    //printf("closest circ = %f, %f", closest_point.x, closest_point.y);
    Vector2 penetration_vector = closest_point - pos;
    var_type penetration = radius - penetration_vector.magnitude();
    //Ball ball_edge;  // create a ball on the edge that is closest to actual oncoming ball
    //ball_edge.pos = closest_point;
    //ball_edge.vel = Vec2_scale(&ball->vel, -1.0); /*ball_edge.acc = 0;*/ 
    //ball_edge.radius = edge->radius;
    //ball_edge.mass_inverse = 1;
    if(penetration > 0)
    {
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

    Vector2 
Collision::reflect(const Vector2& a, const Edge& edge)
{
    var_type v_dot_t = a.dot(edge.tangent); //(Vec2_dot_product(a, &edge->tangent));
    var_type v_dot_n = a.dot(edge.normal);//(Vec2_dot_product(a, &edge->normal));
    Vector2 normal = edge.normal;  // might be adjusted 
    Vector2 tangent = edge.tangent;  // might be adjusted
    if (v_dot_n > 0)
    { 
        v_dot_n = -1.0 * v_dot_n;
        normal = - edge.normal; //Vec2_scale(&normal, -1.0);
    }
    if (v_dot_t < 0)
    { 
        v_dot_t = -1.0 * v_dot_t;
        tangent = -1.0 * tangent; //Vec2_scale(&tangent, -1.0);
    }

    //printf("--- v_dot_t and n: %f, %f\n", v_dot_t, v_dot_n);
    Vector2 v_dot_t_vec = v_dot_t * tangent;  //Vec2_scale(&tangent, v_dot_t);
    Vector2 v_dot_n_vec = v_dot_n * normal ;  //Vec2_scale(&normal, v_dot_n);
    //printf("--- v_dot_t_vec and n: %f, %f\n", v_dot_t_vec, v_dot_n_vec);
    Vector2 result = v_dot_t_vec - v_dot_n_vec; //Vec2_sub(&v_dot_t_vec, &v_dot_n_vec);
    //printf("--- result: %f, %f\n", result.x, result.y);
    return result;
}
