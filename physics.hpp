#ifndef PHYSICS_HPP
#define PHYSICS_HPP
#include "vector.hpp"

class Edge;

class Ball {
public:
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
    var_type radius;
    var_type mass_inverse;
    var_type elasticity;
    static var_type restitution;
    
    Ball(Vector2 pos, Vector2 vel, Vector2 acc,
        var_type radius, var_type mass_inverse, var_type elasticity) 
        : pos(pos), vel(vel), acc(acc),
          radius(radius), mass_inverse(mass_inverse), elasticity(elasticity) {}
    
    Ball() {}
    
    void integrate(const var_type duration);
    // sequential collision checking
    void collides_ball(Ball& ball);  // delete?
    void collides_edge(const Edge& edge);
};

class Edge {
public:
    Vector2 start, end;
    Vector2 normal, tangent;  // derived from start and end. calc's stored here.

    Edge(Vector2 start, Vector2 end)
        : start(start), end(end)
        {
            Vector2 temp = end - start;  // temp points from start to end
            tangent = temp.unit(); 
            normal = tangent.perpendicular();
        }
};

class Collision {
public:
    Ball* ball0;
    Ball* ball1;
    Edge* edge;
    var_type penetration;
    Vector2 contact_normal;
    Vector2 velocity_seperating;

    static Vector2 reflect(const Vector2& v, const Edge& edge);
};

#endif
