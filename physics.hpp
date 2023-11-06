#ifndef PHYSICS_HPP
#define PHYSICS_HPP
#include <vector>
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
    Vector2 force_accumulator;
    static var_type restitution;
    
    Ball(Vector2 pos, Vector2 vel, Vector2 acc,
        var_type radius, var_type mass_inverse, var_type elasticity);
    Ball() {}
    
    void integrate(const var_type duration);
    void force_add(Vector2 force);  // adds force to force_accumulator
    // sequential collision checking
    void collides_ball(Ball& ball);  // delete?
    void collides_edge(const Edge& edge);
};

class Edge {
public:
    Vector2 start, end;
    Vector2 normal, tangent;  // derived from start and end. calc's stored here.

    Edge(Vector2 start, Vector2 end);
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


class ParticleForceGenerator {
public:
    virtual void update(Ball* ball, var_type duration) = 0; 
    virtual void draw(Ball* ball, var_type duration) = 0;
};


class SpringForceGenerator : public ParticleForceGenerator {
private:
    Ball* other_ball;
    var_type rest_length;
    var_type spring_constant;
public:
    SpringForceGenerator(Ball* other_ball, var_type rest_length, var_type spring_constant);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);
};


class SpringAnchoredForceGenerator : public ParticleForceGenerator {
private:
    Vector2* anchor_position;
    var_type rest_length;
    var_type spring_constant;
public:
    SpringAnchoredForceGenerator(Vector2* anchor_position, var_type rest_length, var_type spring_constant);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);
};


//should be named BungieAnchoredForceGenerator
class BungeeForceGenerator : public ParticleForceGenerator {
private:
    Vector2* anchor_position;  // anchor could be attached to another ball?
    var_type rest_length;
    var_type spring_constant;
public:
    BungeeForceGenerator(Vector2* anchor_position, var_type rest_length, var_type spring_constant);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);
};


class GravityForceGenerator : public ParticleForceGenerator { 
private:
    Vector2 force;
public:
    GravityForceGenerator(Vector2 force);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);

};


class ParticleForceRegistry {
private:
    struct ForceRegistration
    {
        Ball* ball;
        ParticleForceGenerator* force_generator;
    };
    std::vector<ForceRegistration> registrations;

public:
    void update_all(var_type duration);
    void draw_all(var_type duration);
    void add(Ball* ball, ParticleForceGenerator* fg);
    void remove(Ball* ball, ParticleForceGenerator* fg);
    void clear();
};


//start using this to resolve collisions and other hard constraints
class Contact {
public:
    Ball* ball[2];
    Vector2 contact_normal;  // Contact normal
    var_type restitution;
    var_type penetration;
    Vector2 movement[2];
    var_type seperating_speed_calculate() const;
    void resolve(var_type duration);
protected:
private:
    void resolve_interpenetration(var_type duration);  
    void resolve_velocity(var_type duration);
};


class ContactResolver { 
protected:
    unsigned int iterations_max;
    unsigned int iterations_count;
public:
    ContactResolver(unsigned int iterations_max);
    void set_iterations(unsigned int iterations_max);
    void resolve_contacts(std::vector<Contact>& contacts, var_type duration);
};


class ContactGenerator {
public:
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit) = 0;
    virtual void draw() = 0;
};


class ParticleLink : public ContactGenerator {
public:
    Ball* balls[2];
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit) = 0;
    virtual void draw() = 0;
protected:
    var_type length_current_calculate() const;
};


class Rod : public ParticleLink {
public:
    var_type length;
    Rod(var_type length);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};


class Cable : public ParticleLink {
public:
    var_type length_max;
    var_type restitution;
    Cable(var_type length_max, var_type restitution);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};


void spring_draw(Vector2 start, Vector2 end, var_type rest_length);


#endif
