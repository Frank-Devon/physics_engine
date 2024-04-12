#ifndef PHYSICS_HPP
#define PHYSICS_HPP
#include <vector>
#include <unordered_map>
#include "vector.hpp"

class Edge;

class Ball {
public:
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
    //Vector2 acc_const;
    var_type radius;
    var_type mass_inverse;
    var_type elasticity;
    Vector2 force_accumulator;
    Vector2 pos_old;
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
    //Ball* ball0;
    //Ball* ball1;
    //Edge* edge;
    //var_type penetration;
    //Vector2 contact_normal;
    //Vector2 velocity_seperating;

    static Vector2 reflect(const Vector2& v, const Edge& edge);
};


class ParticleForceGenerator {
public:
    virtual ~ParticleForceGenerator();
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
    Vector2 anchor_position;
    var_type rest_length;
    var_type spring_constant;
public:
    SpringAnchoredForceGenerator(Vector2 anchor_position, var_type rest_length, var_type spring_constant);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);
};

class BungeeForceGenerator : public ParticleForceGenerator {
private:
    Ball* other_ball;  // anchor could be attached to another ball?
    var_type rest_length;
    var_type spring_constant;
public:
    BungeeForceGenerator(Ball* other_ball, var_type rest_length, 
        var_type spring_constant);
    virtual void update(Ball* ball, var_type duration);
    virtual void draw(Ball* ball, var_type duration);
};

class BungeeAnchoredForceGenerator : public ParticleForceGenerator {
private:
    Vector2 anchor_position;  // anchor could be attached to another ball?
    var_type rest_length;
    var_type spring_constant;
public:
    BungeeAnchoredForceGenerator(Vector2 anchor_position, var_type rest_length, 
        var_type spring_constant);
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
        Ball* ball;  //TODO have ball just be another member of ParticleForceGenerator derived types
        ParticleForceGenerator* force_generator;
    };
    std::vector<ForceRegistration> registrations;

public:
    ParticleForceRegistry();
    ~ParticleForceRegistry();
    void update_all(var_type duration);
    void draw_all(var_type duration);
    void add(Ball* ball, ParticleForceGenerator* fg);
    void remove(Ball* ball, ParticleForceGenerator* fg);
    void clear();
};


//start using this to resolve collisions and other hard constraints
class Contact {
public:
    Contact();
    Contact(Ball* b0, Ball* b1, var_type penetration, var_type restitution);
    Ball* ball[2];
    Vector2 contact_normal;  // Contact normal
    var_type restitution;
    var_type penetration;
    Vector2 movement[2];
    var_type seperating_speed;  // save result of seperating_speed_calculate 
    var_type seperating_speed_calculate() const;
    var_type interpenetration_calculate() const;
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
    // for fast look up of effected contacts
    std::unordered_map<Contact*, std::vector<Contact*>> contact_map;
    void set_iterations(unsigned int iterations_max);
    void resolve_contacts(std::vector<Contact>& contacts, var_type duration);
};


class ContactGenerator {
public:
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit) = 0;
    virtual void draw() = 0;
    // constraints have 1 particle connected to an anchor point. if the user is 
    // controlling the position of a ball connected to a constraint, then the contact will
    // never be resolved and it can screw up the contact resolver
    virtual bool disable_generation(Ball* ball) const = 0; 
    virtual ~ContactGenerator();
    //virtual var_type length_current_calculate() const = 0;
};


class ParticleLink : public ContactGenerator {
public:
    Ball* balls[2];
    bool disable_generation(Ball* _ball) const;
protected:
    ParticleLink(Ball* ball0, Ball* ball1);
    var_type length_current_calculate() const;
};


class ParticleConstraint : public ContactGenerator {
public:
    Ball* ball;
    Vector2 anchor;
    bool disable_generation(Ball* _ball) const;
protected:
    ParticleConstraint(Ball* ball, Vector2 anchor);
    var_type length_current_calculate() const;
};


class RodLink : public ParticleLink {
public:
    var_type length;
    RodLink(var_type length, Ball* ball0, Ball* ball1);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};

class RodConstraint : public ParticleConstraint {
public:
    var_type length;
    RodConstraint(var_type length, Ball* ball0, Vector2 anchor);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};

class CableLink : public ParticleLink {
public:
    var_type length_max;
    CableLink(var_type length_max, Ball* ball0, Ball* ball1);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};

//class

class CableConstraint : public ParticleConstraint {
public:
    var_type length_max;
    CableConstraint(var_type length_max, Ball* ball, Vector2 anchor);
    virtual unsigned int generate_contact(std::vector<Contact>& contacts, unsigned limit);
    virtual void draw();
};

class PhysicsWorld {
public:
    std::vector<Ball> balls;                       
    std::vector<Edge> edges;                                      
    std::vector<Ball> balls_fake;
    std::vector<Contact> contacts;
    std::vector<ContactGenerator*> contact_generators;
    Ball* ball_mouseover;
    Ball* ball_selected;
    var_type ball_selected_mass_inverse;
    //Ball ball_selected_concrete;             
    ParticleForceRegistry particle_force_registry; 
    ContactResolver contact_resolver;

    PhysicsWorld();
    PhysicsWorld(std::string file_name);
    ~PhysicsWorld();

};

void spring_draw(Vector2 start, Vector2 end, var_type rest_length);


#endif
