#ifndef PHYSICS_HPP
#define PHYSICS_HPP
#include <vector>
#include <unordered_map>
#include "vector.hpp"

template <typename T = var_type> class Edge;

template <typename T = var_type>
class Ball {
public:
    enum class IntegrationMethod { explicit_euler, implicit_euler, explicit_midpoint };
    Vector2<T> pos;
    Vector2<T> vel;
    Vector2<T> acc;  // figured as constant acceleration value (due to gravity).
    //Vector2 acc_const;
    var_type radius;
    var_type mass_inverse;
    var_type elasticity;
    Vector2<T> force_accumulator;
    Vector2<T> pos_old;
    static var_type restitution;
    static IntegrationMethod integration_method;
    
    Ball(Vector2<T> pos, Vector2<T> vel, Vector2<T> acc,
        T radius, T mass_inverse, T elasticity);
    Ball();
    
    //void integrate(const var_type duration);
    void force_add(Vector2<T> force);  // adds force to force_accumulator
    // sequential collision checking
    void collides_ball(Ball& ball);  // delete?
    void collides_edge(const Edge<T>& edge);
    void integrate_explicit_euler(const T duration);
    void integrate_implicit_euler(const T duration);
    void integrate_explicit_midpoint(const T duration);
private:
};

template <typename T>
class Edge {
public:
    Vector2<T> start, end;
    Vector2<T> normal, tangent;  // derived from start and end. calc's stored here.

    Edge(Vector2<T> start, Vector2<T> end);
};

template <typename T = var_type>
class Collision {
public:
    static Vector2<T> reflect(const Vector2<T>& v, const Edge<T>& edge);
};

template <typename T = var_type>
class ParticleForceGenerator {
public:
    virtual ~ParticleForceGenerator();
    virtual void update(Ball<T>* ball, T duration) = 0; 
    virtual void draw(Ball<T>* ball) = 0;
};

template <typename T = var_type>
class SpringForceGenerator : public ParticleForceGenerator<T> {
private:
    Ball<T>* other_ball;
    T rest_length;
    T spring_constant;
public:
    SpringForceGenerator(Ball<T>* other_ball, T rest_length, T spring_constant);
    virtual void update(Ball<T>* ball, T duration);
    virtual void draw(Ball<T>* ball);
};

template <typename T = var_type>
class SpringAnchoredForceGenerator : public ParticleForceGenerator<T> {
private:
    Vector2<T> anchor_position;
    T rest_length;
    T spring_constant;
public:
    SpringAnchoredForceGenerator(Vector2<T> anchor_position, T rest_length, T spring_constant);
    virtual void update(Ball<T>* ball, T duration); // TODO maybe duration should have diff type??
    virtual void draw(Ball<T>* ball);
};

template <typename T = var_type>
class BungeeForceGenerator : public ParticleForceGenerator<T> {
private:
    Ball<T>* other_ball;  // anchor could be attached to another ball?
    T rest_length;
    T spring_constant;
public:
    BungeeForceGenerator(Ball<T>* other_ball, T rest_length, T spring_constant);
    virtual void update(Ball<T>* ball, T duration);
    virtual void draw(Ball<T>* ball);
};

template <typename T = var_type>
class BungeeAnchoredForceGenerator : public ParticleForceGenerator<T> {
private:
    Vector2<T> anchor_position;  // anchor could be attached to another ball?
    T rest_length;
    T spring_constant;
public:
    BungeeAnchoredForceGenerator(Vector2<T> anchor_position, T rest_length, T spring_constant);
    virtual void update(Ball<T>* ball, T duration);
    virtual void draw(Ball<T>* ball);
};

template <typename T = var_type>
class GravityForceGenerator : public ParticleForceGenerator<T> { 
private:
    Vector2<T> force;
public:
    GravityForceGenerator(Vector2<T> force);
    virtual void update(Ball<T>* ball, T duration);
    virtual void draw(Ball<T>* ball);

};

template <typename T = var_type>
class ParticleForceRegistry {
private:
    struct ForceRegistration
    {
        //TODO have ball just be another member of ParticleForceGenerator derived types?
        Ball<T>* ball;  
        ParticleForceGenerator<T>* force_generator;
    };
    std::vector<ForceRegistration> registrations;

public:
    ParticleForceRegistry();
    ~ParticleForceRegistry();
    void update_all(T duration);
    void draw_all();
    void add(Ball<T>* ball, ParticleForceGenerator<T>* fg);
    void remove(Ball<T>* ball, ParticleForceGenerator<T>* fg);
    void clear();
};


//start using this to resolve collisions and other hard constraints
template <typename T = var_type>
class Contact {
public:
    Contact();
    Contact(Ball<T>* b0, Ball<T>* b1, T penetration, T restitution);
    Ball<T>* ball[2];
    Vector2<T> contact_normal;  // Contact normal
    T restitution;
    T penetration;
    Vector2<T> movement[2];
    T seperating_speed;  // save result of seperating_speed_calculate 
    T seperating_speed_calculate() const;
    T interpenetration_calculate() const;
    void resolve(T duration);
protected:
private:
    void resolve_interpenetration(T duration);  
    void resolve_velocity(T duration);
};


template <typename T = var_type>
class ContactResolver { 
protected:
    unsigned int iterations_max;
    unsigned int iterations_count;
public:
    ContactResolver(unsigned int iterations_max);
    // for fast look up of effected contacts
    std::unordered_map<Contact<T>*, std::vector<Contact<T>*>> contact_map;
    //void set_iterations(unsigned int iterations_max);
    void resolve_contacts(std::vector<Contact<T>>& contacts, T duration);
    int iterate_over_list_count;
};

template <typename T = var_type>
class ContactGenerator {
public:
    virtual unsigned int generate_contact(std::vector<Contact<T>>& contacts, unsigned limit) = 0;
    virtual void draw() = 0;
    // constraints have 1 particle connected to an anchor point. if the user is 
    // controlling the position of a ball connected to a constraint, then the contact will
    // never be resolved and it can screw up the contact resolver
    virtual bool disable_generation(Ball<T>* ball) const = 0; 
    virtual ~ContactGenerator();
    //virtual var_type length_current_calculate() const = 0;
};

template <typename T = var_type>
class ParticleLink : public ContactGenerator<T> {
public:
    Ball<T>* balls[2];
    bool disable_generation(Ball<T>* _ball) const;
protected:
    ParticleLink(Ball<T>* ball0, Ball<T>* ball1);
    T length_current_calculate() const;
};

template <typename T = var_type>
class ParticleConstraint : public ContactGenerator<T> {
public:
    Ball<T>* ball;
    Vector2<T> anchor;
    bool disable_generation(Ball<T>* _ball) const;
protected:
    ParticleConstraint(Ball<T>* ball, Vector2<T> anchor);
    T length_current_calculate() const;
};

template <typename T = var_type>
class RodLink : public ParticleLink<T> {
public:
    T length;
    RodLink(T length, Ball<T>* ball0, Ball<T>* ball1);
    virtual unsigned int generate_contact(std::vector<Contact<T>>& contacts, unsigned limit);
    virtual void draw();
};

template <typename T = var_type>
class RodConstraint : public ParticleConstraint<T> {
public:
    T length;
    RodConstraint(T length, Ball<T>* ball0, Vector2<T> anchor);
    virtual unsigned int generate_contact(std::vector<Contact<T>>& contacts, unsigned limit);
    virtual void draw();
};

template <typename T = var_type>
class CableLink : public ParticleLink<T> {
public:
    T length_max;
    CableLink(T length_max, Ball<T>* ball0, Ball<T>* ball1);
    virtual unsigned int generate_contact(std::vector<Contact<T>>& contacts, unsigned limit);
    virtual void draw();
};

//class
template <typename T = var_type>
class CableConstraint : public ParticleConstraint<T> {
public:
    T length_max;
    CableConstraint(T length_max, Ball<T>* ball, Vector2<T> anchor);
    virtual unsigned int generate_contact(std::vector<Contact<T>>& contacts, unsigned limit);
    virtual void draw();
};

//class PhysicsWorld {
//public:
//    std::vector<Ball> balls;                       
//    std::vector<Edge> edges;                                      
//    std::vector<Ball> balls_fake;
//    std::vector<Contact> contacts;
//    std::vector<ContactGenerator*> contact_generators;
//    Ball* ball_mouseover;
//    Ball* ball_selected;
//    var_type ball_selected_mass_inverse;
//    //Ball ball_selected_concrete;             
//    ParticleForceRegistry particle_force_registry; 
//    ContactResolver contact_resolver;
//
//    PhysicsWorld();
//    PhysicsWorld(std::string file_name);
//    ~PhysicsWorld();
//
//};

template <typename T = var_type>
void spring_draw(Vector2<T> start, Vector2<T> end, T rest_length);




//start

extern template class Ball<double>;
extern template class Edge<double>;
extern template class Collision<double>;
extern template class ParticleForceGenerator<double>;
extern template class SpringForceGenerator<double>;
extern template class SpringAnchoredForceGenerator<double>;
extern template class BungeeForceGenerator<double>;
extern template class BungeeAnchoredForceGenerator<double>;
extern template class GravityForceGenerator<double>;
extern template class ParticleForceRegistry<double>;
extern template class Contact<double>;
extern template class ContactResolver<double>;
extern template class ContactGenerator<double>;
extern template class ParticleLink<double>;
extern template class ParticleConstraint<double>;
extern template class RodLink<double>;
extern template class RodConstraint<double>;
extern template class CableLink<double>;
extern template class CableConstraint<double>;
extern template void spring_draw<double>(Vector2<double> start, Vector2<double> end, double rest_length);

extern template class Ball<float>;
extern template class Edge<float>;
extern template class Collision<float>;
extern template class ParticleForceGenerator<float>;
extern template class SpringForceGenerator<float>;
extern template class SpringAnchoredForceGenerator<float>;
extern template class BungeeForceGenerator<float>;
extern template class BungeeAnchoredForceGenerator<float>;
extern template class GravityForceGenerator<float>;
extern template class ParticleForceRegistry<float>;
extern template class Contact<float>;
extern template class ContactResolver<float>;
extern template class ContactGenerator<float>;
extern template class ParticleLink<float>;
extern template class ParticleConstraint<float>;
extern template class RodLink<float>;
extern template class RodConstraint<float>;
extern template class CableLink<float>;
extern template class CableConstraint<float>;
extern template void spring_draw<float>(Vector2<float> start, Vector2<float> end, float rest_length);

extern template void spring_draw<int>(Vector2<int> start, Vector2<int> end, int rest_length);



















#endif
