#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include "vector.hpp"
#include "physics.hpp"
#include "sdl_init.hpp" // this allows draw functions to be called

//var_type Ball<var_type>::restitution = 1.0;
template<> var_type Ball<var_type>::restitution = 1.0f;
//Ball::IntegrationMethod Ball::integration_method = Ball::IntegrationMethod::explicit_euler; 

template<>
Ball<var_type>::IntegrationMethod Ball<var_type>::integration_method = Ball<var_type>::IntegrationMethod::explicit_euler; 

//template<T> void spring_draw(Vector2<T> start, Vector2<T> end, T rest_length);

template <typename T>
Ball<T>::Ball() { }

template <typename T>
Ball<T>::Ball(Vector2<T> pos, Vector2<T> vel, Vector2<T> acc,
        T radius, T mass_inverse, T elasticity) 
        : pos(pos), vel(vel), acc(acc),
          radius(radius), mass_inverse(mass_inverse), elasticity(elasticity) 
{
    pos_old = this->pos;
    force_accumulator = Vector2<T>();
}

template <typename T>
void Ball<T>::integrate_explicit_euler(T duration) {
    pos_old = pos;
    pos = pos + duration * vel;
    vel = vel + duration * (acc + mass_inverse * force_accumulator);//acc;
}

template <typename T>
void Ball<T>::integrate_implicit_euler(T duration) {
    pos_old = pos;
    //force_accumulator = Vector2(0.0, 0.0);
    vel = vel + duration * (acc + mass_inverse * force_accumulator);//acc;
    pos = pos + duration * vel;
}

template <typename T>
void Ball<T>::integrate_explicit_midpoint(T duration) {
    pos_old = pos;
    Vector2 k1 = duration * vel;
    // predicted next position (predictor - corrector method, looks like explicit euler);
    //Vector2 pos_next_pred = pos + k1;  
    //TODO fix this
    //force_accumulator = Vector2(0.0, 0.0);
    Vector2 k2 = vel + duration * (acc + mass_inverse * force_accumulator);
    pos = pos + (k1 + k2) / 2.0;
    //vel = (k1 + k2) / 2.0;
    vel = k2 / duration;
}

template <typename T>
void Ball<T>::collides_ball(Ball<T>& b) {
    //TODO ball-ball collisions cause change in energy
    // check if in collision, then immediately fix velocity and position
    //Ball& a = this;
    // ball b is the objecting
    Vector2 pos_relative = b.pos - pos;
    T penetration = radius + b.radius - pos_relative.magnitude();
     
    if (penetration > 0) {
        // objects are penetrating
        Vector2<T> new_vel{};//(0.0, 0.0);
        Vector2<T> b_new_vel{};//(0.0, 0.0);
        T mass = 1.0/mass_inverse;
        T b_mass = 1.0/b.mass_inverse;
        T mass_sum = mass + b_mass;
        T dot_0 = Vector2<T>::dot( vel - b.vel, pos - b.pos);
        T dot_1 = Vector2<T>::dot( b.vel - vel, b.pos - pos);
        T distance_squared = pow(pos_relative.magnitude(), 2.0);
        new_vel = vel - (2.0 * b_mass / mass_sum) * (dot_0 / distance_squared) * (pos - b.pos); 
        b_new_vel = b.vel - (2.0 * mass / mass_sum) * (dot_1 / distance_squared) * (b.pos - pos); 
        //new_vel = (mass * vel + b_mass * b.vel - b_mass * restitution * vel
        //    + b_mass * restitution * b.vel) / ( mass + b_mass);
        //b_new_vel = (mass * vel + b_mass * b.vel - mass * restitution * b.vel
        //    + mass * restitution * vel) / ( mass + b_mass);
        // assign new velocities
        vel = new_vel;
        b.vel = b_new_vel;
        // solve interpenetrations
        Vector2<T> delta   = b_mass * vel.unit() / mass_sum;
        Vector2<T> delta_b = mass * b.vel.unit() / mass_sum;
        pos += delta;
        b.pos += delta_b;
        //std::cout << "howdy" << penetration << std::endl;
    }
    else {
        //std::cout << "pen: " << penetration << std::endl;
    }
}

template <typename T>
void Ball<T>::collides_edge(const Edge<T>& edge) {

    // P S E are 2d Vectors (Vec2)
    // P = center of ball
    // S = start of edge
    // E = end of edge
    //Vec2 PS = Vec2_sub(&edge->start, &ball->pos); 
    Vector2<T> PS = this->pos - edge.start;   //Vec2_sub(&ball->pos, &edge->start); 
    Vector2<T> SE = edge.end - edge.start; //edge.start - edge.end;   //Vec2_sub(&edge->end, &edge->start);

    // will find the SE.x, SE.SE.x, SE.y point on the edge to the ball
    T dot_product = PS.dot(SE);  //Vec2_dot_product(&PS, &SE);
    T SE_magnitude = SE.magnitude();  //Vec2_magnitude(&SE);
    // normalize t between 0 and 1
    T t = fmax(0.0, fmin(SE_magnitude, dot_product / SE_magnitude));
    t = t / SE_magnitude;
    //std::cout << "t: " << t << std::endl;
    Vector2<T> closest_point = t * SE; //Vec2_scale(&SE, t);
    closest_point = closest_point + edge.start;  //Vec2_add(&closest_point, &edge->start); // get global position
    //printf("closest circ = %f, %f", closest_point.x, closest_point.y);
    Vector2<T> penetration_vector = closest_point - pos;
    T penetration = radius - penetration_vector.magnitude();
    if (penetration > 0) {
        // this will correct the velocity, but not position      
        //Ball_resolve_collision(ball, &ball_edge);  
        Vector2 v_reflected = Collision<T>::reflect(vel, edge);
        vel = v_reflected;

        // correct position
        Vector2 delta = penetration * v_reflected.unit();
        pos = pos + delta;
    }
}

template <typename T>
Edge<T>::Edge(Vector2<T> start, Vector2<T>  end) : start(start), end(end)
{
    Vector2<T> temp = end - start;  // temp points from start to end
    tangent = temp.unit(); 
    normal = tangent.perpendicular();
}

template <typename T>
Vector2<T> 
Collision<T>::reflect(const Vector2<T>& a, const Edge<T>& edge)
{
    T v_dot_t = a.dot(edge.tangent); //(Vec2_dot_product(a, &edge->tangent));
    T v_dot_n = a.dot(edge.normal);//(Vec2_dot_product(a, &edge->normal));
    Vector2<T> normal = edge.normal;  // might be adjusted 
    Vector2<T> tangent = edge.tangent;  // might be adjusted
    if (v_dot_n > 0) {
        v_dot_n = -1.0 * v_dot_n;
        normal = - edge.normal; //Vec2_scale(&normal, -1.0);
    }
    if (v_dot_t < 0) {
        v_dot_t = -1.0 * v_dot_t;
        tangent = -1.0 * tangent; //Vec2_scale(&tangent, -1.0);
    }
    //printf("--- v_dot_t and n: %f, %f\n", v_dot_t, v_dot_n);
    Vector2<T> v_dot_t_vec = v_dot_t * tangent;  //Vec2_scale(&tangent, v_dot_t);
    Vector2<T> v_dot_n_vec = v_dot_n * normal ;  //Vec2_scale(&normal, v_dot_n);
    //printf("--- v_dot_t_vec and n: %f, %f\n", v_dot_t_vec, v_dot_n_vec);
    Vector2<T> result = v_dot_t_vec - v_dot_n_vec; //Vec2_sub(&v_dot_t_vec, &v_dot_n_vec);
    //printf("--- result: %f, %f\n", result.x, result.y);
    return result;
}

template <typename T>
ParticleForceGenerator<T>::~ParticleForceGenerator() 
{

}

template <typename T>
SpringForceGenerator<T>::SpringForceGenerator(Ball<T>* other_ball, T rest_length, T spring_constant)
    : other_ball(other_ball), rest_length(rest_length), spring_constant(spring_constant) { }

template <typename T>
void SpringForceGenerator<T>::update(Ball<T>* ball, T duration)
{
    // get vector representing relative position of string
    Vector2<T> spring_vector = ball->pos - other_ball->pos;
    // find current length of spring
    T spring_length = spring_vector.magnitude();
    // find force generated on ball
    Vector2<T> force = - spring_constant * ( spring_length - rest_length) * spring_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;  //TODO should this force be halved?
    // add opposite force to other ball
    other_ball->force_accumulator -= force;  //TODO should this force be halved?
}

template <typename T>
void SpringForceGenerator<T>::draw(Ball<T>* ball)
{
   //void spring_draw(Vector2 start, Vector2 end, var_type rest_length);
   spring_draw(ball->pos, other_ball->pos, rest_length);
}

template <typename T>
SpringAnchoredForceGenerator<T>::SpringAnchoredForceGenerator(Vector2<T> anchor_position, T rest_length, T spring_constant) : anchor_position(anchor_position), rest_length(rest_length), spring_constant(spring_constant) { }

template <typename T>
void SpringAnchoredForceGenerator<T>::update(Ball<T>* ball, T duration)
{
    // get vector representing relative position of string
    Vector2<T> spring_vector = ball->pos - anchor_position;
    // find current length of spring
    T spring_length = spring_vector.magnitude();
    // find force generated on ball
    Vector2 force = - spring_constant * ( spring_length - rest_length) * spring_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;
}

template <typename T>
void SpringAnchoredForceGenerator<T>::draw(Ball<T>* ball)
{
    // maybe draw square where anchor is
    spring_draw(ball->pos, anchor_position, rest_length);
}

template <typename T>
GravityForceGenerator<T>::GravityForceGenerator(Vector2<T> force) : force(force) { }

template <typename T>
void GravityForceGenerator<T>::update(Ball<T>* ball, T duration)
{
    ball->force_accumulator += force;
}

template <typename T>
void GravityForceGenerator<T>::draw(Ball<T>* ball) { }

template <typename T>
BungeeForceGenerator<T>::BungeeForceGenerator(Ball<T>* other_ball, T rest_length, T spring_constant) : other_ball(other_ball), rest_length(rest_length), spring_constant(spring_constant) {}

template <typename T>
void BungeeForceGenerator<T>::update(Ball<T>* ball, T duration)
{
    // get vector representing relative position of strinunsigned
    Vector2<T> bungie_vector = ball->pos - other_ball->pos;
    // find current length of spring
    T bungie_length = bungie_vector.magnitude();
    // 
    if (bungie_length < rest_length) return;  // no force applied when bungie isn't extended
    // find force generated on ball
    Vector2<T> force = - spring_constant * ( bungie_length - rest_length) * bungie_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += 0.5 * force;
    other_ball->force_accumulator += 0.5 * force;
}

template <typename T>
void BungeeForceGenerator<T>::draw(Ball<T>* ball) 
{
    T bungie_length = (ball->pos - other_ball->pos).magnitude();
    if (bungie_length < rest_length) {
        SDL_SetRenderDrawColor(gsdl.renderer, 200, 0, 0, 0);
    } else {
        SDL_SetRenderDrawColor(gsdl.renderer, 0, 200, 0, 0);
    }
    SDL_RenderDrawLine(gsdl.renderer, ball->pos.x, ball->pos.y, 
                       other_ball->pos.x, other_ball->pos.y);
}

template <typename T>
BungeeAnchoredForceGenerator<T>::BungeeAnchoredForceGenerator(Vector2<T> anchor_position, T rest_length, T spring_constant) : anchor_position(anchor_position), rest_length(rest_length), spring_constant(spring_constant) { }

template <typename T>
void BungeeAnchoredForceGenerator<T>::update(Ball<T>* ball, T duration)
{
    // get vector representing relative position of strinunsigned
    Vector2<T> bungie_vector = ball->pos - anchor_position;
    // find current length of spring
    T bungie_length = bungie_vector.magnitude();
    // 
    if (bungie_length < rest_length) return;  // no force applied when bungie isn't extended
    // find force generated on ball
    Vector2<T> force = - spring_constant * ( bungie_length - rest_length) * bungie_vector.unit(); 
    // add force to ball->force_accumulator
    ball->force_accumulator += force;
}

template <typename T>
void BungeeAnchoredForceGenerator<T>::draw(Ball<T>* ball) 
{
    T bungie_length = (ball->pos - anchor_position).magnitude();
    if (bungie_length < rest_length) {
        SDL_SetRenderDrawColor(gsdl.renderer, 200, 0, 0, 0);
    } else {
        SDL_SetRenderDrawColor(gsdl.renderer, 0, 200, 0, 0);
    }
    SDL_RenderDrawLine(gsdl.renderer, ball->pos.x, ball->pos.y, 
                       anchor_position.x, anchor_position.y);
}

template <typename T>
ParticleForceRegistry<T>::ParticleForceRegistry()
{
    std::cout << "ParticleForceRegistry() called\n";
    registrations.reserve(1000);
}

template <typename T>
ParticleForceRegistry<T>::~ParticleForceRegistry()
{
    this->clear(); 
}

template <typename T>
void ParticleForceRegistry<T>::update_all(T duration)
{
    for(auto& registry : registrations) {
        registry.force_generator->update(registry.ball, duration);
    }
}

template <typename T>
void ParticleForceRegistry<T>::draw_all()
{
    for(auto& registry : registrations) {
        registry.force_generator->draw(registry.ball);
    }
}

template <typename T>
void ParticleForceRegistry<T>::add(Ball<T>* ball, ParticleForceGenerator<T>* fg)
{
    ParticleForceRegistry::ForceRegistration entry;
    entry.ball = ball;
    entry.force_generator = fg;
    // 
    //TODO check size, and print error message if dynamic array reallocates
    //
    registrations.emplace_back(entry);
}

template <typename T>
void ParticleForceRegistry<T>::remove(Ball<T>* ball, ParticleForceGenerator<T>* fg)
{
    // not needed now
    // TODO
}

template <typename T>
void ParticleForceRegistry<T>::clear()
{
    for(ForceRegistration& fr : registrations) {
        delete fr.force_generator;
    }
    registrations.clear();
}

template <typename T>
Contact<T>::Contact() {
}

template <typename T>
Contact<T>::Contact(Ball<T>* b0, Ball<T>* b1, T penetration, T restitution) 
        /*: ball{b0, b1}, penetration(penetration), restitution(restitution)*/ {
    
    ball[0] = b0;
    ball[1] = b1;
    this->penetration = penetration;
    restitution = 1.0;  // TODO make variable later?
    contact_normal = (ball[0]->pos - ball[1]->pos).unit();
    movement[0] = Vector2<T>(); //Vector2(0.0, 0.0);
    movement[1] = Vector2<T>(); //Vector2(0.0, 0.0);
    seperating_speed = seperating_speed_calculate();
}

template <typename T>
void Contact<T>::resolve(T duration)
{
    resolve_velocity(duration);
    resolve_interpenetration(duration);
}
 
template <typename T>
T Contact<T>::seperating_speed_calculate() const
{
    Vector2<T> vel_rel = ball[0]->vel;
    if (ball[1]) vel_rel = vel_rel - ball[1]->vel;
    return Vector2<T>::dot(contact_normal, vel_rel);
}

template <typename T>
T Contact<T>::interpenetration_calculate() const
{
    return 0; //TODO FINISH    
}

template <typename T>
void Contact<T>::resolve_interpenetration(T duration)
{
    if (penetration < 0.0) return;
    var_type mass_inverse_total = ball[0]->mass_inverse;
    if (ball[1]) mass_inverse_total += ball[1]->mass_inverse;
    if (mass_inverse_total <= 0.0) return; // impluses won't effect two immovable objects
    Vector2<T> mass_penetration = (penetration / mass_inverse_total) * contact_normal;
    movement[0] = ball[0]->mass_inverse * mass_penetration;
    if (ball[1] && ball[1]->mass_inverse != 0.0) {
        movement[1] = -ball[1]->mass_inverse * mass_penetration;
    } else {
        movement[1] = Vector2<T>(0, 0);  // needs to be zeroed, might be float type?
    }    
    // apply movements
    ball[0]->pos += movement[0];
    if (ball[1] && ball[1]->mass_inverse != 0.0) ball[1]->pos += movement[1];
    // clearing movements, probably not necessary
}

template <typename T>
void Contact<T>::resolve_velocity(T duration)
{
    T seperating_speed = seperating_speed_calculate(); // TODO, segfault here
    if (seperating_speed > 0.0) return;

    // find new seperating velocity
    T new_seperating_speed = -1.0 * restitution * seperating_speed;
    // find velocity due to acceleration this frame 
    Vector2<T> vel_from_acc = ball[0]->acc;
    if (ball[1] && ball[1]->mass_inverse != 0.0) vel_from_acc -= ball[1]->acc;
    T sep_speed_from_acc = duration * Vector2<T>::dot(contact_normal, vel_from_acc);

    // did acceleration contribute to a closing a velocity? if so, subtract it.
    if (sep_speed_from_acc < 0.0) {
        new_seperating_speed += restitution * sep_speed_from_acc;
        if (new_seperating_speed < 0.0) new_seperating_speed = 0.0;
    }

    T vel_delta = new_seperating_speed - seperating_speed;

    T mass_inverse_total = ball[0]->mass_inverse;
    if (ball[1]) mass_inverse_total += ball[1]->mass_inverse;

    if (mass_inverse_total <= 0.0) return; // impluses won't effect two immovable objects
    T impulse = vel_delta / mass_inverse_total;
    Vector2<T> impulse_per_mass_inverse = impulse * contact_normal;  

    ball[0]->vel += ball[0]->mass_inverse * impulse_per_mass_inverse;
    if (ball[1] && ball[1]->mass_inverse != 0.0) ball[1]->vel += - ball[1]->mass_inverse * impulse_per_mass_inverse;

}

template <typename T>
ContactResolver<T>::ContactResolver(unsigned int iterations_max) : iterations_max(iterations_max)
{ 
    using std::array, std::unordered_map;
    iterations_count = 0;
    iterate_over_list_count = 10;
    contact_map = {};
}    

template <typename T>
void ContactResolver<T>::resolve_contacts(std::vector<Contact<T>>& contacts, T duration)
{
    // Create Contact to Contact hash table (unordered_map).  When a balls position changes
    // after a resolve(...) call, the penetration associated with this ball (if it was involved
    // in other Contacts) will need to be updated
    
    // fill in contact_map... key is Contact*, value is Contact* array.  The array
    // of pointers tells us which contacts share balls
    contact_map.clear();
    for (int i = 0; i < (int)contacts.size() - 1; ++i) {
        for (int j = i + 1; j < (int)contacts.size(); ++j) {
            if (contacts[i].ball[0] == contacts[j].ball[0]) {
                contact_map[&contacts[i]].push_back(&contacts[j]);
                contact_map[&contacts[j]].push_back(&contacts[i]);
                continue;
            } else if (contacts[i].ball[0] && contacts[j].ball[1] ) {
                if (contacts[i].ball[0] == contacts[j].ball[1]) {
                    contact_map[&contacts[i]].push_back(&contacts[j]);
                    contact_map[&contacts[j]].push_back(&contacts[i]);
                    continue;
                }
            } else if (contacts[i].ball[1] && contacts[j].ball[0]) {
                if (contacts[i].ball[1] == contacts[j].ball[0]) {
                    contact_map[&contacts[i]].push_back(&contacts[j]);
                    contact_map[&contacts[j]].push_back(&contacts[i]);
                    continue;
                }
            } else if (contacts[i].ball[1] && contacts[j].ball[1]) { if (contacts[i].ball[1] == contacts[j].ball[1]) {
                    contact_map[&contacts[i]].push_back(&contacts[j]);
                    contact_map[&contacts[j]].push_back(&contacts[i]);
                    continue;
                }
            }
        }
    }
    for (int i = 0; i < (int)contacts.size(); ++i) {
        contact_map[&contacts[i]].push_back(&contacts[i]);
    }
    
    iterations_max =  iterate_over_list_count *  contacts.size(); 
    iterations_count = 0;
   
    //find lowest seperating velocity first
    T minimum = std::numeric_limits<T>::max();
    // find most negative seperating speed from all contacts
    Contact<T>* contact_min = nullptr;  // contact minimum seperating speed (highest closing speed)
    while (iterations_count < iterations_max) { 
        minimum = std::numeric_limits<var_type>::max();
        contact_min = nullptr;
        for (Contact<T>& contact : contacts) {
            if(contact.seperating_speed < minimum && 
                    (contact.seperating_speed < 0 || contact.penetration > 0) ) {
                minimum = contact.seperating_speed;
                contact_min = &contact;
            }
        }
        if (contact_min == nullptr) {
            return;
        }
        //// resolve contact
        contact_min->resolve(duration);

        for (Contact<T>* contact1 : contact_map[contact_min]) {
            //if (contact1 == contact_min) { continue; }
            Contact<T>& contact = *contact1;
            if (contact.ball[0] == contact_min->ball[0] ) {
                contact.penetration -= contact_min->movement[0].dot(contact.contact_normal);
            } else if (contact.ball[0] == contact_min->ball[1]) {
                contact.penetration -= contact_min->movement[1].dot(contact.contact_normal);
            }
            if (contact_min->ball[1]) {
                if (contact.ball[1] == contact_min->ball[0]) {
                    contact.penetration += contact_min->movement[0].dot(contact.contact_normal);
                } else if (contact.ball[1] == contact_min->ball[1]) {
                    contact.penetration += contact_min->movement[1].dot(contact.contact_normal);
                }
            }
            //// important, recalculating seperating speed of Contact
            contact.seperating_speed = contact.seperating_speed_calculate();
        }

        // above not correctly updating all seperating speeds. Correction is made below.
        for (Contact<T>& c : contacts) {
            c.seperating_speed = c.seperating_speed_calculate();
        }
         
        iterations_count++;
    }
}

//void resolve_contacts_old(std::vector<Contact>& contacts, var_type duration)
//{
//    if (contacts.size() > 0) {
//        //std::cout << contacts.size() << std::endl;
//    }
//    int iterations_max =  5 *  contacts.size(); 
//    iterations_count = 0;
//    while (iterations_count < iterations_max) {
//        var_type minimum = std::numeric_limits<var_type>::max();
//        unsigned int minimum_index = 0;
//        var_type speed_sep = 0.0;
//        // find most negative seperating speed from all contacts
//        for(size_t i = 0; i < contacts.size(); ++i) { //    for(auto& contact : contacts) {
//            speed_sep = contacts[i].seperating_speed_calculate(); 
//            if (speed_sep < minimum) {
//                minimum       = speed_sep;
//                minimum_index = i;
//            }
//        }
//        // if all contacts are seperating (speed >= 0.0) then all contacts are resolved.
//        if (minimum >= 0.0 ) return;
//        // resolve contact
//        contacts[minimum_index].resolve(duration);
//        // update interpenetration variable in contact list
//        for(auto& contact : contacts) {
//            
//        }
//         
//        iterations_count++;
//    }
//}

template <typename T>
ContactGenerator<T>::~ContactGenerator() { }

// C++ doesn't like array initializer syntax here
//ParticleLink::ParticleLink(Ball* _ball0, Ball* _ball1) : balls[0](_ball0), balls[1](_ball1)
template <typename T>
ParticleLink<T>::ParticleLink(Ball<T>* _ball0, Ball<T>* _ball1) 
{  
    balls[0] = _ball0;
    balls[1] = _ball1;
}

template <typename T>
T ParticleLink<T>::length_current_calculate() const
{
   return (balls[1]->pos - balls[0]->pos).magnitude();
}

template <typename T>
bool ParticleLink<T>::disable_generation(Ball<T>* _ball) const
{
    return false;
}

template <typename T>
ParticleConstraint<T>::ParticleConstraint(Ball<T>* _ball, Vector2<T> _anchor)
    : ball(_ball), anchor(_anchor) {}

template <typename T>
T ParticleConstraint<T>::length_current_calculate() const
{
    return (anchor - ball->pos).magnitude();
}

template <typename T>
bool ParticleConstraint<T>::disable_generation(Ball<T>* _ball) const
{
    return (ball == _ball);
}

template <typename T>
RodLink<T>::RodLink(T _length, Ball<T>* _ball0, Ball<T>* _ball1) 
    : ParticleLink<T>(_ball0, _ball1), length(_length) {}

template <typename T>
unsigned int RodLink<T>::generate_contact(std::vector<Contact<T>>& contacts, unsigned limit) 
{
    T length_current = this->length_current_calculate();
    if (length_current == length) return 0; // TODO maybe always generate contact?
    using std::cout, std::endl;
    //cout << "howdy partner" << endl;
    Contact<T> c;
    c.ball[0] = this->balls[0];
    c.ball[1] = this->balls[1];
    //TODO restitution at 1.0 ensures rod (when spun manually) doesn't excede max length.
    // But once let go, the rod spins wildly. At restitution 0.0, when manually spun, rod can
    // excede max length. But when let go, doesn't spin wildly.
    c.restitution = 0.0;  // no bouncing
    c.contact_normal = (c.ball[0]->pos - c.ball[1]->pos).unit(); 
    c.movement[0] =  Vector2<T>();//Vector2(0.0, 0.0);
    c.movement[1] =  Vector2<T>();//Vector2(0.0, 0.0);
    if (length_current < length) {  //compression
        // objects are too close, spread them.
        c.penetration = length - length_current;
    } else {  //extension
        c.penetration = length_current - length; 
        c.contact_normal *= -1.0;
    }
    contacts.emplace_back(c);
    return 1;
}

template <typename T>
void RodLink<T>::draw()
{
    SDL_SetRenderDrawColor(gsdl.renderer, 128, 128, 128, 0);
    SDL_RenderDrawLine(gsdl.renderer, this->balls[0]->pos.x, this->balls[0]->pos.y, 
                      this->balls[1]->pos.x, this->balls[1]->pos.y);
}









template <typename T>
RodConstraint<T>::RodConstraint(T _length, Ball<T>* _ball0, Vector2<T> _anchor) 
    : ParticleConstraint<T>(_ball0, _anchor), length(_length) {}

template <typename T>
unsigned int RodConstraint<T>::generate_contact(std::vector<Contact<T>>& contacts, unsigned limit) 
{
    T length_current = this->length_current_calculate();
    if (length_current == length) return 0; // TODO maybe always generate contact?
    using std::cout, std::endl;
    //cout << "howdy partner" << endl;
    Contact<T> c;
    c.ball[0] = this->ball;
    c.ball[1] = nullptr; //balls[1];
    //TODO restitution at 1.0 ensures rod (when spun manually) doesn't excede max length.
    // But once let go, the rod spins wildly. At restitution 0.0, when manually spun, rod can
    // excede max length. But when let go, doesn't spin wildly.
    c.restitution = 0.0;  // no bouncing
    //c.contact_normal = (c.ball[0]->pos - c.ball[1]->pos).unit(); 
    c.contact_normal = (c.ball[0]->pos - this->anchor).unit(); 
    c.movement[0] =  Vector2<T>(0.0, 0.0);
    c.movement[1] =  Vector2<T>(0.0, 0.0);
    if (length_current < length) {  //compression
        // objects are too close, spread them.
        c.penetration = length - length_current;
    } else {  //extension
        c.penetration = length_current - length; 
        c.contact_normal *= -1.0;
    }
    contacts.emplace_back(c);
    return 1;
}

template <typename T>
void RodConstraint<T>::draw()
{
    SDL_SetRenderDrawColor(gsdl.renderer, 128, 128, 128, 0);
    SDL_RenderDrawLine(gsdl.renderer, this->ball->pos.x, this->ball->pos.y, 
                      this->anchor.x, this->anchor.y);
}














template <typename T>
CableLink<T>::CableLink(T length_max, Ball<T>* ball0, Ball<T>* ball1) 
        : ParticleLink<T>(ball0, ball1), length_max(length_max)
{
    //balls[0] = ball0;
    //balls[1] = ball1;
}

template <typename T>
unsigned int CableLink<T>::generate_contact(std::vector<Contact<T>>& contacts, unsigned limit) 
{
    T length_current = this->length_current_calculate();
    if (length_current <= length_max) return 0;
    Contact<T> c;
    c.ball[0] = this->balls[0];
    c.ball[1] = this->balls[1];
    c.restitution = 0.0; //was 0 // TODO make variable later?
    c.contact_normal = (c.ball[1]->pos - c.ball[0]->pos).unit(); //reverse the normal
    c.movement[0] =  Vector2<T>(0.0, 0.0);
    c.movement[1] =  Vector2<T>(0.0, 0.0);
    //c.contact_normal *= -1.0; // v1 - v0
    c.penetration = length_current - length_max;
    contacts.emplace_back(c);
    return 1;
}

template <typename T>
void CableLink<T>::draw()
{
    SDL_SetRenderDrawColor(gsdl.renderer, 139, 69, 19, 0);
    SDL_RenderDrawLine(gsdl.renderer, this->balls[0]->pos.x, this->balls[0]->pos.y, 
                      this->balls[1]->pos.x, this->balls[1]->pos.y);
}

template <typename T>
CableConstraint<T>::CableConstraint(T length_max, Ball<T>* ball0, Vector2<T> anchor) 
        : ParticleConstraint<T>(ball0, anchor), length_max(length_max) {}

template <typename T>
unsigned int CableConstraint<T>::generate_contact(std::vector<Contact<T>>& contacts, unsigned limit) 
{
    T length_current = this->length_current_calculate();
    if (length_current <= length_max) return 0;
    Contact<T> c;
    c.ball[0] = this->ball;
    c.ball[1] = nullptr;
    c.restitution = 0.0; // TODO make variable later?
    c.contact_normal = (this->anchor - c.ball[0]->pos).unit(); //reverse the normal
    c.movement[0] =  Vector2<T>(0.0, 0.0);
    c.movement[1] =  Vector2<T>(0.0, 0.0);
    //c.contact_normal *= -1.0; // v1 - v0
    c.penetration = length_current - length_max;
    contacts.emplace_back(c);
    return 1;
}

template <typename T>
void CableConstraint<T>::draw()
{
    SDL_SetRenderDrawColor(gsdl.renderer, 139, 19, 69, 0);
    SDL_RenderDrawLine(gsdl.renderer, this->ball->pos.x, this->ball->pos.y, 
                      this->anchor.x, this->anchor.y);
}

// private function
template <typename T>
void spring_draw(Vector2<T> start, Vector2<T> end, T rest_length)
{
    using std::max, std::min;
    // find how many "turns" are in the spring. depends only on rest_length
    int turns_count = min(10, (int)max(rest_length / 25.0, 3.0)); // max 10 turns, min 3
    // "turn" positions
    Vector2<T> turn_positions[10];
    // how far does the "turn" extend
    T turn_distance = 25.0;

    // relative position
    Vector2<T> pos_relative = end - start;
    // get current length
    T length = pos_relative.magnitude();
    // get direction
    Vector2<T> direction = pos_relative.unit();
    // 5% of the total length will be drawn as a straight wire. Instead of a turning wire.
    Vector2<T> start_turn, end_turn;
    start_turn = start + 0.025 * length * direction;
    end_turn   = end - 0.025 * length * direction;
    // get current length of turning wire
    T length_turn = length * 0.95;
    // get distance between each turn
    T length_per_turn = length_turn / ((float)turns_count + 1.0);
    // determine positions of "turns"
    T length_current = 0.0;
    Vector2<T> turn_vector = turn_distance * direction.perpendicular();
    for(int i = 0; i < turns_count; i++)
    {
       length_current += length_per_turn;
       turn_vector = -1.0 * turn_vector;
       // convert length of "walk", to Vector2 position
       Vector2<T> centered_pos = start_turn + length_current * direction;
       turn_positions[i] = centered_pos + turn_vector; 
    }
    

    //
    // draw spring from data determined above
    //
    if (length < rest_length) {
        SDL_SetRenderDrawColor(gsdl.renderer, 200, 0, 0, 0);
    } else {
        SDL_SetRenderDrawColor(gsdl.renderer, 0, 200, 0, 0);
    }
    
    //int displacement = length - rest_length;
    //if (displacement > 250) displacement = 250;
    //else if (displacement < -250) displacement = -250;
    //
    //int green = 0; int red = 0;
    //if (displacement > 0) green = green + displacement;
    //else red = red - displacement;
    //
    //SDL_SetRenderDrawColor(gsdl.renderer, red, green, 20, 255);




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


template class Ball<double>;
template class Edge<double>;
template class Collision<double>;
template class ParticleForceGenerator<double>;
template class SpringForceGenerator<double>;
template class SpringAnchoredForceGenerator<double>;
template class BungeeForceGenerator<double>;
template class BungeeAnchoredForceGenerator<double>;
template class GravityForceGenerator<double>;
template class ParticleForceRegistry<double>;
template class Contact<double>;
template class ContactResolver<double>;
template class ContactGenerator<double>;
template class ParticleLink<double>;
template class ParticleConstraint<double>;
template class RodLink<double>;
template class RodConstraint<double>;
template class CableLink<double>;
template class CableConstraint<double>;
template void spring_draw<double>(Vector2<double> start, Vector2<double> end, double rest_length);

template class Ball<float>;
template class Edge<float>;
template class Collision<float>;
template class ParticleForceGenerator<float>;
template class SpringForceGenerator<float>;
template class SpringAnchoredForceGenerator<float>;
template class BungeeForceGenerator<float>;
template class BungeeAnchoredForceGenerator<float>;
template class GravityForceGenerator<float>;
template class ParticleForceRegistry<float>;
template class Contact<float>;
template class ContactResolver<float>;
template class ContactGenerator<float>;
template class ParticleLink<float>;
template class ParticleConstraint<float>;
template class RodLink<float>;
template class RodConstraint<float>;
template class CableLink<float>;
template class CableConstraint<float>;
template void spring_draw<float>(Vector2<float> start, Vector2<float> end, float rest_length);

template void spring_draw<int>(Vector2<int> start, Vector2<int> end, int rest_length);
