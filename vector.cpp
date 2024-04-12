#include <cmath>
#include <iostream>
#include "vector.hpp"



// addition
Vector2 operator+(const Vector2& a, const Vector2& b) {
    return Vector2(a.x + b.x, a.y + b.y);
}

// subraction
Vector2 operator-(const Vector2& a, const Vector2& b) {
    return Vector2(a.x - b.x, a.y - b.y);
}

// unary minus aka negation
Vector2 operator-(const Vector2& a) {
    return Vector2(-a.x, -a.y);
}

// scalar multiplication
Vector2 operator*(const var_type a, const Vector2& b) {
    return Vector2(a * b.x, a * b.y);
}

// scalar division
Vector2 operator/(const Vector2& a, const var_type b) {
    return Vector2(a.x / b, a.y / b);
}

// addition assignment
void Vector2::operator+=(const Vector2& a) {
    x += a.x;
    y += a.y;
}

// subtraction assignment
void Vector2::operator-=(const Vector2& a) {
    x -= a.x;
    y -= a.y;
}

// multiplication assignment
void Vector2::operator*=(const var_type a) {
    x = x * a;
    y = y * a;
}


bool operator!=(const Vector2& a, const Vector2& b) {
    //return a.x != b.x || a.y != b.y; 
    return !(a.x == b.x && a.y == b.y); 
}

bool operator==(const Vector2& a, const Vector2& b) {
    return a.x == b.x && a.y == b.y; 
}

std::ostream& operator<<(std::ostream& output, const Vector2& v) {
    output << '(' << v.x << ", " << v.y << ") ";
    return output;
}

var_type Vector2::dot(const Vector2& a) const {
    var_type sum = 0.0;
    sum = x * a.x + y * a.y;
    return sum;
}

var_type Vector2::dot(const Vector2& a, const Vector2& b) {
    return a.x * b.x + a.y * b.y;     
}

var_type Vector2::magnitude() const {
    return sqrt(pow(x,2.0) + pow(y, 2.0));
}

Vector2 Vector2::unit() const {
    Vector2 result;
    var_type magnitude = this->magnitude();
    result.x = x / magnitude;
    result.y = y / magnitude;
    return result;
}

Vector2 Vector2::perpendicular() const {
    // if v isn't null, will find the vector perpendicular to (this - v)
    // maybe this shouldn't be used? and should have no parameters
    //Vector2 v0;
    //if( v == NULL ) {
    //    v0 = *this;
    //} else {
    //    v0 = *this - v;
    //}
    //v0 = this->unit();
    //Vector2 result; 
    //result.x = -v0.y;
    //result.y = v0.x;
    //return result;
    Vector2 result; 
    result.x = - this->y;
    result.y = this->x;
    return result;
}

var_type distance(const Vector2& a, const Vector2& b) {
    Vector2 pos_relative = a - b; 
    return pos_relative.magnitude();
}

