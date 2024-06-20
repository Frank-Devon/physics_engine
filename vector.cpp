#include <cmath>
#include <iostream>
#include "vector.hpp"

template <typename T>
Vector2<T>::Vector2() {
    x = T();
    y = T();
}

template <typename T>
Vector2<T>::Vector2(T a, T b): x(a), y(b) { }

// addition
template <typename T>
Vector2<T> operator+(const Vector2<T>& a, const Vector2<T>& b) {
    return Vector2(a.x + b.x, a.y + b.y);
}

//// subraction
//Vector2 operator-(const Vector2& a, const Vector2& b) {
//    return Vector2(a.x - b.x, a.y - b.y);
//}

// unary minus aka negation
template <typename T>
Vector2<T> operator-(const Vector2<T>& a) {
    return Vector2(-a.x, -a.y);
}

// scalar multiplication
template <typename T>
//Vector2<T> operator*(const T a, const T& b) {
Vector2<T> operator*(const T a, const Vector2<T>& b) {
    return Vector2(a * b.x, a * b.y);
}

// scalar division
template <typename T>
Vector2<T> operator/(const Vector2<T>& a, const T b) {
    return Vector2(a.x / b, a.y / b);
}

// addition assignment
template <typename T>
void Vector2<T>::operator+=(const Vector2& a) {
    x += a.x;
    y += a.y;
}

// subtraction assignment
template <typename T>
void Vector2<T>::operator-=(const Vector2& a) {
    x -= a.x;
    y -= a.y;
}

// multiplication assignment
template <typename T>
void Vector2<T>::operator*=(const T a) {
    x = x * a;
    y = y * a;
}

template <typename T>
bool operator!=(const Vector2<T>& a, const Vector2<T>& b) {
    //return a.x != b.x || a.y != b.y; 
    return !(a.x == b.x && a.y == b.y); 
}

template <typename T>
bool operator==(const Vector2<T>& a, const Vector2<T>& b) {
    return a.x == b.x && a.y == b.y; 
}

template <typename T>
std::ostream& operator<<(std::ostream& output, const Vector2<T>& v) {
    output << '(' << v.x << ", " << v.y << ") ";
    return output;
}

template <typename T>
T Vector2<T>::dot(const Vector2& a) const {
    T sum = 0.0;
    sum = x * a.x + y * a.y;
    return sum;
}

template <typename T>
T Vector2<T>::dot(const Vector2& a, const Vector2& b) {
    return a.x * b.x + a.y * b.y;     
}

//var_type Vector2::magnitude() const {
//    return sqrt(pow(x,2.0) + pow(y, 2.0));
//}
template <typename T>
Vector2<T> Vector2<T>::unit() const {
    Vector2 result;
    var_type magnitude = this->magnitude();
    result.x = x / magnitude;
    result.y = y / magnitude;
    return result;
}

template <typename T>
Vector2<T> Vector2<T>::perpendicular() const {
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

template <typename T>
T distance(const Vector2<T>& a, const Vector2<T>& b) {
    Vector2<T> pos_relative = a - b; 
    return pos_relative.magnitude();
}

template class Vector2<int>;
template class Vector2<float>;
template class Vector2<double>;
template Vector2<int> operator-(const Vector2<int>&);
template Vector2<float> operator-(const Vector2<float>&);
template Vector2<double> operator-(const Vector2<double>&);
template Vector2<int> operator+(const Vector2<int>&, const Vector2<int>&);
template Vector2<float> operator+(const Vector2<float>&, const Vector2<float>&);
template Vector2<double> operator+(const Vector2<double>&, const Vector2<double>&);
template bool operator==(const Vector2<int>&, const Vector2<int>&);
template bool operator==(const Vector2<float>&, const Vector2<float>&);
template bool operator==(const Vector2<double>&, const Vector2<double>&);
template Vector2<int> operator*(const int, const Vector2<int>&);   // scalar multiplication
template Vector2<float> operator*(const float, const Vector2<float>&);   // scalar multiplication
template Vector2<double> operator*(const double, const Vector2<double>&);   // scalar multiplication
