#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <limits>
#include <cmath>
#include <ostream>


typedef float var_type;
//typedef std::numeric_limits<float>::max() VAR_TYPE_MAX; //expected initializer before ‘VAR_TYPE_MAX’

template <typename T = var_type>
class Vector2 {
public:
    T x;
    T y;

    Vector2(T x0, T y0);
    Vector2();

    void    operator+=(const Vector2&); 
    void    operator-=(const Vector2&);
    void    operator*=(const T);

    T dot(const Vector2&) const; // dot product
    inline T magnitude() const {
        return sqrt(pow(x,2.0) + pow(y, 2.0));
    }
    Vector2 unit() const;
    Vector2 perpendicular() const; // get's vector perpendicular

    static T dot(const Vector2&, const Vector2&);
};

template <typename T = float>
Vector2<T> operator-(const Vector2<T>&);                   // unary minus
// subtraction
template <typename T = float>
inline Vector2<T> operator-(const Vector2<T>& a, const Vector2<T>& b) {
    return Vector2(a.x - b.x, a.y - b.y);
}
template <typename T>
Vector2<T> operator+(const Vector2<T>&, const Vector2<T>&);   // addition
template <typename T>
Vector2<T> operator*(const T, const Vector2<T>&);   // scalar multiplication
template <typename T, typename U>  // scalar multiplication with 2 different numeric types
Vector2<U> operator*(const T a, const Vector2<U>& b) {
    Vector2<U> result;
    result.x = a * b.x;
    result.y = a * b.y;
    return result;
}
//template <typename T>
//Vector2<T> operator/(const Vector2<T>&, const T);
template <typename T, typename U>
Vector2<T> operator/(const Vector2<T>& a, const U b) {
    Vector2<T> result;
    result.x = a.x / b;
    result.y = a.y / b;
    return result;
}
template <typename T>
bool    operator!=(const Vector2<T>&, const Vector2<T>&);
template <typename T>
bool    operator==(const Vector2<T>&, const Vector2<T>&);
template <typename T>
std::ostream& operator<<(std::ostream& output, const Vector2<T>&);
template <typename T>
T distance(const Vector2<T>&, const Vector2<T>&);

extern template class Vector2<int>;
extern template class Vector2<float>;
extern template class Vector2<double>;
extern template Vector2<int> operator-(const Vector2<int>&);
extern template Vector2<float> operator-(const Vector2<float>&);
extern template Vector2<double> operator-(const Vector2<double>&);
extern template Vector2<int> operator+(const Vector2<int>&, const Vector2<int>&);
extern template Vector2<float> operator+(const Vector2<float>&, const Vector2<float>&);
extern template Vector2<double> operator+(const Vector2<double>&, const Vector2<double>&);
extern template bool operator==(const Vector2<int>&, const Vector2<int>&);
extern template bool operator==(const Vector2<float>&, const Vector2<float>&);
extern template bool operator==(const Vector2<double>&, const Vector2<double>&);
extern template Vector2<int> operator*(const int, const Vector2<int>&);   // scalar multiplication
extern template Vector2<float> operator*(const float, const Vector2<float>&);   // scalar multiplication
extern template Vector2<double> operator*(const double, const Vector2<double>&);   // scalar multiplication
#endif
