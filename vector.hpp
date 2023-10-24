#ifndef VECTOR_HPP
#define VECTOR_HPP


typedef float var_type;

class Vector2 {
    public:
        var_type x;
        var_type y;

        Vector2(var_type x0, var_type y0): x{x0}, y{y0} {}
        Vector2(): x{0}, y{0} {}

        void    operator+=(const Vector2&); 
        void    operator-=(const Vector2&);
        void    operator*=(const var_type);

        var_type dot(const Vector2&) const; // dot product
        var_type magnitude() const;
        Vector2 unit() const;
        Vector2 perpendicular() const; // get's vector perpendicular

        static var_type dot(const Vector2&, const Vector2&);
};

Vector2 operator-(const Vector2&);          // unary minus
Vector2 operator-(const Vector2&, const Vector2&);   // subtraction
Vector2 operator+(const Vector2&, const Vector2&);   // addition
Vector2 operator*(const var_type, const Vector2&);  // scalar multiplication
Vector2 operator/(const Vector2&, const var_type);
#endif
