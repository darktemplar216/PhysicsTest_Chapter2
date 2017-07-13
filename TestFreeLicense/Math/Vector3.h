
class Vector3
{
public:
    
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        
        float v[3];
    };
    
    /// default constructor.
    /// does nothing for speed.
    
    Vector3() { memset(v, 0, sizeof(v)); }
    
    /// construct Vector3 from x,y,z components.
    
    Vector3(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    /// set Vector3 to zero.
    
    void zero()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    
    /// negate Vector3.
    
    void negate()
    {
        x = -x;
        y = -y;
        z = -z;
    }
    
    /// add another Vector3 to this Vector3.
    
    void add(const Vector3 &Vector3)
    {
        x += Vector3.x;
        y += Vector3.y;
        z += Vector3.z;
    }
    
    /// subtract another Vector3 from this Vector3.
    
    void subtract(const Vector3 &Vector3)
    {
        x -= Vector3.x;
        y -= Vector3.y;
        z -= Vector3.z;
    }
    
    /// multiply this Vector3 by a scalar.
    
    void multiply(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
    }
    
    /// divide this Vector3 by a scalar.
    
    void divide(float scalar)
    {
        assert(scalar != 0);
        const float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
    }
    
    /// calculate dot product of this Vector3 with another Vector3.
    
    float dot(const Vector3 &Vector3) const
    {
        return x * Vector3.x + y * Vector3.y + z * Vector3.z;
    }
    
    /// calculate cross product of this Vector3 with another Vector3.
    
    Vector3 cross(const Vector3& vec) const
    {
        return Vector3(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x);
    }
    
    /// calculate cross product of this Vector3 with another Vector3, store result in parameter.
    
    void cross(const Vector3& vec, Vector3& result) const
    {
        result.x = y * vec.z - z * vec.y;
        result.y = z * vec.x - x * vec.z;
        result.z = x * vec.y - y * vec.x;
    }
    
    /// calculate length of Vector3 squared
    
    float lengthSquared() const
    {
        return x*x + y*y + z*z;
    }
    
    /// calculate length of Vector3.
    
    float length() const
    {
        return sqrt(x*x + y*y + z*z);
    }
    
    /// normalize Vector3 and return reference to normalized self.
    
    Vector3& normalize()
    {
        const float magnitude = sqrt(x*x + y*y + z*z);
        if (magnitude > 0.00001f)
        {
            const float scale = 1.0f / magnitude;
            x *= scale;
            y *= scale;
            z *= scale;
        }
        return *this;
    }
    
    float normalizeF()
    {
        const float magnitude = sqrt(x*x + y*y + z*z);
        if (magnitude > 0.00001f)
        {
            const float scale = 1.0f / magnitude;
            x *= scale;
            y *= scale;
            z *= scale;
        }
        return magnitude;
    }
    
    /// return unit length Vector3
    
    Vector3 unit() const
    {
        Vector3 Vector3(*this);
        Vector3.normalize();
        return Vector3;
    }
    
    /// test if Vector3 is normalized.
    
    bool normalized() const
    {
        return fabs(length() - 1) < 0.0001f;
    }
    
    /// equals operator
    
    bool operator ==(const Vector3 &other) const
    {
        if (fabs(x - other.x) < EPSILON_PW_FLT
            && fabs(y - other.y) < EPSILON_PW_FLT
            && fabs(z - other.z) < EPSILON_PW_FLT)
            return true;
        else
            return false;
    }
    
    void operator =(const GLKVector3& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    
    GLKVector3 toGLKVector3()
    {
        return GLKVector3Make(x, y, z);
    }
    
    /// not equals operator
    
    bool operator !=(const Vector3 &other) const
    {
        return !(*this==other);
    }
    
    float& operator [](int i)
    {
        assert(i>=0);
        assert(i<=2);
        return *(&x+i);
    }
    
    /// element access (const)
    
    const float& operator [](int i) const
    {
        assert(i>=0);
        assert(i<=2);
        return *(&x+i);
    }
    
    friend inline Vector3 operator-(const Vector3 &a);
    friend inline Vector3 operator+(const Vector3 &a, const Vector3 &b);
    friend inline Vector3 operator-(const Vector3 &a, const Vector3 &b);
    friend inline Vector3 operator*(const Vector3 &a, const Vector3 &b);
    friend inline Vector3& operator+=(Vector3 &a, const Vector3 &b);
    friend inline Vector3& operator-=(Vector3 &a, const Vector3 &b);
    friend inline Vector3& operator*=(Vector3 &a, const Vector3 &b);
    
    friend inline Vector3 operator*(const Vector3 &a, float s);
    friend inline Vector3 operator/(const Vector3 &a, float s);
    friend inline Vector3& operator*=(Vector3 &a, float s);
    friend inline Vector3& operator/=(Vector3 &a, float s);
    friend inline Vector3 operator*(float s, const Vector3 &a);
    friend inline Vector3& operator*=(float s, Vector3 &a);
    
};


inline Vector3 operator-(const Vector3 &a)
{
    return Vector3(-a.x, -a.y, -a.z);
}

inline Vector3 operator+(const Vector3 &a, const Vector3 &b)
{
    return Vector3(a.x+b.x, a.y+b.y, a.z+b.z);
}

inline Vector3 operator-(const Vector3 &a, const Vector3 &b)
{
    return Vector3(a.x-b.x, a.y-b.y, a.z-b.z);
}

inline Vector3 operator*(const Vector3 &a, const Vector3 &b)
{
    return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Vector3& operator+=(Vector3 &a, const Vector3 &b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vector3& operator-=(Vector3 &a, const Vector3 &b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Vector3& operator*=(Vector3 &a, const Vector3 &b)
{
    const float cx = a.y * b.z - a.z * b.y;
    const float cy = a.z * b.x - a.x * b.z;
    const float cz = a.x * b.y - a.y * b.x;
    a.x = cx;
    a.y = cy;
    a.z = cz;
    return a;
}

inline Vector3 operator*(const Vector3 &a, float s)
{
    return Vector3(a.x*s, a.y*s, a.z*s);
}

inline Vector3 operator/(const Vector3 &a, float s)
{
    assert(s!=0);
    return Vector3(a.x/s, a.y/s, a.z/s);
}

inline Vector3& operator*=(Vector3 &a, float s)
{
    a.x *= s;
    a.y *= s;
    a.z *= s;
    return a;
}

inline Vector3& operator/=(Vector3 &a, float s)
{
    assert(s!=0);
    a.x /= s;
    a.y /= s;
    a.z /= s;
    return a;
}

inline Vector3 operator*(float s, const Vector3 &a)
{
    return Vector3(a.x*s, a.y*s, a.z*s);
}

inline Vector3& operator*=(float s, Vector3 &a)
{
    a.x *= s;
    a.y *= s;
    a.z *= s;
    return a;
}
