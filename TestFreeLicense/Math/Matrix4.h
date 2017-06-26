
class Matrix4
{
public:
    
    /// default constructor.
    /// does nothing for speed.
    
    Matrix4() {};
    
    /// construct a Matrix4 from three basis Vector3s.
    /// the x,y,z values from each of these basis Vector3s map to rows in the 3x3 sub Matrix4.
    /// note: the rest of the Matrix4 (row 4 and column 4 are set to identity)
    
    Matrix4(const Vector3 &a, const Vector3 &b, const Vector3 &c)
    {
        // ax ay az 0
        // bx by bz 0
        // cx cy cz 0
        // 0  0  0  1
        
        m11 = a.x;
        m12 = a.y;
        m13 = a.z;
        m14 = 0;
        m21 = b.x;
        m22 = b.y;
        m23 = b.z;
        m24 = 0;
        m31 = c.x;
        m32 = c.y;
        m33 = c.z;
        m34 = 0;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// construct a Matrix4 from explicit values for the 3x3 sub Matrix4.
    /// note: the rest of the Matrix4 (row 4 and column 4 are set to identity)
    
    Matrix4(float m11, float m12, float m13,
           float m21, float m22, float m23,
           float m31, float m32, float m33)
    {
        this->m11 = m11;
        this->m12 = m12;
        this->m13 = m13;
        this->m14 = 0;
        this->m21 = m21;
        this->m22 = m22;
        this->m23 = m23;
        this->m24 = 0;
        this->m31 = m31;
        this->m32 = m32;
        this->m33 = m33;
        this->m34 = 0;
        this->m41 = 0;
        this->m42 = 0;
        this->m43 = 0;
        this->m44 = 1;
    }
    
    /// construct a Matrix4 from explicit entry values for the whole 4x4 Matrix4.
    
    Matrix4(float m11, float m12, float m13, float m14,
           float m21, float m22, float m23, float m24,
           float m31, float m32, float m33, float m34,
           float m41, float m42, float m43, float m44)
    {
        this->m11 = m11;
        this->m12 = m12;
        this->m13 = m13;
        this->m14 = m14;
        this->m21 = m21;
        this->m22 = m22;
        this->m23 = m23;
        this->m24 = m24;
        this->m31 = m31;
        this->m32 = m32;
        this->m33 = m33;
        this->m34 = m34;
        this->m41 = m41;
        this->m42 = m42;
        this->m43 = m43;
        this->m44 = m44;
    }
    
    /// load Matrix4 from raw float array.
    /// data is assumed to be stored linearly in memory in row order, from left to right, top to bottom.
    
    Matrix4(const float data[])
    {
        this->m11 = data[0];
        this->m12 = data[1];
        this->m13 = data[2];
        this->m14 = data[3];
        this->m21 = data[4];
        this->m22 = data[5];
        this->m23 = data[6];
        this->m24 = data[7];
        this->m31 = data[8];
        this->m32 = data[9];
        this->m33 = data[10];
        this->m34 = data[11];
        this->m41 = data[12];
        this->m42 = data[13];
        this->m43 = data[14];
        this->m44 = data[15];
    }
    
    /// set all entries in Matrix4 to zero.
    
    void zero()
    {
        m11 = 0;
        m12 = 0;
        m13 = 0;
        m14 = 0;
        m21 = 0;
        m22 = 0;
        m23 = 0;
        m24 = 0;
        m31 = 0;
        m32 = 0;
        m33 = 0;
        m34 = 0;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 0;
    }
    
    /// set Matrix4 to identity.
    
    void identity()
    {
        m11 = 1;
        m12 = 0;
        m13 = 0;
        m14 = 0;
        m21 = 0;
        m22 = 1;
        m23 = 0;
        m24 = 0;
        m31 = 0;
        m32 = 0;
        m33 = 1;
        m34 = 0;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// set to a translation Matrix4.
    
    void translate(float x, float y, float z)
    {
        m11 = 1;		  // 1 0 0 x
        m12 = 0;		  // 0 1 0 y
        m13 = 0;		  // 0 0 1 z
        m14 = x;		  // 0 0 0 1
        m21 = 0;
        m22 = 1;
        m23 = 0;
        m24 = y;
        m31 = 0;
        m32 = 0;
        m33 = 1;
        m34 = z;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// set to a translation Matrix4.
    
    void translate(const Vector3 &Vector3)
    {
        m11 = 1;		  // 1 0 0 x
        m12 = 0;		  // 0 1 0 y
        m13 = 0;		  // 0 0 1 z
        m14 = Vector3.x;   // 0 0 0 1
        m21 = 0;
        m22 = 1;
        m23 = 0;
        m24 = Vector3.y;
        m31 = 0;
        m32 = 0;
        m33 = 1;
        m34 = Vector3.z;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// set to a scale Matrix4.
    
    void scale(float x, float y, float z)
    {
        m11 = x;
        m12 = 0;
        m13 = 0;
        m14 = 0;
        m21 = 0;
        m22 = y;
        m23 = 0;
        m24 = 0;
        m31 = 0;
        m32 = 0;
        m33 = z;
        m34 = 0;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// set to a diagonal Matrix4.
    
    void diagonal(float a, float b, float c, float d = 1)
    {
        m11 = a;
        m12 = 0;
        m13 = 0;
        m14 = 0;
        m21 = 0;
        m22 = b;
        m23 = 0;
        m24 = 0;
        m31 = 0;
        m32 = 0;
        m33 = c;
        m34 = 0;
        m41 = 0;
        m42 = 0;
        m43 = 0;
        m44 = d;
    }
    
    /// set to a rotation Matrix4 about a specified axis / angle.
    
    void rotate(float angle, Vector3 axis)
    {
        // note: adapted from david eberly's code with permission
        
        if (axis.lengthSquared() < 0.000001f)
        {
            identity();
        }
        else
        {
            axis.normalize();
            
            float fCos = (float) cos(angle);
            float fSin = (float) sin(angle);
            float fOneMinusCos = 1.0f-fCos;
            float fX2 = axis.x*axis.x;
            float fY2 = axis.y*axis.y;
            float fZ2 = axis.z*axis.z;
            float fXYM = axis.x*axis.y*fOneMinusCos;
            float fXZM = axis.x*axis.z*fOneMinusCos;
            float fYZM = axis.y*axis.z*fOneMinusCos;
            float fXSin = axis.x*fSin;
            float fYSin = axis.y*fSin;
            float fZSin = axis.z*fSin;
            
            m11 = fX2*fOneMinusCos+fCos;
            m12 = fXYM-fZSin;
            m13 = fXZM+fYSin;
            m14 = 0;
            
            m21 = fXYM+fZSin;
            m22 = fY2*fOneMinusCos+fCos;
            m23 = fYZM-fXSin;
            m24 = 0;
            
            m31 = fXZM-fYSin;
            m32 = fYZM+fXSin;
            m33 = fZ2*fOneMinusCos+fCos;
            m34 = 0;
            
            m41 = 0;
            m42 = 0;
            m43 = 0;
            m44 = 1;
        }
    }
    
    /// set to a look at Matrix4.
    
    void lookat(const Vector3 &eye, const Vector3 &at, const Vector3 &up)
    {
        // left handed
        
        Vector3 z_axis = at - eye;
        Vector3 x_axis = up.cross(z_axis);
        Vector3 y_axis = z_axis.cross(x_axis);
        
        x_axis.normalize();
        y_axis.normalize();
        z_axis.normalize();
        
        m11	= x_axis.x;
        m12 = x_axis.y;
        m13 = x_axis.z;
        m14 = - x_axis.dot(eye);
        
        m21	= y_axis.x;
        m22 = y_axis.y;
        m23 = y_axis.z;
        m24 = - y_axis.dot(eye);
        
        m31	= z_axis.x;
        m32 = z_axis.y;
        m33 = z_axis.z;
        m34 = - z_axis.dot(eye);
        
        m41	= 0;
        m42 = 0;
        m43 = 0;
        m44 = 1;
    }
    
    /// set to an orthographic projection Matrix4.
    
    void orthographic(float l, float r, float b, float t, float n, float f)
    {
        float sx = 1 / (r - l);
        float sy = 1 / (t - b);
        float sz = 1 / (f - n);
        m11 = 2 * sx;
        m21 = 0;
        m31 = 0;
        m41 = - (r+l) * sx;
        m12 = 0;
        m22 = 2 * sy;
        m32 = 0;
        m42 = - (t+b) * sy;
        m13 = 0;
        m23 = 0;
        m33 = -2 * sz;
        m43 = - (n+f) * sz;
        m14 = 0;
        m24 = 0;
        m34 = 0;
        m44 = 1;
    }
    
    /// set to a perspective projection Matrix4.
    
    void perspective(float l, float r, float t, float b, float n, float f)
    {
        m11	= 2*n / (r-l);
        m12 = 0;
        m13 = 0;
        m14 = 0;
        
        m21 = 0;
        m22 = 2*n / (t-b);
        m23 = 0;
        m24 = 0;
        
        m31 = 0;
        m32 = 0;
        m33 = f / (f-n);
        m34 = n*f / (n-f);
        
        m41 = 0;
        m42 = 0;
        m43 = 1;
        m44 = 0;
    }
    
    /// set to a perspective projection Matrix4 specified in terms of field of view and aspect ratio.
    
    void perspective(float fov, float aspect, float n, float f)
    {
        const float t = tan(fov*0.5f) * n;
        const float b = -t;
        
        const float l = aspect * b;
        const float r = aspect * t;
        
        perspective(l,r,t,b,n,f);
    }
    
    /// calculate determinant of 3x3 sub Matrix4.
    
    float determinant() const
    {
        return -m13*m22*m31 + m12*m23*m31 + m13*m21*m32 - m11*m23*m32 - m12*m21*m33 + m11*m22*m33;
    }
    
    /// determine if Matrix4 is invertible.
    /// note: currently only checks 3x3 sub Matrix4 determinant.
    
    bool invertible() const
    {
        return !equal(determinant(),0);
    }
    
    /// calculate inverse of Matrix4.
    
    Matrix4 inverse() const
    {
        Matrix4 m;
        inverse(m);
        return m;
    }
    
    /// calculate inverse of Matrix4 and write result to parameter Matrix4.
    
    void inverse(Matrix4 &inverse) const
    {
        const float determinant = this->determinant();
        
        assert(!equal(determinant,0));
        
        float k = 1.0f / determinant;
        
        inverse.m11 = (m22*m33 - m32*m23) * k;
        inverse.m12 = (m32*m13 - m12*m33) * k;
        inverse.m13 = (m12*m23 - m22*m13) * k;
        inverse.m21 = (m23*m31 - m33*m21) * k;
        inverse.m22 = (m33*m11 - m13*m31) * k;
        inverse.m23 = (m13*m21 - m23*m11) * k;
        inverse.m31 = (m21*m32 - m31*m22) * k;
        inverse.m32 = (m31*m12 - m11*m32) * k;
        inverse.m33 = (m11*m22 - m21*m12) * k;
        
        inverse.m14 = -(inverse.m11*m14 + inverse.m12*m24 + inverse.m13*m34);
        inverse.m24 = -(inverse.m21*m14 + inverse.m22*m24 + inverse.m23*m34);
        inverse.m34 = -(inverse.m31*m14 + inverse.m32*m24 + inverse.m33*m34);
        
        inverse.m41 = m41;
        inverse.m42 = m42;
        inverse.m43 = m43;
        inverse.m44 = m44;
    }
    
    /// calculate transpose of Matrix4.
    
    Matrix4 transpose() const
    {
        Matrix4 m;
        transpose(m);
        return m;
    }
    
    /// calculate transpose of Matrix4 and write to parameter Matrix4.
    
    void transpose(Matrix4 &transpose) const
    {
        transpose.m11 = m11;
        transpose.m12 = m21;
        transpose.m13 = m31;
        transpose.m14 = m41;
        transpose.m21 = m12;
        transpose.m22 = m22;
        transpose.m23 = m32;
        transpose.m24 = m42;
        transpose.m31 = m13;
        transpose.m32 = m23;
        transpose.m33 = m33;
        transpose.m34 = m43;
        transpose.m41 = m14;
        transpose.m42 = m24;
        transpose.m43 = m34;
        transpose.m44 = m44;
    }
    
    /// transform a Vector3 by this Matrix4.
    /// the convention used is post-multiplication by a column Vector3: x=Ab.
    
    Vector3 transform(const Vector3 &vec) const
    {
        float x = vec.x * m11 + vec.y * m12 + vec.z * m13 + m14;
        float y = vec.x * m21 + vec.y * m22 + vec.z * m23 + m24;
        float z = vec.x * m31 + vec.y * m32 + vec.z * m33 + m34;
        return Vector3(x, y, z);
    }
    
    /// transform a Vector3 by this Matrix4 using only the 3x3 rotation subMatrix4.
    /// the convention used is post-multiplication by a column Vector3: x=Ab.
    
    Vector3 transform3x3(const Vector3 &vec) const
    {
        float x = vec.x * m11 + vec.y * m12 + vec.z * m13;
        float y = vec.x * m21 + vec.y * m22 + vec.z * m23;
        float z = vec.x * m31 + vec.y * m32 + vec.z * m33;
        return Vector3(x, y, z);
    }
    
    /// add another Matrix4 to this Matrix4.
    
    void add(const Matrix4 &m)
    {
        m11 += m.m11;
        m12 += m.m12;
        m13 += m.m13;
        m14 += m.m14;
        m21 += m.m21;
        m22 += m.m22;
        m23 += m.m23;
        m24 += m.m24;
        m31 += m.m31;
        m32 += m.m32;
        m33 += m.m33;
        m34 += m.m34;
        m41 += m.m41;
        m42 += m.m42;
        m43 += m.m43;
        m44 += m.m44;
    }
    
    /// subtract a Matrix4 from this Matrix4.
    
    void subtract(const Matrix4 &m)
    {
        m11 -= m.m11;
        m12 -= m.m12;
        m13 -= m.m13;
        m14 -= m.m14;
        m21 -= m.m21;
        m22 -= m.m22;
        m23 -= m.m23;
        m24 -= m.m24;
        m31 -= m.m31;
        m32 -= m.m32;
        m33 -= m.m33;
        m34 -= m.m34;
        m41 -= m.m41;
        m42 -= m.m42;
        m43 -= m.m43;
        m44 -= m.m44;
    }
    
    /// multiply this Matrix4 by a scalar.
    
    void multiply(float scalar)
    {
        m11 *= scalar;
        m12 *= scalar;
        m13 *= scalar;
        m14 *= scalar;
        m21 *= scalar;
        m22 *= scalar;
        m23 *= scalar;
        m24 *= scalar;
        m31 *= scalar;
        m32 *= scalar;
        m33 *= scalar;
        m34 *= scalar;
        m41 *= scalar;
        m42 *= scalar;
        m43 *= scalar;
        m44 *= scalar;
    }
    
    /// equals operator
    
    bool operator ==(const Matrix4 &other) const
    {
        if (equal(m11,other.m11) &&
            equal(m12,other.m12) &&
            equal(m13,other.m13) &&
            equal(m14,other.m14) &&
            equal(m21,other.m21) &&
            equal(m22,other.m22) &&
            equal(m23,other.m23) &&
            equal(m24,other.m24) &&
            equal(m31,other.m31) &&
            equal(m32,other.m32) &&
            equal(m33,other.m33) &&
            equal(m34,other.m34) &&
            equal(m41,other.m41) &&
            equal(m42,other.m42) &&
            equal(m43,other.m43) &&
            equal(m44,other.m44)) return true;
        else return false;
    }
    
    /// not equals operator
    
    bool operator !=(const Matrix4 &other) const
    {
        return !(*this==other);
    }
    
    /// cute access to Matrix4 elements via overloaded () operator.
    /// use it like this: Matrix4 Matrix4; float element = Matrix4(row, column);
    
    float& operator()(int i, int j)
    {
        assert(i>=0);
        assert(i<=3);
        assert(j>=0);
        assert(j<=3);
        float *data = &m11;
        return data[(i<<2) + j];
    }
    
    /// const version of element access above.
    
    const float& operator()(int i, int j) const
    {
        assert(i>=0);
        assert(i<=3);
        assert(j>=0);
        assert(j<=3);
        const float *data = &m11;
        return data[(i<<2) + j];
    }
    
    /// data accessor for easy conversion to float* for OpenGL
    
    float* data()
    {
        return &m11;
    }
    
    friend inline Matrix4 operator-(const Matrix4 &a);
    friend inline Matrix4 operator+(const Matrix4 &a, const Matrix4 &b);
    friend inline Matrix4 operator-(const Matrix4 &a, const Matrix4 &b);
    friend inline Matrix4 operator*(const Matrix4 &a, const Matrix4 &b);
    friend inline Matrix4& operator+=(Matrix4 &a, const Matrix4 &b);
    friend inline Matrix4& operator-=(Matrix4 &a, const Matrix4 &b);
    friend inline Matrix4& operator*=(Matrix4 &a, const Matrix4 &b);
    
    friend inline Vector3 operator*(const Matrix4 &m, const Vector3 &v);
    friend inline Vector3 operator*(const Vector3 &v, const Matrix4 &m);
    friend inline Vector3& operator*=(Vector3 &v, const Matrix4 &m);
    
    friend inline Matrix4 operator*(const Matrix4 &a, float s);
    friend inline Matrix4 operator/(const Matrix4 &a, float s);
    friend inline Matrix4& operator*=(Matrix4 &a, float s);
    friend inline Matrix4& operator/=(Matrix4 &a, float s);
    friend inline Matrix4 operator*(float s, const Matrix4 &a);
    
    
    // 4x4 Matrix4, index m[row][column], convention: pre-multiply column Vector3, Ax = b
    // hence: (m11,m21,m31) make up the x axis of the 3x3 sub Matrix4,
    // and (m14,m24,m34) is the translation Vector3.
    
    float m11,m12,m13,m14;
    float m21,m22,m23,m24;
    float m31,m32,m33,m34;
    float m41,m42,m43,m44;
};


inline Matrix4 operator-(const Matrix4 &a)
{
    return Matrix4(-a.m11, -a.m12, -a.m13, -a.m14,
                  -a.m21, -a.m22, -a.m23, -a.m24,
                  -a.m31, -a.m32, -a.m33, -a.m34,
                  -a.m41, -a.m42, -a.m43, -a.m44);
}

inline Matrix4 operator+(const Matrix4 &a, const Matrix4 &b)
{
    return Matrix4(a.m11+b.m11, a.m12+b.m12, a.m13+b.m13, a.m14+b.m14,
                  a.m21+b.m21, a.m22+b.m22, a.m23+b.m23, a.m24+b.m24,
                  a.m31+b.m31, a.m32+b.m32, a.m33+b.m33, a.m34+b.m34,
                  a.m41+b.m41, a.m42+b.m42, a.m43+b.m43, a.m44+b.m44);
}

inline Matrix4 operator-(const Matrix4 &a, const Matrix4 &b)
{
    return Matrix4(a.m11-b.m11, a.m12-b.m12, a.m13-b.m13, a.m14-b.m14,
                  a.m21-b.m21, a.m22-b.m22, a.m23-b.m23, a.m24-b.m24,
                  a.m31-b.m31, a.m32-b.m32, a.m33-b.m33, a.m34-b.m34,
                  a.m41-b.m41, a.m42-b.m42, a.m43-b.m43, a.m44-b.m44);
}

inline Matrix4 operator*(const Matrix4 &a, const Matrix4 &b)
{
    return Matrix4(a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31 + a.m14*b.m41,
                  a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32 + a.m14*b.m42,
                  a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33 + a.m14*b.m43,
                  a.m11*b.m14 + a.m12*b.m24 + a.m13*b.m34 + a.m14*b.m44,
                  a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31 + a.m24*b.m41,
                  a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32 + a.m24*b.m42,
                  a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33 + a.m24*b.m43,
                  a.m21*b.m14 + a.m22*b.m24 + a.m23*b.m34 + a.m24*b.m44,
                  a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31 + a.m34*b.m41,
                  a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32 + a.m34*b.m42,
                  a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33 + a.m34*b.m43,
                  a.m31*b.m14 + a.m32*b.m24 + a.m33*b.m34 + a.m34*b.m44,
                  a.m41*b.m11 + a.m42*b.m21 + a.m43*b.m31 + a.m44*b.m41,
                  a.m41*b.m12 + a.m42*b.m22 + a.m43*b.m32 + a.m44*b.m42,
                  a.m41*b.m13 + a.m42*b.m23 + a.m43*b.m33 + a.m44*b.m43,
                  a.m41*b.m14 + a.m42*b.m24 + a.m43*b.m34 + a.m44*b.m44);
}

inline Matrix4& operator+=(Matrix4 &a, const Matrix4 &b)
{
    a.add(b);
    return a;
}

inline Matrix4& operator-=(Matrix4 &a, const Matrix4 &b)
{
    a.subtract(b);
    return a;
}

inline Matrix4& operator*=(Matrix4 &a, const Matrix4 &b)
{
    a = Matrix4(a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31 + a.m14*b.m41,
               a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32 + a.m14*b.m42,
               a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33 + a.m14*b.m43,
               a.m11*b.m14 + a.m12*b.m24 + a.m13*b.m34 + a.m14*b.m44,
               a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31 + a.m24*b.m41,
               a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32 + a.m24*b.m42,
               a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33 + a.m24*b.m43,
               a.m21*b.m14 + a.m22*b.m24 + a.m23*b.m34 + a.m24*b.m44,
               a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31 + a.m34*b.m41,
               a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32 + a.m34*b.m42,
               a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33 + a.m34*b.m43,
               a.m31*b.m14 + a.m32*b.m24 + a.m33*b.m34 + a.m34*b.m44,
               a.m41*b.m11 + a.m42*b.m21 + a.m43*b.m31 + a.m44*b.m41,
               a.m41*b.m12 + a.m42*b.m22 + a.m43*b.m32 + a.m44*b.m42,
               a.m41*b.m13 + a.m42*b.m23 + a.m43*b.m33 + a.m44*b.m43,
               a.m41*b.m14 + a.m42*b.m24 + a.m43*b.m34 + a.m44*b.m44);
    return a;											 
}

inline Vector3 operator*(const Matrix4& m, const Vector3& vec)
{
    return Vector3(vec.x * m.m11 + vec.y * m.m12 + vec.z * m.m13 + m.m14,
                  vec.x * m.m21 + vec.y * m.m22 + vec.z * m.m23 + m.m24,
                  vec.x * m.m31 + vec.y * m.m32 + vec.z * m.m33 + m.m34);
}

inline Vector3 operator*(const Vector3& vec, const Matrix4& m)
{
    // when we premultiply x*A we assume the Vector3 is a row Vector3
    
    return Vector3(vec.x * m.m11 + vec.y * m.m21 + vec.z * m.m31 + m.m41,
                  vec.x * m.m12 + vec.y * m.m22 + vec.z * m.m32 + m.m42,
                  vec.x * m.m13 + vec.y * m.m23 + vec.z * m.m33 + m.m43);
}

inline Matrix4 operator*(const Matrix4 &a, float s)
{
    return Matrix4(s*a.m11, s*a.m12, s*a.m13, s*a.m14,
                  s*a.m21, s*a.m22, s*a.m23, s*a.m24,
                  s*a.m31, s*a.m32, s*a.m33, s*a.m34,
                  s*a.m41, s*a.m42, s*a.m43, s*a.m44);
}

inline Matrix4 operator/(const Matrix4 &a, float s)
{
    assert(s!=0);
    const float inv = 1.0f / s;
    return Matrix4(inv*a.m11, inv*a.m12, inv*a.m13, inv*a.m14,
                  inv*a.m21, inv*a.m22, inv*a.m23, inv*a.m24,
                  inv*a.m31, inv*a.m32, inv*a.m33, inv*a.m34,
                  inv*a.m41, inv*a.m42, inv*a.m43, inv*a.m44);
}

inline Matrix4& operator*=(Matrix4 &a, float s)
{
    a.multiply(s);
    return a;
}

inline Matrix4& operator/=(Matrix4 &a, float s)
{
    assert(s!=0);
    a.multiply(1.0f/s);
    return a;
}

inline Matrix4 operator*(float s, const Matrix4 &a)
{
    return Matrix4(s*a.m11, s*a.m12, s*a.m13, s*a.m14,
                  s*a.m21, s*a.m22, s*a.m23, s*a.m24,
                  s*a.m31, s*a.m32, s*a.m33, s*a.m34,
                  s*a.m41, s*a.m42, s*a.m43, s*a.m44);
}

inline Matrix4& operator*=(float s, Matrix4 &a)
{
    a.multiply(s);
    return a;
}
