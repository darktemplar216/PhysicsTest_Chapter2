
class Quaternion
{
public:
    
    /// default constructor.
    /// does nothing for speed.
    
    Quaternion() {}
    
    /// construct quaternion from real component w and imaginary x,y,z.
    
    Quaternion(float w, float x, float y, float z)
    {
        this->w = w;
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    /// construct quaternion from angle-axis
    
    Quaternion(float angle, const Vector3 &axis)
    {
        const float a = angle * 0.5f;
        const float s = (float) sin(a);
        const float c = (float) cos(a);
        w = c;
        x = axis.x * s;
        y = axis.y * s;
        z = axis.z * s;
    }
    
    /// construct quaternion from rotation matrix.
    
    Quaternion(const Matrix4 &matrix)
    {
        // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
        // article "Quaternion Calculus and Fast Animation".
        
        const float trace = matrix.m11 + matrix.m22 + matrix.m33;
        
        if (trace>0)
        {
            // |w| > 1/2, may as well choose w > 1/2
            
            float root = sqrt(trace + 1.0f);  // 2w
            w = 0.5f * root;
            root = 0.5f / root;  // 1/(4w)
            x = (matrix.m32-matrix.m23) * root;
            y = (matrix.m13-matrix.m31) * root;
            z = (matrix.m21-matrix.m12) * root;
        }
        else
        {
            // |w| <= 1/2
            
            static int next[3] = { 2, 3, 1 };
            
            int i = 1;
            if (matrix.m22>matrix.m11)  i = 2;
            if (matrix.m33>matrix(i,i)) i = 3;
            int j = next[i];
            int k = next[j];
            
            float root = sqrt(matrix(i,i)-matrix(j,j)-matrix(k,k) + 1.0f);
            float *quaternion[3] = { &x, &y, &z };
            *quaternion[i] = 0.5f * root;
            root = 0.5f / root;
            w = (matrix(k,j)-matrix(j,k))*root;
            *quaternion[j] = (matrix(j,i)+matrix(i,j))*root;
            *quaternion[k] = (matrix(k,i)+matrix(i,k))*root;
        }
    }
    
    /// convert quaternion to matrix.
    
    Matrix4 matrix() const
    {
        // from david eberly's sources used with permission.
        
        float fTx  = 2.0f*x;
        float fTy  = 2.0f*y;
        float fTz  = 2.0f*z;
        float fTwx = fTx*w;
        float fTwy = fTy*w;
        float fTwz = fTz*w;
        float fTxx = fTx*x;
        float fTxy = fTy*x;
        float fTxz = fTz*x;
        float fTyy = fTy*y;
        float fTyz = fTz*y;
        float fTzz = fTz*z;
        
        return Matrix4(1.0f-(fTyy+fTzz), fTxy-fTwz, fTxz+fTwy,
                      fTxy+fTwz, 1.0f-(fTxx+fTzz), fTyz-fTwx,
                      fTxz-fTwy, fTyz+fTwx, 1.0f-(fTxx+fTyy));
    }
    
    /// convert quaternion to angle-axis.
    
    void angleAxis(float &angle, Vector3 &axis) const
    {
        const float squareLength = x*x + y*y + z*z;
        
        if (squareLength > EPSILON_PW_FLT)
        {
            angle = 2.0f * (float) acos(w);
            const float inverseLength = 1.0f / (float) pow(squareLength, 0.5f);
            axis.x = x * inverseLength;
            axis.y = y * inverseLength;
            axis.z = z * inverseLength;
        }
        else
        {
            angle = 0.0f;
            axis.x = 1.0f;
            axis.y = 0.0f;
            axis.z = 0.0f;
        }
    }
    
    /// set quaternion to zero.
    
    void zero()
    {
        w = 0;
        x = 0;
        y = 0;
        z = 0;
    }
    
    /// set quaternion to identity.
    
    void identity()
    {
        w = 1;
        x = 0;
        y = 0;
        z = 0;
    }
    
    /// add another quaternion to this quaternion.
    
    void add(const Quaternion &q)
    {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
    }
    
    /// subtract another quaternion from this quaternion.
    
    void subtract(const Quaternion &q)
    {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
    }
    
    /// multiply this quaternion by a scalar.
    
    void multiply(float s)
    {
        w *= s;
        x *= s;
        y *= s;
        z *= s;
    }
    
    /// divide this quaternion by a scalar.
    
    void divide(float s)
    {
        assert(s!=0);
        const float inv = 1.0f / s;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
    }
    
    /// multiply this quaternion with another quaternion.
    
    void multiply(const Quaternion &q)
    {
        const float rw = w*q.w - x*q.x - y*q.y - z*q.z;
        const float rx = w*q.x + x*q.w + y*q.z - z*q.y;
        const float ry = w*q.y - x*q.z + y*q.w + z*q.x;
        const float rz = w*q.z + x*q.y - y*q.x + z*q.w;
        w = rw;
        x = rx;
        y = ry;
        z = rz;
    }
    
    /// multiply this quaternion with another quaternion and store result in parameter.
    
    void multiply(const Quaternion &q, Quaternion &result) const
    {
        result.w = w*q.w - x*q.x - y*q.y - z*q.z;
        result.x = w*q.x + x*q.w + y*q.z - z*q.y;
        result.y = w*q.y - x*q.z + y*q.w + z*q.x;
        result.z = w*q.z + x*q.y - y*q.x + z*q.w;
    }
    
    /// dot product of two quaternions.
    
    Quaternion dot(const Quaternion &q)
    {
        return Quaternion(w*q.w + x*q.x + y*q.y + z*q.z, 0, 0, 0);
    }
    
    /// dot product of two quaternions writing result to parameter.
    
    void dot(const Quaternion &q, Quaternion &result)
    {
        result = Quaternion(w*q.w + x*q.x + y*q.y + z*q.z, 0, 0, 0);
    }
    
    /// calculate conjugate of quaternion.
    
    Quaternion conjugate()
    {
        return Quaternion(w, -x, -y, -z);
    }
    
    /// calculate conjugate of quaternion and store result in parameter.
    
    void conjugate(Quaternion &result) const
    {
        result = Quaternion(w, -x, -y, -z);
    }
    
    /// calculate length of quaternion
    
    float length() const
    {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    
    /// calculate norm of quaternion.
    
    float norm() const
    {
        return w*w + x*x + y*y + z*z;
    }
    
    /// normalize quaternion.
    
    void normalize()
    {
        const float length = this->length();
        
        if (length == 0)
        {
            w = 1;
            x = 0;
            y = 0;
            z = 0;
        }
        else
        {
            float inv = 1.0f / length;
            x = x * inv;
            y = y * inv;
            z = z * inv;
            w = w * inv;
        }
    }
    
    /// check if quaternion is normalized
    
    bool normalized() const
    {
        return equal(norm(),1);
    }
    
    /// calculate inverse of quaternion
    
    Quaternion inverse() const
    {
        const float n = norm();
        assert(n!=0);
        return Quaternion(w/n, -x/n, -y/n, -z/n);
    }
    
    /// calculate inverse of quaternion and store result in parameter.
    
    void inverse(Quaternion &result) const
    {
        const float n = norm();
        result = Quaternion(w/n, -x/n, -y/n, -z/n);
    }
    
    void operator =(const GLKQuaternion& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
    }
    
    GLKQuaternion toGLKQuaternion()
    {
        return GLKQuaternionMake(x, y, z, w);
    }
    
    Vector3 rotateV(const Vector3& v) const
    {
        // orgre, nVidia SDK implementation
        Vector3 uv, uuv;
        Vector3 qvec(x, y, z);
        uv = qvec * v;
        uuv = qvec * uv;
        uv *= (2.0f * w);
        uuv *= 2.0f;
        return v + uv + uuv;
        
        // 			Quaternion t;
        // 			t = *this * v * (~*this);
        // 			return t.getVector3f();
    }
    
    /// equals operator
    
    bool operator ==(const Quaternion &other) const
    {
        if ( equal(w,other.w) && equal(x,other.x) && equal(y,other.y) && equal(z,other.z))
            return true;
        else
            return false;
    }
    
    /// not equals operator
    
    bool operator !=(const Quaternion &other) const
    {
        return !(*this==other);
    }
    
    /// element access
    
    float& operator [](int i)
    {
        assert(i>=0);
        assert(i<=2);
        return *(&w+i);
    }
    
    /// element access (const)
    
    const float& operator [](int i) const
    {
        assert(i>=0);
        assert(i<=2);
        return *(&w+i);
    }
    
    static Quaternion slerp(const Quaternion &a, const Quaternion &b, float t)
    {
        assert(t>=0);
        assert(t<=1);
        
        float flip = 1;
        
        float cosine = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        
        if (cosine<0)
        {
            cosine = -cosine;
            flip = -1;
        }
        
        if ((1-cosine) < EPSILON_FLT)
            return a * (1-t) + b * (t*flip);
        
        float theta = (float)acos(cosine);
        float sine = (float)sin(theta);
        float beta = (float)sin((1-t)*theta) / sine;
        float alpha = (float)sin(t*theta) / sine * flip;
        
        return a * beta + b * alpha; 
    }
    
    friend inline Quaternion operator-(const Quaternion &a);
    friend inline Quaternion operator+(const Quaternion &a, const Quaternion &b);
    friend inline Quaternion operator-(const Quaternion &a, const Quaternion &b);
    friend inline Quaternion operator*(const Quaternion &a, const Quaternion &b);
    friend inline Quaternion& operator+=(Quaternion &a, const Quaternion &b);
    friend inline Quaternion& operator-=(Quaternion &a, const Quaternion &b);
    friend inline Quaternion& operator*=(Quaternion &a, const Quaternion &b);
    
    friend inline bool operator ==(const Quaternion &q, float scalar);
    friend inline bool operator !=(const Quaternion &q, float scalar);
    friend inline bool operator ==(float scalar, const Quaternion &q);
    friend inline bool operator !=(float scalar, const Quaternion &q);
    
    friend inline Quaternion operator*(const Quaternion &a, float s);
    friend inline Quaternion operator/(const Quaternion &a, float s);
    friend inline Quaternion& operator*=(Quaternion &a, float s);
    friend inline Quaternion& operator/=(Quaternion &a, float s);
    friend inline Quaternion operator*(float s, const Quaternion &a);
    friend inline Quaternion& operator*=(float s, Quaternion &a);
    
    float w;        ///< w component of quaternion
    float x;        ///< x component of quaternion
    float y;        ///< y component of quaternion
    float z;        ///< z component of quaternion
};


inline Quaternion operator-(const Quaternion &a)
{
    return Quaternion(-a.w, -a.x, -a.y, -a.z);
}

inline Quaternion operator+(const Quaternion &a, const Quaternion &b)
{
    return Quaternion(a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z);
}

inline Quaternion operator-(const Quaternion &a, const Quaternion &b)
{
    return Quaternion(a.w-b.w, a.x-b.x, a.y-b.y, a.z-b.z);
}

inline Quaternion operator*(const Quaternion &a, const Quaternion &b)
{
    return Quaternion( a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
                      a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                      a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
                      a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w );
}

inline Quaternion& operator+=(Quaternion &a, const Quaternion &b)
{
    a.w += b.w;
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Quaternion& operator-=(Quaternion &a, const Quaternion &b)
{
    a.w -= b.w;
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline Quaternion& operator*=(Quaternion &a, const Quaternion &b)
{
    a.multiply(b);
    return a;
}


inline bool operator ==(const Quaternion &q, float scalar)
{
    if ( equal(q.w,scalar) && equal(q.x,0) && equal(q.y,0) && equal(q.z,0))
        return true;
    else
        return false;
}

inline bool operator !=(const Quaternion &q, float scalar)
{
    return !(q==scalar);
}

inline bool operator ==(float scalar, const Quaternion &q)
{
    if ( equal(q.w,scalar) && equal(q.x,0) && equal(q.y,0) && equal(q.z,0)) 
        return true;
    else 
        return false;
}

inline bool operator !=(float scalar, const Quaternion &q)
{
    return !(q==scalar);
}

inline Quaternion operator*(const Quaternion &a, float s)
{
    return Quaternion(a.w*s, a.x*s, a.y*s, a.z*s);
}

inline Quaternion operator/(const Quaternion &a, float s)
{
    return Quaternion(a.w/s, a.x/s, a.y/s, a.z/s);
}

inline Quaternion& operator*=(Quaternion &a, float s)
{
    a.multiply(s);
    return a;
}

inline Quaternion& operator/=(Quaternion &a, float s)
{
    a.divide(s);
    return a;
}

inline Quaternion operator*(float s, const Quaternion &a)
{
    return Quaternion(a.w*s, a.x*s, a.y*s, a.z*s);
}

inline Quaternion& operator*=(float s, Quaternion &a)
{
    a.multiply(s);
    return a;
}
