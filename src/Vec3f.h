#pragma once
#include <stdio.h>
#include <iostream>

namespace SimpleOBJ
{
  class Vec3f
  {
  public:

    //Constructors
    Vec3f();
    Vec3f(float x,float y, float z);
    Vec3f(const Vec3f& v);
    //Deconstructor
    virtual ~Vec3f();
  public:
    //Operators

    //Operator []
    float& operator [](int index)
    {
      if (index >= 0 && index < 3)
        return _p[index];
      else
        std::cout << "Out of the index!!!\n";
    }

    const float& operator [](int index) const
    {
      if (index >= 0 && index < 3)
        return _p[index];
      else
        std::cout << "Out of the index!!!\n";
    }

    //Operator =
    Vec3f& operator = (const Vec3f& v);

    //Operators +=,-=, *=, /=
    void operator +=(const Vec3f& v);
    void operator +=(float f);
    void operator -=(const Vec3f& v);
    void operator -=(float f);
    void operator *=(const Vec3f& v);
    void operator *=(float f);
    void operator /=(const Vec3f& v);
    void operator /=(float f);

    //Operators +,-.*,/
    Vec3f operator +(const Vec3f&v) const;
    Vec3f operator +(float f) const;
    Vec3f operator -(const Vec3f&v) const;
    Vec3f operator -(float f) const;
    Vec3f operator *(const Vec3f&v) const;
    Vec3f operator *(float f) const;
    Vec3f operator /(const Vec3f&v) const;
    Vec3f operator /(float f) const;

    Vec3f operator -() const;

  public:
    void Normalize();
    float L2Norm_Sqr();

  public:
    union
    {
      struct
      { float _p[3]; };
      struct
      { float x,y,z; };
      struct
      { float r,g,b; };
    };
    enum {_len = 3};

  };
}


