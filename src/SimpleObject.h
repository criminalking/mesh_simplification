#pragma once
#include "Vec3f.h"
#include <vector>
#include <iostream>
#include <stdio.h>

namespace SimpleOBJ
{
    template <typename T, int N> class Array
    {
    public:
        enum {_len = N};
        typedef T t_Val;
    public:
        T& operator[] (int i)
        {
            if (i>=0&&i<N)
              return _p[i];
            else std::cout<<"Out of the index!!!\n";
        }
        const T& operator[] (int i) const
        {
            if (i>=0&&i<N)
              return _p[i];
            else std::cout<<"Out of the index!!!\n";
        }

    protected:
        T _p[N];
    };

    class CSimpleObject
    {
    public:
        CSimpleObject(void);
        ~CSimpleObject(void);

    public:
        bool IsLoaded() { return m_pVertexList!=NULL;}

        void Destroy();
        bool LoadFromObj(const char* fn);
        bool SaveToObj(const char* fn);

    protected:
        bool Parse(FILE* fp);
        bool CheckParse(int nVertices,std::vector<Array<int,3> > & vecTriangles);

    public: // TODO: need modification(maybe friend?? I just want "process" access to these parameters)
        int             m_nVertices;
        int             m_nTriangles;
        Vec3f*          m_pVertexList;
        Array<int,3>*   m_pTriangleList;
    };

}
