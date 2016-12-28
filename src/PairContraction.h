#include <iostream>
#include <stdio.h>
#include "SimpleObject.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;

struct Vertex
{
  Vector4f v; // value of this vertex
  Matrix4f Q; // every vertex has a Q
};

struct Pairs
{
  Vertex v1, v2;
  Vertex v;
  float cost;
};

struct Plane
{

  Matrix4f Kp;
};

class CPairContraction
{
 public:
 CPairContraction(int m_nVertices, int m_nTriangles, SimpleOBJ::Vec3f* m_pVertexList, SimpleOBJ::Array<int,3>* m_pTriangleList): m_nVertices(m_nVertices), m_nTriangles(m_nTriangles) {
    vertex = new Vertex[m_nVertices];
    for (int i = 0; i < m_nVertices; ++i)
      {
        vertex[i].v << m_pVertexList[i][0], m_pVertexList[i][1], m_pVertexList[i][2], 1.0;
        vertex[i].Q = Matrix4f::Zero(); // initialize Q
      }
    plane = new Plane[m_nTriangles];
    for (int i = 0; i < m_nTriangles; ++i)
      {
        int v1_index = m_pTriangleList[i][0];
        int v2_index = m_pTriangleList[i][1];
        int v3_index = m_pTriangleList[i][2];
        plane[i].Kp = ComputeP(m_pVertexList[v1_index], m_pVertexList[v2_index], m_pVertexList[v3_index]);
        vertex[v1_index].Q = vertex[v1_index].Q + plane[i].Kp;
        vertex[v2_index].Q = vertex[v2_index].Q + plane[i].Kp;
        vertex[v3_index].Q = vertex[v3_index].Q + plane[i].Kp;
      }
  }

  Matrix4f ComputeP(SimpleOBJ::Vec3f x, SimpleOBJ::Vec3f y, SimpleOBJ::Vec3f z); // compute p for every plane


 private:
  int m_nVertices;
  int m_nTriangles;
  Plane *plane;
  Vertex *vertex;
  Pairs *pairs;
};
