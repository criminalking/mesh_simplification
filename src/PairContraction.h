#include <iostream>
#include <stdio.h>
#include "SimpleObject.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <list>

using namespace Eigen;

struct Vertex // for write
{
  Vector4f v; // value of this vertex
  Matrix4f Q; // Q
  std::list<int> friend_index; // another vertex of a pair including this vertex
  bool is_active; // true: simplified model should include this vertex
};

struct Pairs
{
  int v1_index, v2_index; // TODO: could only save index
  float cost; // TODO: add Vertex v???
  int triangle_index[2]; // index of triangle including these two vertexes, size should be 2
};

struct Plane // for write
{
  int vertex_index[3]; // index of vertexes of this triangle
  Matrix4f Kp;
  bool is_active; //true: simplified model should include this plane
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
        plane[i].vertex_index[0] = v1_index;
        plane[i].vertex_index[1] = v2_index;
        plane[i].vertex_index[2] = v3_index;
        plane[i].Kp = ComputeP(m_pVertexList[v1_index], m_pVertexList[v2_index], m_pVertexList[v3_index]);
        vertex[v1_index].Q += plane[i].Kp;
        vertex[v2_index].Q += plane[i].Kp;
        vertex[v3_index].Q += plane[i].Kp;
      }
  }
  void SelectPairs();
  float ComputeCost(Matrix4f Q1, Matrix4f Q2); // should verify invertibility
  void AddToHeap(Pairs pair); // add pairs to minimum heap
  Matrix4f ComputeP(SimpleOBJ::Vec3f x, SimpleOBJ::Vec3f y, SimpleOBJ::Vec3f z); // compute p for every plane


  // for test
  void PrintMatrix(Matrix4f M) {
    for (int i = 0; i < 16; ++i)
      std::cout << M(i) << "   ";
  }


 private:
  int m_nVertices;
  int m_nTriangles;
  Plane *plane;
  Vertex *vertex;
  Pairs *pairs;
};
