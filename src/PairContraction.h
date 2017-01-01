#include <iostream>
#include <stdio.h>
#include <math.h>
#include "SimpleObject.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <queue>          // std::priority_queue

using namespace Eigen;

struct Vertex // for write
{
  Vector4f v; // value of this vertex
  Matrix4f Q; // Q
  std::vector<int> friend_index; // another vertex of a pair including this vertex
  std::vector<int> pairs_index; // index of pairs including vertex and friend vertex
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
  bool is_active; //true: simplified model should include this plane
};

class CPairContraction
{
 public:
 CPairContraction(int m_nVertices, int m_nTriangles, SimpleOBJ::Vec3f* m_pVertexList, SimpleOBJ::Array<int,3>* m_pTriangleList): m_nVertices(m_nVertices), m_nTriangles(m_nTriangles) {
    for (int i = 0; i < m_nVertices; ++i)
      {
        Vertex vertex;
        vertex.v << m_pVertexList[i][0], m_pVertexList[i][1], m_pVertexList[i][2], 1.0;
        vertex.Q = Matrix4f::Zero(); // initialize Q
        vertex.is_active = true;
        vertexes.push_back(vertex);
      }
    for (int i = 0; i < m_nTriangles; ++i)
      {
        Plane plane;
        int v1_index = m_pTriangleList[i][0];
        int v2_index = m_pTriangleList[i][1];
        int v3_index = m_pTriangleList[i][2];
        plane.vertex_index[0] = v1_index;
        plane.vertex_index[1] = v2_index;
        plane.vertex_index[2] = v3_index;
        plane.is_active = true;
        planes.push_back(plane);
        Matrix4f Kp = ComputeP(m_pVertexList[v1_index], m_pVertexList[v2_index], m_pVertexList[v3_index]);
        vertexes[v1_index].Q += Kp;
        vertexes[v2_index].Q += Kp;
        vertexes[v3_index].Q += Kp;
      }
  }
  void SelectPairs();
  float ComputeCost(Matrix4f Q1, Matrix4f Q2); // should verify invertibility
  void CreatePairs(int v1, int v2, int index); // add(v1, v2), index is index of this triangle
  void AddPairs(int v1_index, int v2_index, int triangle_index); //
  void AddToHeap(Pairs pair); // add pairs to minimum heap
  void BuildHeap(); // build pairs heap
  void Iteration(float ratio); // ratio of area
  Matrix4f ComputeP(SimpleOBJ::Vec3f x, SimpleOBJ::Vec3f y, SimpleOBJ::Vec3f z); // compute p for every plane
  void RefreshIndex(); // refresh index of vertexes and planes(for write model)


  // for test
  void PrintMatrix(Matrix4f M) {
    for (int i = 0; i < 16; ++i)
      std::cout << M(i) << "   ";
  }


 private:
  int m_nVertices;
  int m_nTriangles;
  std::vector<Plane> planes;
  std::vector<Vertex> vertexes;
  std::vector<Pairs> pairs;
};
