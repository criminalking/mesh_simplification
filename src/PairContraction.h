#include <iostream>
#include <stdio.h>
#include <math.h>
#include "SimpleObject.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <queue>          // std::priority_queue
#include <ctime>

using namespace Eigen;

struct Vertex // for write
{
  Vector4f v; // value of this vertex
  Matrix4f Q; // Q
  std::vector<int> friend_index; // another vertex of a pair including this vertex(just for building heap)
  std::vector<int> pairs_index; // index of pairs including vertex and friend vertex(just for building heap)
  bool is_active; // true: simplified model should include this vertex
};

struct Pairs
{
  int v1_index, v2_index; // TODO: modify to []
  float cost; // TODO: add Vertex v???
  int triangle_index[2]; // index of triangle including these two vertexes, general:size should be 2, also could be 1
};

struct Plane // for write
{
  int vertex_index[3]; // index of vertexes of this triangle
  bool is_active; //true: simplified model should include this plane
};


class mycomparison
{
public:
  mycomparison() {}
  bool operator() (const Pairs& lhs, const Pairs&rhs) const
  {
    return (lhs.cost>rhs.cost);
  }
};

typedef std::priority_queue<float,std::vector<Pairs>,mycomparison> Heap;

class CPairContraction
{
 public:
  CPairContraction(int m_nVertices, int m_nTriangles, SimpleOBJ::Vec3f* m_pVertexList, SimpleOBJ::Array<int,3>* m_pTriangleList, float ratio);
  void SelectPairs();
  float ComputeCost(Matrix4f Q1, Matrix4f Q2); // should verify invertibility
  Vector4f ComputeV(Matrix4f Q); // compute new v
  void CreatePairs(int v1, int v2, int index); // add(v1, v2), index is index of this triangle
  void AddPairs(int v1_index, int v2_index, int triangle_index); //
  void BuildHeap(); // build pairs heap
  void Iteration();
  inline bool IsThisPair(int v1, int v2, Pairs pair);
  inline bool PointInPlane(int v_index, int p_index); // detect one vertex is in plane or not
  inline bool IsThisPlane(int v1, int v2, int v3, int plane);
  Matrix4f ComputeP(SimpleOBJ::Vec3f x, SimpleOBJ::Vec3f y, SimpleOBJ::Vec3f z); // compute p for every plane
  void RefreshIndex(int &object_nVertices, int &object_nTriangles, SimpleOBJ::Vec3f* m_pVertexList, SimpleOBJ::Array<int,3>* m_pTriangleList); // refresh index of vertexes and planes(for write model)
  inline void Error(std::string error);

  void Run();

  // for test
  void PrintMatrix(Matrix4f M) {
    for (int i = 0; i < 16; ++i)
      std::cout << M(i) << "   ";
  }

  void PrintPlane(int plane) {
    for (int i = 0; i < 3; i++)
      {
        std::cout << planes[plane].vertex_index[i] << "  ";
      }
    std::cout << "Plane" << std::endl;
  }

  int FindOnePair(Pairs pair)
  {
    int k = 0;
    Heap temp;
    while(!heap.empty())
      {
        Pairs p = heap.top();
        heap.pop();
        temp.push(p);
        if (IsThisPair(p.v1_index, p.v2_index, pair)) k++;
      }
    heap.swap(temp);
    return k;
  }

 private:
  int m_nVertices;
  int m_nTriangles;
  std::vector<Plane> planes; // all planes
  std::vector<Vertex> vertexes; // all vertexes
  std::vector<Pairs> pairs; // all pairs
  Heap heap; // minimum cost is always on the top
  int m_nTriangles_new; // size of triangles after simplified
};
