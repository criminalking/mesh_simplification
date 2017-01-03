#include "PairContraction.h"

Matrix4f CPairContraction::ComputeP(SimpleOBJ::Vec3f x, SimpleOBJ::Vec3f y, SimpleOBJ::Vec3f z)
{
  float a = (y[1] - x[1]) * (z[2] - x[2]) - (z[1] - x[1]) * (y[2] - x[2]);
  float b = (y[2] - x[2]) * (z[0] - x[0]) - (z[2] - x[2]) * (y[0] - x[0]);
  float c = (y[0] - x[0]) * (z[1] - x[1]) - (z[0] - x[0]) * (y[1] - x[1]);
  Vector3f v(a, b, c);
  v = v.normalized(); // let a^2+b^2+c^2=1
  a = v(0); b = v(1); c = v(2);
  float d = -a * x[0] - b * x[1] - c * x[2];
  Matrix4f Kp;
  Kp << a*a, a*b, a*c, a*d, a*b, b*b, b*c, b*d, a*c, b*c, c*c, c*d, a*d, b*d, c*d, d*d;
  return Kp;
}

Vector4f CPairContraction::ComputeV(Matrix4f Q)
{
  // compute v
  Matrix4f M;
  M << Q(1,1), Q(1,2), Q(1,3), Q(1,4), Q(1,2), Q(2,2), Q(2,3), Q(2,4), Q(1,3), Q(2,3), Q(3,3), Q(3,4), 0, 0, 0, 1;
  Vector4f vec(0,0,0,1);
  Vector4f v = M.inverse() * vec; // TODO: if invertible
  return v;
}

float CPairContraction::ComputeCost(Matrix4f Q1, Matrix4f Q2)
{
  Matrix4f Q = Q1 + Q2;
  Vector4f v = ComputeV(Q);
  // compute cost
  float cost = v.transpose() * Q * v;
  return cost;
}

void CPairContraction::BuildHeap()
{
  // use priority queues to build a pairs heap
  for (size_t i = 0; i < pairs.size(); ++i)
    {
      heap.push(pairs[i]);
    }
}

void CPairContraction::Iteration()
{
  int iter_num = ceil((1 - ratio) * m_nTriangles / 2); // TODO: need modify, some edge only have one plane
  for (int i = 0; i < iter_num; ++i)
    {
      if (heap.empty())
        {
          std::cout << "Heap is empty now! Maybe some errors!\n";
          exit(1);
        }
      else
        {
          Pairs pair = heap.top();
          heap.pop(); // delete the top element
          int v1_index = pair.v1_index;
          int v2_index = pair.v2_index;
          Matrix4f v_Q = vertexes[v1_index].Q + vertexes[v2_index].Q; // v of (v1, v2)
          // refresh heap
          // find another vertex of planes including v1 and v2
          int v3_index = -1, v4_index = -1;
          for (int k = 0; k < 3; ++k)
            {
              if (planes[pair.triangle_index[0]].vertex_index[k] != v1_index && planes[pair.triangle_index[0]].vertex_index[k] != v2_index) v3_index = planes[pair.triangle_index[0]].vertex_index[k];
              if (planes[pair.triangle_index[1]].vertex_index[k] != v1_index && planes[pair.triangle_index[1]].vertex_index[k] != v2_index) v4_index = planes[pair.triangle_index[1]].vertex_index[k];
            }
          if (v3_index < 0) Error("No plane of this edge!!!\n");

          Heap new_heap;
          while (!heap.empty()) // traverse heap
            {
              Pairs pair = heap.top(); // get one pair from this heap
              heap.pop();
              // if v1 in this pair
              int id = IsInPairs(v1_index, pair);
              if (id >= 0)
                {
                  // TODO: use operator
                  Pairs new_pair;
                  new_pair.v1_index = pair.v1_index;
                  new_pair.v2_index = pair.v2_index;
                  new_pair.triangle_index[0] = pair.triangle_index[0];
                  new_pair.triangle_index[1] = pair.triangle_index[1];
                  if (id == 0) new_pair.cost = ComputeCost(v_Q, vertexes[pair.v2_index].Q); // use v instead of v1
                  else new_pair.cost = ComputeCost(v_Q, vertexes[pair.v1_index].Q); // use v instead of v1
                  new_heap.push(new_pair);
                }

              // if v2 in this pair
              id = IsInPairs(v2_index, pair);
              if (id >= 0)
                {
                  int another_index = -1;
                  if (id == 0)
                    another_index = pair.v2_index;
                  else
                    another_index = pair.v1_index;
                  if (another_index < 0) Error("Pairs no two vertexes...\n");
                  if (another_index != v3_index && another_index != v4_index)
                    {
                      Pairs new_pair;
                      if (pair.v1_index == v2_index) new_pair.v1_index = v1_index;
                      else new_pair.v1_index = pair.v1_index;
                      if (pair.v2_index == v2_index) new_pair.v2_index = v1_index;
                      else new_pair.v2_index = pair.v2_index;
                      new_pair.triangle_index[0] = pair.triangle_index[0];
                      new_pair.triangle_index[1] = pair.triangle_index[1];
                      if (id == 0) new_pair.cost = ComputeCost(v_Q, vertexes[pair.v2_index].Q); // use v instead of v2
                      else new_pair.cost = ComputeCost(v_Q, vertexes[pair.v1_index].Q); // use v instead of v2
                      new_heap.push(new_pair);
                    }
                }
            }
          // swap heap and new heap
          heap.swap(new_heap);

          // !!!!!!!!!!!!TODO: Modify, some edges only have one plane!!!

          //refresh vertex, modify friend_index/pairs_index
          vertexes[v2_index].is_active = false;
          vertexes[v1_index].Q += vertexes[v2_index].Q;
          vertexes[v1_index].v = ComputeV(vertexes[v1_index].Q);
          for (int k = 0; k < vertexes[v2_index].friend_index.size(); ++k)
            {
              int index = vertexes[v2_index].friend_index[k];
              int m = 0;
              while (m != vertexes[index].friend_index.size() && vertexes[index].friend_index[m] != v2_index) ++m;
              if (m == vertexes[index].friend_index.size())
                Error("Cannot find v2 in friend_index, maybe some errors...\n");
              vertexes[index].friend_index.erase (vertexes[index].friend_index.begin() + m);
              vertexes[index].pairs_index.erase (vertexes[index].pairs_index.begin() + m);
            }

          //refresh plane (one iteration decrease two planes, one vertex)
          planes[pair.triangle_index[0]].is_active = false;
          planes[pair.triangle_index[1]].is_active = false;
          //modify all planes including v2_index to v1_index
          for (int j = 0; j < vertexes[v2_index].pairs_index.size(); ++j)
            {
              if (planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[0]].is_active == true) // not deleted triangles
                {
                  for (int k = 0; k < 3; ++k)
                    if (planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[0]].vertex_index[k] == v2_index) planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[0]].vertex_index[k] = v1_index;
                }
              if (planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[1]].is_active == true) // not deleted triangles
                {
                  for (int k = 0; k < 3; ++k)
                    if (planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[1]].vertex_index[k] == v2_index) planes[pairs[vertexes[v2_index].pairs_index[j]].triangle_index[1]].vertex_index[k] = v1_index;
                }
            }
        }
    }
}

void CPairContraction::CreatePairs(int v1_index, int v2_index, int index)
{
  Pairs pair;
  pair.v1_index = v1_index;
  pair.v2_index = v2_index;
  pair.triangle_index[0] = index;
  pair.triangle_index[1] = -1; // if an edge only belongs to one plane, it doesn't have second triangle
  pair.cost = ComputeCost(vertexes[v1_index].Q, vertexes[v2_index].Q);
  pairs.push_back(pair);
}

void CPairContraction::AddPairs(int v1_index, int v2_index, int triangle_index)
{
  if (!vertexes[v1_index].friend_index.empty()) // list is not empty
    {
      // search if v2 in lists
      std::vector<int>::iterator iter = std::find (vertexes[v1_index].friend_index.begin(), vertexes[v1_index].friend_index.end(), v2_index);
      if (iter == vertexes[v1_index].friend_index.end()) // not find v2
        {
          vertexes[v1_index].friend_index.push_back(v2_index); // add v2 to v1'friend_index
          vertexes[v1_index].pairs_index.push_back(pairs.size());
          vertexes[v2_index].friend_index.push_back(v1_index); // add v1 to v2'friend_index
          vertexes[v2_index].pairs_index.push_back(pairs.size());
          CreatePairs(v1_index, v2_index, triangle_index);
        }
      else // find v2, add plane_index in triangle_index
        {
          int index = std::distance(vertexes[v1_index].friend_index.begin(), iter); // index of this friend in this Vertex
          pairs[vertexes[v1_index].pairs_index[index]].triangle_index[1] = triangle_index;
        }
    }
  else
    {
      vertexes[v1_index].friend_index.push_back(v2_index); // add v2 to v1'friend_index
      vertexes[v1_index].pairs_index.push_back(pairs.size());
      vertexes[v2_index].friend_index.push_back(v1_index); // add v1 to v2'friend_index
      vertexes[v2_index].pairs_index.push_back(pairs.size());
      CreatePairs(v1_index, v2_index, triangle_index);
    }
}

void CPairContraction::SelectPairs()
{
  for (int i = 0; i < m_nTriangles; ++i)
    {
      int v1_index = planes[i].vertex_index[0];
      int v2_index = planes[i].vertex_index[1];
      int v3_index = planes[i].vertex_index[2];
      // add v1 && v2, go to v1
      // TODO: compare length of v1 and v2, choose shorter one
      AddPairs(v1_index, v2_index, i);
      // add v1 && v3, go to v1
      AddPairs(v1_index, v3_index, i);
      // add v2 && v3, go to v2
      AddPairs(v2_index, v3_index, i);
    }
}

int CPairContraction::IsInPairs(int v, Pairs pair)
{
  int v1 = pair.v1_index;
  int v2 = pair.v2_index;
  if (v1 == v) return 0;
  else if (v2 == v) return 1;
  else return -1;
}

void CPairContraction::Run()
{
  SelectPairs();
  BuildHeap();
  Iteration();
  RefreshIndex();
  // output obj with vertexes and planes
}

void CPairContraction::Error(std::string error)
{
  std::cout << error;
  exit(1);
}
