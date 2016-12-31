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

float CPairContraction::ComputeCost(Matrix4f Q1, Matrix4f Q2)
{
  // compute v
  Matrix4f Q = Q1 + Q2;
  Matrix4f M;
  M << Q(1,1), Q(1,2), Q(1,3), Q(1,4), Q(1,2), Q(2,2), Q(2,3), Q(2,4), Q(1,3), Q(2,3), Q(3,3), Q(3,4), 0, 0, 0, 1;
  Vector4f vec(0,0,0,1);
  Vector4f v = M.inverse() * vec; // TODO: if invertible
  // compute cost
  float cost = v.transpose() * Q * v;
  return cost;
}

void CPairContraction::AddToHeap(Pairs pair)
{

}

void CPairContraction::BuildHeap()
{

}

void CPairContraction::Iteration(float ratio)
{
  int iter_num = ceil((1 - ratio) * m_nTriangles / 2);
  for (int i = 0; i < iter_num; ++i)
    {


    }
}

void CPairContraction::CreatePairs(int v1_index, int v2_index, int index)
{
  Pairs pair;
  pair.v1_index = v1_index;
  pair.v2_index = v2_index;
  pair.triangle_index[0] = index;
  pair.cost = ComputeCost(vertexes[v1_index].Q, vertexes[v2_index].Q);
  pairs.push_back(pair);
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
              CreatePairs(v1_index, v2_index, i);
            }
          else // find v2, add plane_index in triangle_index
            {
              int index = std::distance(vertexes[v1_index].friend_index.begin(), iter); // index of this friend in this Vertex
              pairs[vertexes[v1_index].pairs_index[index]].triangle_index[1] = i;
            }
        }
      else
        {
          vertexes[v1_index].friend_index.push_back(v2_index); // add v2 to v1'friend_index
          vertexes[v1_index].pairs_index.push_back(pairs.size());
          vertexes[v2_index].friend_index.push_back(v1_index); // add v1 to v2'friend_index
          vertexes[v2_index].pairs_index.push_back(pairs.size());
          CreatePairs(v1_index, v2_index, i);
        }
    }


      // add v2 && v3



      // add v1 && v3
    }
}
