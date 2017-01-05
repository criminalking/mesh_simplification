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
  M << Q(0,0), Q(0,1), Q(0,2), Q(0,3), Q(0,1), Q(1,1), Q(1,2), Q(1,3), Q(0,2), Q(1,2), Q(2,2), Q(2,3), 0, 0, 0, 1;
  Vector4f vec(0,0,0,1);
  //solve Mv=vec
  ColPivHouseholderQR<Matrix4f> dec(M); // not invertible is also ok
  Vector4f v = dec.solve(vec);
  // float determinant = M.determinant();
  // Vector4f v2;
  // if (determinant > 1e-6) // not singular
  //    v2 = M.inverse() * vec;
  // else Error("Matrix is singular!!!\n"); // TODO: if singular, how should do?
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
  // already verified: no repeat
}

void CPairContraction::Iteration()
{
  while(m_nTriangles > m_nTriangles_new)
    {
      if (heap.empty()) Error("Heap is empty now! Maybe some errors!\n");
      else
        {
          int begin = heap.size();
          std::cout << heap.size() << "size begin\n";
          Pairs pair_delete = heap.top();
          heap.pop(); // delete the top element

          int v1_index = pair_delete.v1_index;
          int v2_index = pair_delete.v2_index;

          std::cout << "v1: "<< v1_index <<"," << "v2: " <<  v2_index << std::endl;

          Matrix4f v_Q = vertexes[v1_index].Q + vertexes[v2_index].Q; // new v of (v1, v2)
          // refresh heap
          // find another vertex of planes including v1 and v2
          int plane1 = pair_delete.triangle_index[0];
          int plane2 = pair_delete.triangle_index[1];
          int v3_index = -1, v4_index = -1;
          for (int k = 0; k < 3; ++k)
            {
              if (planes[plane1].vertex_index[k] != v1_index && planes[plane1].vertex_index[k] != v2_index) v3_index = planes[plane1].vertex_index[k];
              if (plane2 != -1 && planes[plane2].vertex_index[k] != v1_index && planes[plane2].vertex_index[k] != v2_index) v4_index = planes[plane2].vertex_index[k];
            }
          if (v3_index < 0)
            {
              Error("No plane of this edge!!!\n");
            }

          std::cout << "v3: "<< v3_index << "," << "v4: " <<  v4_index << std::endl;

          int plane13_0 = -1, plane13_1 = -1, plane14_0 = -1, plane14_1 = -1;
          Heap new_heap;
          while (!heap.empty()) // traverse heap
            {
              Pairs pair = heap.top(); // get one pair from this heap
              heap.pop();

              if (IsThisPair(v1_index, v2_index, pair)) std::cout << v1_index << v2_index<< "first" <<std::endl; // check

              // if pair = (v1, v3)
              if (IsThisPair(v1_index, v3_index, pair))
                {
                  std::cout << "13\n";
                  if (IsThisPlane(v1_index, v2_index, v3_index, pair.triangle_index[0])) plane13_0 = pair.triangle_index[0]; // this plane is (v1, v2, v3)
                  else if (IsThisPlane(v1_index, v2_index, v3_index, pair.triangle_index[1])) plane13_0 = pair.triangle_index[1];
                }
              // if pair = (v1, v4) TODO: need consider -1
              else if (IsThisPair(v1_index, v4_index, pair))
                {
                  std::cout << "14\n";
                  if (IsThisPlane(v1_index, v2_index, v4_index, pair.triangle_index[0])) plane14_0 = pair.triangle_index[0];
                  else if (IsThisPlane(v1_index, v2_index, v4_index, pair.triangle_index[1])) plane14_0 = pair.triangle_index[1];
                }
              // if pair = (v2, v3)
              else if (IsThisPair(v2_index, v3_index, pair))
                {
                  std::cout << "23\n";
                  if (!IsThisPlane(v1_index, v2_index, v3_index, pair.triangle_index[0])) plane13_1 = pair.triangle_index[0];
                  else if (!IsThisPlane(v1_index, v2_index, v3_index, pair.triangle_index[1])) plane13_1 = pair.triangle_index[1];

                  if (plane13_1 == -1)
                    {
                      PrintPlane(pair.triangle_index[0]);
                      PrintPlane(pair.triangle_index[0]);
                    }

                }
              // if pair = (v2, v4)
              else if (IsThisPair(v2_index, v4_index, pair))
                {
                  std::cout << "24\n";
                  if (!IsThisPlane(v1_index, v2_index, v4_index, pair.triangle_index[0])) plane14_1 = pair.triangle_index[0];
                  else if (!IsThisPlane(v1_index, v2_index, v4_index, pair.triangle_index[1])) plane14_1 = pair.triangle_index[1];
                }
              // if v1 in this pair && another vertex is not v3 or v4
              else if (v1_index == pair.v1_index || v1_index == pair.v2_index)
                {
                  // TODO: use operator
                  Pairs new_pair;
                  new_pair.v1_index = pair.v1_index;
                  new_pair.v2_index = pair.v2_index;
                  new_pair.triangle_index[0] = pair.triangle_index[0];
                  new_pair.triangle_index[1] = pair.triangle_index[1];

                  if (v1_index == pair.v1_index) // v1 is in the first place
                    {
                      new_pair.cost = ComputeCost(v_Q, vertexes[pair.v2_index].Q); // use v instead of v1
                    }
                  else // v1 is in the second place
                    {
                      new_pair.cost = ComputeCost(v_Q, vertexes[pair.v1_index].Q); // use v instead of v1
                    }
                  new_heap.push(new_pair);
                }

              // if v2 in this pair && another vertex is not v3 or v4
              else if (v2_index == pair.v1_index || v2_index == pair.v2_index)
                {
                  Pairs new_pair;
                  if (v2_index == pair.v1_index) // v2 is in the first place
                    {
                      new_pair.v1_index = v1_index;
                      new_pair.v2_index = pair.v2_index;
                      new_pair.cost = ComputeCost(v_Q, vertexes[pair.v2_index].Q); // use v instead of v2
                    }
                  else if (v2_index == pair.v2_index) // v2 is in the second place
                    {
                      new_pair.v1_index = pair.v1_index;
                      new_pair.v2_index = v1_index;
                      new_pair.cost = ComputeCost(v_Q, vertexes[pair.v1_index].Q); // use v instead of v2
                    }
                  new_pair.triangle_index[0] = pair.triangle_index[0];
                  new_pair.triangle_index[1] = pair.triangle_index[1];
                  // change v2 to v1
                  for (int k = 0; k < 3; ++k)
                    {
                      if (planes[pair.triangle_index[0]].vertex_index[k] == v2_index) planes[pair.triangle_index[0]].vertex_index[k] = v1_index;
                      if (planes[pair.triangle_index[1]].vertex_index[k] == v2_index) planes[pair.triangle_index[1]].vertex_index[k] = v1_index;
                    }
                  new_heap.push(new_pair);
                }
              else new_heap.push(pair);
            }
          // swap heap and new heap
          heap.swap(new_heap);

          // push (v1,v3) && (v1,v4)
          if (plane13_0 == -1 || plane13_1 == -1)
            Error("13 == -1\n");
          Pairs pair13;
          pair13.v1_index = v1_index; pair13.v2_index = v3_index;
          pair13.cost = ComputeCost(v_Q, vertexes[v3_index].Q);
          pair13.triangle_index[0] = plane13_0; pair13.triangle_index[1] = plane13_1;
          heap.push(pair13);
          if (v4_index != -1)
            {
              Pairs pair14;
              pair14.v1_index = v1_index; pair14.v2_index = v4_index;
              pair14.cost = ComputeCost(v_Q, vertexes[v4_index].Q);
              pair14.triangle_index[0] = plane14_0; pair14.triangle_index[1] = plane14_1;
              heap.push(pair14);
            }

          int end = heap.size();
          std::cout << heap.size() << "size end\n";
          if (begin - end < 3) std::cout << v4_index << "v4\n";

          //refresh vertex, modify friend_index/pairs_index
          m_nVertices--;
          vertexes[v2_index].is_active = false;
          vertexes[v1_index].Q += vertexes[v2_index].Q;
          vertexes[v1_index].v = ComputeV(vertexes[v1_index].Q);
          //delete v2 in his friends' f_i(including v1)
          for (int k = 0; k < vertexes[v2_index].friend_index.size(); ++k)
            {
              int index = vertexes[v2_index].friend_index[k];

              if (index == v1_index || index == v3_index || index == v4_index) // friend is v1,v3,v4
                {
                  // delete v2 in v1,v3,v4
                  int m = 0;
                  while (m != vertexes[index].friend_index.size() && vertexes[index].friend_index[m] != v2_index) ++m;
                  if (m == vertexes[index].friend_index.size())
                    Error("Cannot find v2 in friend_index, maybe some errors...\n");
                  vertexes[index].friend_index.erase(vertexes[index].friend_index.begin() + m);
                }
              else // change v2 to v1 in them, add them in v1
                {
                  int m = 0;
                  while (m != vertexes[index].friend_index.size() && vertexes[index].friend_index[m] != v2_index) ++m;
                  if (m == vertexes[index].friend_index.size()) Error("Cannot find v2 in friend_index, maybe some errors...\n");
                  vertexes[index].friend_index[m] = v1_index;
                  vertexes[v1_index].friend_index.push_back(index);
                }
            }

          //refresh plane (one iteration decrease one/two planes, one vertex)
          planes[plane1].is_active = false;
          m_nTriangles--;
          if (plane2 != -1) // two planes
            {
              planes[plane2].is_active = false;
              m_nTriangles--;
            }
        }
    }
}

bool CPairContraction::PointInPlane(int v_index, int p_index)
{
  if (planes[p_index].vertex_index[0] == v_index || planes[p_index].vertex_index[1] == v_index || planes[p_index].vertex_index[2] == v_index) return true;
  else return false;
}

bool CPairContraction::IsThisPair(int v1, int v2, Pairs pair)
{
  if ((v1 == pair.v1_index && v2 == pair.v2_index) || (v1 == pair.v2_index && v2 == pair.v1_index)) return true;
  else return false;
}

bool CPairContraction::IsThisPlane(int v1, int v2, int v3, int plane)
{
  if (PointInPlane(v1, plane) && PointInPlane(v2, plane) && PointInPlane(v3, plane))
    return true;
  else return false;
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
      // search if v2 in v1's lists
      int index = vertexes[v1_index].friend_index.size();
      for (int i = 0; i < vertexes[v1_index].friend_index.size(); ++i)
        if (vertexes[v1_index].friend_index[i] == v2_index) index = i;
      if (index == vertexes[v1_index].friend_index.size()) // not find v2
        {
          vertexes[v1_index].friend_index.push_back(v2_index); // add v2 to v1'friend_index
          vertexes[v1_index].pairs_index.push_back(pairs.size());
          vertexes[v2_index].friend_index.push_back(v1_index); // add v1 to v2'friend_index
          vertexes[v2_index].pairs_index.push_back(pairs.size());
          CreatePairs(v1_index, v2_index, triangle_index);
        }
      else // find v2, add plane_index in triangle_index
        {
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
  std::clock_t start;
  start = std::clock();
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
  double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"time: "<< duration <<'\n';
}

void CPairContraction::RefreshIndex(int &object_nVertices, int &object_nTriangles, SimpleOBJ::Vec3f* m_pVertexList, SimpleOBJ::Array<int,3>* m_pTriangleList)
{
  // if vertex's is_active == false, swap this vertex with the last one, size--,
  int k = 0; // record deleted number
  std::vector<int> new_index;
  for (int i = 0; i < vertexes.size(); ++i) new_index.push_back(i); // old index
  // refresh vertexes
  for (int i = 0; i < vertexes.size(); ++i)
    {
      if (!vertexes[i].is_active)
        {
          k++;
          new_index[i] = -1;
        }
      else
        {
          new_index[i] -= k;
        }
    }
  for (int i = 0; i < vertexes.size(); ++i)
    {
      if (new_index[i] >= 0) vertexes[new_index[i]] = vertexes[i];
    }
  vertexes.resize(vertexes.size()-k);

  //refresh planes
  k = 0;
  std::vector<int> new_index_plane;
  for (int i = 0; i < planes.size(); ++i) new_index_plane.push_back(i); // old index
  for (int i = 0; i < planes.size(); ++i)
    {
      if (!planes[i].is_active)
        {
          k++;
          new_index_plane[i] = -1;
        }
      else
        {
          // modify index of vertexes in this plane
          for (int j = 0; j < 3; ++j)
            {
              if (new_index[planes[i].vertex_index[j]] == -1) Error("Strange!\n");
              planes[i].vertex_index[j] = new_index[planes[i].vertex_index[j]];
            }
          new_index_plane[i] -= k;
        }
    }
  for (int i = 0; i < planes.size(); ++i)
    {
      if (new_index_plane[i] >= 0) planes[new_index_plane[i]] = planes[i];
    }
  planes.resize(planes.size()-k);

  // create new m_pVertexList, m_pTriangleList
  object_nVertices = m_nVertices;
  object_nTriangles = m_nTriangles;
  for (int i = 0; i < m_nVertices; ++i)
    {
      m_pVertexList[i][0] = vertexes[i].v[0];
      m_pVertexList[i][1] = vertexes[i].v[1];
      m_pVertexList[i][2] = vertexes[i].v[2];
    }
  for (int i = 0; i < m_nTriangles; ++i)
    {
      m_pTriangleList[i][0] = planes[i].vertex_index[0];
      m_pTriangleList[i][1] = planes[i].vertex_index[1];
      m_pTriangleList[i][2] = planes[i].vertex_index[2];
    }
}

void CPairContraction::Run()
{
  SelectPairs();
  BuildHeap();
  Iteration();
  std::cout << "New Vertex Number = " << m_nVertices << "\nNew Triangle Number = " << m_nTriangles << std::endl;
  // output obj with vertexes and planes
}

void CPairContraction::Error(std::string error)
{
  std::cout << error;
  exit(1);
}
