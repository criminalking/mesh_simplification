#include <iostream>
#include <stdio.h>
#include "SimpleObject.h"
#include "PairContraction.h"

int main()
{
  SimpleOBJ::CSimpleObject object;
  object.LoadFromObj("obj/Arma.obj"); // . is the address of Makefile
  CPairContraction simplified_obj(object.m_nVertices, object.m_nTriangles, object.m_pVertexList, object.m_pTriangleList);

  return 0;
}
