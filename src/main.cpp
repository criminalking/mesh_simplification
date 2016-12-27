#include <iostream>
#include <stdio.h>
#include "SimpleObject.h"

int main()
{
  SimpleOBJ::CSimpleObject object;
  object.LoadFromObj("obj/Arma.obj"); // . is the address of Makefile
  return 0;
}
