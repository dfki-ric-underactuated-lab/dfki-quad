//
// Created by jm on 08.03.24.
//
#include <iostream>
#include "quad_model_symbolic.hpp"
#include "stdio.h"

int main(int argc, char* argv[]) {
  QuadModelSymbolic test_model(QuadModelSymbolic::UNITREE_QUAD);

//  std::cout << argc << std::endl;
//
//  for (int i = 0; i < argc; ++i) {
//    printf("%s\n", argv[i]);
//  }

  std::cout << test_model.getBaseMass() << std::endl;

  return 0;
}
