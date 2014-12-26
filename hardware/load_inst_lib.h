#ifndef EMP_LOAD_INST_LIB_H
#define EMP_LOAD_INST_LIB_H

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//  This file contains tools to load instructions sets.
//

#include <functional>
using namespace std::placeholders;

#include "HardwareCPU.h"
#include "InstLib.h"

namespace emp {

  // void Load4StackDefault(InstLib<HardwareCPU<>, Instruction> & inst_lib) {
  template <int NUM_STACKS=8, int STACK_SIZE=16, int NUM_ARG_NOPS=8>
  void Load4StackDefault(InstLib<HardwareCPU<NUM_STACKS, STACK_SIZE, NUM_ARG_NOPS>, Instruction> & inst_lib) {
    // Load as many nops as we need.  This we be called Nop-0, Nop-1, Nop-2, etc.
    for (int i = 0; i < NUM_ARG_NOPS; i++) {
      std::string inst_name = "Nop-";
      inst_name += std::to_string(i);
      inst_lib.AddInst(inst_name, std::bind(&HardwareCPU<>::Inst_Nop, _1), 1, i);
    }

    // Load in single-argument math operations.
    inst_lib.AddInst("Inc", std::bind(&HardwareCPU<>::Inst_AddConst, _1, 1, 1, 1));
    inst_lib.AddInst("Dec", std::bind(&HardwareCPU<>::Inst_AddConst, _1, -1, 1, 1));
    inst_lib.AddInst("Shift-L", std::bind(&HardwareCPU<>::Inst_Shift, _1, 1, 1, 1));
    inst_lib.AddInst("Shift-R", std::bind(&HardwareCPU<>::Inst_Shift, _1, -1, 1, 1));

    // Load in double-argument math operations.
    inst_lib.AddInst("Nand", std::bind(&HardwareCPU<>::Inst_Nand, _1, 1, 2, 3));
    inst_lib.AddInst("Add",  std::bind(&HardwareCPU<>::Inst_Add,  _1, 1, 2, 3));
    inst_lib.AddInst("Sub",  std::bind(&HardwareCPU<>::Inst_Sub,  _1, 1, 2, 3));
    inst_lib.AddInst("Mult", std::bind(&HardwareCPU<>::Inst_Mult, _1, 1, 2, 3));
    inst_lib.AddInst("Div",  std::bind(&HardwareCPU<>::Inst_Div,  _1, 1, 2, 3));
    inst_lib.AddInst("Mod",  std::bind(&HardwareCPU<>::Inst_Mod,  _1, 1, 2, 3));
  }

};

#endif
