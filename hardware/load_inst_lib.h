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
      inst_lib.AddInst(inst_name, std::bind(&HardwareCPU<>::Inst_Nop, _1), i, 1);
    }

    // Load in single-argument math operations.
    inst_lib.AddInst("Inc", std::mem_fn(&HardwareCPU<>::Inst_AddConst<1, 1, 0>));
    inst_lib.AddInst("Dec", std::mem_fn(&HardwareCPU<>::Inst_AddConst<-1, 1, 0>));
    inst_lib.AddInst("Shift-L", std::mem_fn(&HardwareCPU<>::Inst_Shift<1, 1, 0>));
    inst_lib.AddInst("Shift-R", std::mem_fn(&HardwareCPU<>::Inst_Shift<-1, 1, 0>));

    // Load in double-argument math operations.
    inst_lib.AddInst("Nand", std::mem_fn(&HardwareCPU<>::Inst_Nand<1,1,3>));
    inst_lib.AddInst("Add",  std::mem_fn(&HardwareCPU<>::Inst_Add<1,1,3>));
    inst_lib.AddInst("Sub",  std::mem_fn(&HardwareCPU<>::Inst_Sub<1,1,3>));
    inst_lib.AddInst("Mult", std::mem_fn(&HardwareCPU<>::Inst_Mult<1,1,3>));
    inst_lib.AddInst("Div",  std::mem_fn(&HardwareCPU<>::Inst_Div<1,1,3>));
    inst_lib.AddInst("Mod",  std::mem_fn(&HardwareCPU<>::Inst_Mod<1,1,3>));

    // Load in Jump operations  [we neeed to do better...  push and pop heads?]
    inst_lib.AddInst("Jump",     std::mem_fn(&HardwareCPU<>::Inst_MoveHeadToHead<0, 3>));
    inst_lib.AddInst("Bookmark", std::mem_fn(&HardwareCPU<>::Inst_MoveHeadToHead<3, 0>));
    // "Set-Memory" - Jumps the flow head (?2) to memory space 1 (?1).
    // "Find-Label" - Jumps the flow head to a complement label (?...) in its current memory.    

    // Conditionals
    // "If-Equal"
    // "If-NEqual"
    // "If-Less"
    // "If-Label"

    // Juggle stack contents
    // "Val-Move"
    // "Val-Delete"
    // "Val-Copy"

    // Load in "Biological" instructions
    // "Divide"     - Moves memory space 1 (?1) into its own organism.  Needs callback!
    // "Inst-Read"
    // "Inst-Write"
    // "Inst-Copy"
    // "IO"         - Needs callback
    // "Inject" ??  - Needs callback
  }

};

#endif
