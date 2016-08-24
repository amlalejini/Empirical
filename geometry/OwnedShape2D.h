//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  THOUGHT: this could actually just be OwnedGeometry. No reason for it to be limited to shapes.
//
//  OwnedShape class:
//   *

#ifndef EMP_OWNEDSHAPE_2D_H
#define EMP_OWNEDSHAPE_2D_H

namespace emp {

  template <typename SHAPE_TYPE, typename OWNER_TYPE>
  class OwnedShape : public SHAPE_TYPE {
  protected:
    OWNER_TYPE * owner_ptr;
    bool has_owner;

  public:
    template <typename... ARGS>
    OwnedShape(ARGS... args) :
      SHAPE_TYPE(std::forward<ARGS>(args)...),
      owner_ptr(nullptr),
      has_owner(false)
    { std::cout << "OwnedShape::Constructor(No Owner, ...)" << std::endl; }

    template<typename... ARGS>
    OwnedShape(OWNER_TYPE * o_ptr, ARGS... args) :
      SHAPE_TYPE(std::forward<ARGS>(args)...),
      owner_ptr(o_ptr),
      has_owner(true)
    { std::cout << "OwnedShape::Constructor(Owner, ...)" << std::endl; }

    ~OwnedShape() {
      std::cout << "OwnedShape::~OwnedShape" << std::endl;
    }

    OWNER_TYPE * GetOwnerPtr() { return owner_ptr; }
    OWNER_TYPE & GetOwner() { return *owner_ptr; }
    const OWNER_TYPE & GetConstOwner() const { return *owner_ptr; }

    void AttachOwner(OWNER_TYPE * ptr) {
      emp_assert(ptr != nullptr);
      owner_ptr = ptr;
      has_owner = true;
    }
  };

}

#endif
