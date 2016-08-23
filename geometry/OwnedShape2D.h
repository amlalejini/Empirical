//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  THOUGHT: this could actually just be OwnedGeometry. No reason for it to be limited to shapes.
//
//  OwnedShape class:
//   * Owned shapes will have a ptr to a shape and a ptr to an owner.

namespace emp {

  template <typename SHAPE_TYPE, typename OWNER_TYPE>
  class OwnedShape {
  protected:
    OWNER_TYPE * owner_ptr;
    SHAPE_TYPE * shape_ptr;

    bool has_owner;
    bool has_shape;

  public:

    OwnedShape() : owner_ptr(nullptr), shape_ptr(nullptr), has_owner(false), has_shape(false) { ; }
    OwnedShape(SHAPE_TYPE * s_ptr, OWNER_TYPE * o_ptr)
      : owner_ptr(o_ptr), shape_ptr(s_ptr), has_owner(true), has_shape(true) { ; }

    ~OwnedShape() { if (has_owner) delete owner_ptr; if (has_shape) delete shape_ptr;}

    const OWNER_TYPE * GetOwnerPtr() { return owner_ptr; }
    OWNER_TYPE & GetOwner() { return *owner_ptr; }
    const OWNER_TYPE & GetConstOwner() const { return *owner_ptr; }

    const SHAPE_TYPE * GetShapePtr() { return shape_ptr; }
    SHAPE_TYPE & GetShape() { return *shape_ptr; }
    const SHAPE_TYPE & GetConstShape() const { return *shape_ptr; }

    void AttachShape(SHAPE_TYPE * ptr) {
      emp_assert(ptr != nullptr);
      shape_ptr = ptr;
      has_shape = true;
    }
    void AttachOwner(OWNER_TYPE * ptr) {
      emp_assert(ptr != nullptr);
      owner_ptr = ptr;
      has_owner = true;
    }
  };

}
