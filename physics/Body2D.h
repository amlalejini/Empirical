//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//
// Body_Base is a base class, containing info unrelated to its shape or its owner.
// It *does* contain the set of signals that might need to be triggered, such as
// specific types of collisions.
//
// Body is a template class that takes SHAPE and OWNER as template arguments.
// It creates an intenral instance of SHAPE (which has a pointer back to Body_Base)
// and a pointer back to OWNER.
//
//
//  Development notes:
//  * If we are going to have a lot of links, we may want a better data structure than vector.
//    (if we don't have a lot, vector may be the best choice...)

#ifndef EMP_BODY_2D_H
#define EMP_BODY_2D_H

#include "../geometry/Angle2D.h"
#include "../geometry/Shape2D.h"
#include "../geometry/OwnedShape2D.h"

#include "tools/assert.h"
#include "tools/alert.h"
#include "tools/Ptr.h"
#include "tools/vector.h"
#include "tools/signal.h"

#include <iostream>
#include <functional>

namespace emp {
  // TODO: Discuss the fate of BODY_LINK_TYPE. Should we still be using this? Or is there something
  //  better?
  // Bodies can be linked in several ways.
  // DEFAULT -> Joined together with no extra meaning
  // REPRODUCTION -> "from" is gestating "to"
  // ATTACK -> "from" is trying to eat "to"
  // PARASITE -> "from" is stealing resources from "to"
  // CONSUME_RESOURCE -> "from" is eating "to" where "from" is an organism and "to" is a resource.
  enum class BODY_LINK_TYPE { DEFAULT, REPRODUCTION, ATTACK, PARASITE, CONSUME_RESOURCE };

  class Body2D_Base {
  protected:
    // Body properties.
    Point<double> velocity; // Speed and direction of movement.
    double mass;
    double inv_mass;
    double pressure;
    double max_pressure;

    // Useful internal member variables.
    Point<double> shift;            // How should this body be updated to minimize overlap?
    Point<double> cum_shift;        // Build up of shift not yet acted upon.
    Point<double> total_abs_shift;  // Total absolute-value of shifts (to calculate pressure)

    // TODO: body signals etc
    Signal<Body2D_Base*> on_collision_sig;
    Signal<> on_destruction_sig;

    Body2D_Base() : mass(1.0), inv_mass(1 / mass), pressure(0.0), max_pressure(1.0) { ; }

  public:
    virtual ~Body2D_Base() { ; }



  };

  // Example: a body could be in the shape of a circle and be owned by a resource.
  template<typename SHAPE_TYPE, typename OWNER_TYPE = std::nullptr_t>
  class Body : public Body2D_Base {
  protected:
    struct BodyLink {
      using BODY_TYPE = Body2D_Base;
      BODY_LINK_TYPE type;   // DEFAULT, REPRODUCTION, ATTACK, PARASITE
      BODY_TYPE * from;      // Initiator of the connection (e.g., parent, attacker)
      BODY_TYPE * to;        // Target of the connection (e.g., offspring, prey/host)
      double cur_dist;       // How far are bodies currently being kept apart?
      double target_dist;    // How far should the be moved to? (e.g., if growing)
      double link_strength;  // How strong is the link? (used to determine who wins in competive links)

      BodyLink() : type(BODY_LINK_TYPE::DEFAULT), from(nullptr), to(nullptr), cur_dist(0)
                 , target_dist(0), link_strength(0) { ; }
      BodyLink(BODY_LINK_TYPE t, BODY_TYPE * _frm, BODY_TYPE * _to, double cur=0, double target=0, double lnk_str=0)
        : type(t), from(_frm), to(_to), cur_dist(cur), target_dist(target), link_strength(lnk_str) { ; }
      BodyLink(const BodyLink &) = default;
      ~BodyLink() { ; }
    };

    using Shape_t = OwnedShape<SHAPE_TYPE, Body>;
    Shape_t * shape_ptr; // circle, rectangle, etc.
    OWNER_TYPE * owner_ptr; // organism, resource, etc.
    bool has_owner;

  public:
    // TODO - QUESTION - When constructing body, require construction of shape as well?
    template <typename... ARGS>
    Body(ARGS... args) :
      owner_ptr(nullptr),
      has_owner(false)
    {
      std::cout << "Body Constructor. Not defining an owner." << std::endl;
      shape_ptr = new Shape_t(this, std::forward<ARGS>(args)...);
    }

    template <typename... ARGS>
    Body(OWNER_TYPE * o_ptr, ARGS... args) :
      owner_ptr(o_ptr),
      has_owner(true)
    {
      std::cout << "Body Constructor. Defining an owner. " << std::endl;
      shape_ptr = new Shape_t(this, std::forward<ARGS>(args)...);
    }

    const Shape_t * GetShapePtr() { return shape_ptr; }
    Shape_t & GetShape() { return *shape_ptr; }
    const Shape_t & GetConstShape() { return *shape_ptr; }

    const OWNER_TYPE * GetBodyOwnerPtr() { return owner_ptr; }
    OWNER_TYPE & GetBodyOwner() { return owner_ptr; }
    const OWNER_TYPE & GetConstBodyOwner() { return owner_ptr; }

    void AttachOwner(OWNER_TYPE * ptr) {
      emp_assert(ptr != nullptr);
      owner_ptr = ptr;
      has_owner = true;
    }
    // TODO: alert owner that body is no longer attached? -- Probably.
    void DetachOwner() {
      owner_ptr = nullptr;
      has_owner = false;
    }



  };
}

#endif
