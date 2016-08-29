//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  This is a modified version of Physics2D.
//
//  Physics2D - handles movement and collissions in a simple 2D world.
//  This describes environment physics.

#ifndef EMP_PHYSICS_2D_H
#define EMP_PHYSICS_2D_H

#include <iostream>

#include "../geometry/Surface2D.h"
#include "Body2D.h"
#include "../geometry/Shape2D.h"
#include "../geometry/OwnedShape2D.h"

#include "tools/Random.h"
#include "tools/assert.h"
#include "tools/functions.h"
#include "tools/meta.h"

#include "web/Canvas.h"
#include "web/canvas_utils.h"

#include <vector>
#include <tuple>
#include <utility>
#include <type_traits>
#include <memory>
#include <typeinfo>

// For each owner, create a Surface and add it to surface set.
// Surface set will be indexed by the order in which things were passed into the template.
// Note: OWNER_TYPES should be a **SET** -- no repeat types passed in.
// If repeats are passed in, things *should* work fine. But only the first surface of that type will ever
// be used.

// TODO: eventually make number of surfaces generic
namespace emp {

  // Struct used to pass around collision information.
  // TODO: Is this worth incorporating? Quite a bit of boiler plate. Not sure how much it will actually save us.
  struct CollisionInfo {
    // Phase one detection.
    Point<double> dist;
    double sq_pair_dist;
    double radius_sum;
    double sq_min_dist;
    // Phase two detection.
    double true_dist;
    double overlap_dist;
    double overlap_frac;
    // Other information.
    bool resolved;

    CollisionInfo(Point<double> _dist, double _sq_pair_dist, double _radius_sum, double _sq_min_dist) :
      dist(_dist), sq_pair_dist(_sq_pair_dist), radius_sum(_radius_sum), sq_min_dist(_sq_min_dist),
      true_dist(-1), overlap_dist(-1), overlap_frac(-1), resolved(false) { ; }
    CollisionInfo(Point<double> _dist, double _sq_pair_dist, double _radius_sum, double _sq_min_dist,
                  double _true_dist, double _overlap_dist, double _overlap_frac) :
                  dist(_dist), sq_pair_dist(_sq_pair_dist), radius_sum(_radius_sum), sq_min_dist(_sq_min_dist),
                  true_dist(_true_dist), overlap_dist(_overlap_dist), overlap_frac(_overlap_frac),
                  resolved(false) { ; }
    void Resolve() { resolved = true; }
    bool IsResolved() { return resolved; }
  };

  // Simple physics with CircleBody2D bodies.
  // BODY_TYPES cannot be empty. Each BODY_TYPE will be assigned a unique physics ID.
  // For circle physics, every body must derive from Body<Circle, ...> where ... can be anything.
  template <typename... BODY_OWNERS>
  class CirclePhysics2D {
    protected:
      using Shape_t = Circle;
      using Tracker_t = TypeTracker<BODY_OWNERS...>;
      using Surface_t = Surface2D< OwnedShape<Shape_t, Body<Shape_t>> >;
      //using collision_resolution_fun_type = std::function<void(Body2D_Base *, Body2D_Base *)>;
      //collision_resolution_fun_type collision_resolution_fun;
      Tracker_t body_owner_tt;
      Surface_t *surface;
      Point<double> *max_pos;       // Max position across all surfaces.
      bool configured;              // Have the physics been configured yet?
      emp::Random *random_ptr;
      double max_radius;

      //Signal<Body2D_Base *, Body2D_Base *> on_collision_sig;
      Signal<> on_update_sig;

      //////////////////////////////////////////////////////////////////////////////////////////////
      //                             Internal functions.
      //////////////////////////////////////////////////////////////////////////////////////////////
      // void BuildSurfaceSet(double width, double height, double friction) {
      //   for (int i = 0; i < bodied_type_tracker.GetNumTypes(); i++) {
      //     surface_set.push_back(new Surface2D<OwnedShape<Shape_t, Body<Shape_t>>(width, height, friction));
      //   }
      // }

    public:
      CirclePhysics2D()
        : configured(false)
      {
        emp_assert(sizeof...(BODY_OWNERS) > 0);
      }

      CirclePhysics2D(double width, double height, emp::Random *r, double surface_friction) {
        emp_assert(sizeof...(BODY_OWNERS) > 0);
        ConfigPhysics(width, height, r, surface_friction);
      }

      ~CirclePhysics2D() {
        emp_assert(configured);
        delete surface;
        delete max_pos;
      }

      // Call GetTypeID<type_name>() to get the ID associated with owner type type_name.
      template<typename T>
      constexpr static int GetTypeID() { return get_type_index<T, BODY_OWNERS...>(); }
      // Call GetTypeID(owner) to get the ID associated with 'owner'.
      template <typename T>
      constexpr static int GetTypeID(const T &) { return get_type_index<T, BODY_OWNERS...>(); }

      double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
      double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }
      // emp::vector<Surface*> & GetSurfaceSet() { return surface_set; }
      // const emp::vector<Surface*> & GetConstSurfaceSet() const { return surface_set; }

      CirclePhysics2D & Clear() {
        surface->Clear();
        return *this;
      }

      // Config needs to be able to be called multiple times..
      // Configure physics. This must be called if default constructor was
      // used when creating this.
      void ConfigPhysics(double width, double height, emp::Random *r, double surface_friction) {
        std::cout << "Physics::ConfigPhysics" << std::endl;
        if (configured) { delete surface; }
        // collision_resolution_fun = [this](Body2D_Base * body1, Body2D_Base * body2)
        //                            { this->DefaultCollisionResolution(body1, body2); };
        surface = new Surface_t(width, height, surface_friction);
        max_pos = new Point<double>(width, height);
        random_ptr = r;
        configured = true;
      }

      // Callback registration.
      // TODO: Ensure that this interface is consistent with the rest of Empirical.
      // void RegisterOnUpdateCallback(std::function<void()> callback) {
      //   on_update_sig.AddAction(callback);
      // }
      // void RegisterOnCollisionCallback(std::function<void(Body2D_Base *, Body2D_Base *)> callback) {
      //   on_collision_sig.AddAction(callback);
      // }
      // // Set the fallback collision resolution fun (if all signal triggers fail to resolve collision).
      // void SetCollisionResolutionFun(std::function<void(Body2D_Base *, Body2D_Base *)> fun) {
      //   collision_resolution_fun = fun;
      // }

      template <typename OWNER_TYPE>
      CirclePhysics2D & AddBody(OWNER_TYPE * in_body) {
        emp_assert(configured);
        std::cout << "CirclePhysics::AddBody" << std::endl;
        //in_body->GetBodyPtr()->AttachTrackedOwner(body_owner_tt.New<OWNER_TYPE>(*in_body));
        return *this;
      }

      template <typename BODY_TYPE>
      CirclePhysics2D & RemoveBody(BODY_TYPE * in_body) {
        emp_assert(configured);

        return *this;
      }

      // Progress physics by a single time step.
      void Update() {
        emp_assert(configured);
        std::cout << "Physics update!" << std::endl;
        on_update_sig.Trigger();
        // Reset max_radius.
        max_radius = 0.0;
        // Update all bodies. Remove those marked for removal. Will also find the max radius.
        //UpdateSurfaceBodies();
        // Test for collisions.
        //  - If nothing exists, no need to test for collisions.
        //if (max_radius > 0.0) TestCollisions();
        // Finalize positions and test for stress-induced removal.
        //FinalizeSurfaceBodies();
      }

  };

}

#endif
