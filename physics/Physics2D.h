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

  // TODO: check that all owned shape ty

  // Simple physics with CircleBody2D bodies.
  // BODY_TYPES cannot be empty.
  // TODO: guarantee that BODY_TYPES is not empty
  template <typename... BODY_TYPES>
  class CirclePhysics2D {
    protected:
      using Shape_t = Circle;
      emp::vector<Surface*> surface_set;
      Point<double> *max_pos;       // Max position across all surfaces.
      bool configured;              // Have the physics been configured yet?
      emp::Random *random_ptr;

      Signal<Body2D_Base *, Body2D_Base *> on_collision_sig;
      Signal<> on_update_sig;

      // TODO: clean up template-meta programming stuff to have _imp versions and public versions.
      // Template meta-programming magic to build surface set.
      template <typename FIRST_TYPE>
      void BuildSurfaceSet(double width, double height, double friction) {
        // Build set.
        surface_set.push_back(new Surface2D<OwnedShape<Shape_t, FIRST_TYPE>>(width, height, friction));
      }
      template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      void BuildSurfaceSet(double width, double height, double friction) {
        BuildSurfaceSet<FIRST_TYPE>(width, height, friction);
        BuildSurfaceSet<SECOND_TYPE, MORE_TYPES...>(width, height, friction);
      }

      // Internal funtion used to find the correct surface and apply a function to that surface.
      template <typename BODY_TYPE>
      void FindSurfaceApplyFun(BODY_TYPE * in_body,
                               std::function<void(Surface2D<OwnedShape<Circle, BODY_TYPE>>*)> fun,
                               int i = 0) { emp_assert(false && "Failed to find correct surface."); }
      template <typename BODY_TYPE, typename FIRST_TYPE, typename... MORE_TYPES>
      void FindSurfaceApplyFun(BODY_TYPE * in_body,
                               std::function<void(Surface2D<OwnedShape<Circle, BODY_TYPE>>*)> fun,
                               int i = 0) {
        // Does BODY_TYPE match FIRST_TYPE?
        if (std::is_same<BODY_TYPE, FIRST_TYPE>())
          fun(static_cast<Surface2D<OwnedShape<Circle, BODY_TYPE>>*>(surface_set[i]));
        else
          FindSurfaceApplyFun<BODY_TYPE, MORE_TYPES...>(in_body, fun, ++i);
      }

      // This doesn't do what I want becuas FIRST type is figured out via type deduction.
      // template <typename FIRST_TYPE>
      // void LoopSurfacesApplyFun(std::function<void(Surface2D<OwnedShape<Circle, FIRST_TYPE>>*)> fun, int i = 0) {
      //   fun(static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]));
      // }
      // template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      // void LoopSurfacesApplyFun(std::function<void(Surface2D<OwnedShape<Circle, FIRST_TYPE>>*)> fun, int i = 0) {
      //   LoopSurfacesApplyFun<FIRST_TYPE>(fun, i);
      //   LoopSurfacesApplyFun<SECOND_TYPE, MORE_TYPES...>(fun, ++i);
      // }
      template <typename FIRST_TYPE>
      void UpdateSurfaceBodies(int i = 0) {
        std::cout << "Updating bodies on surface: " << i << std::endl;
        auto * surface = static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]);
        auto & surface_shapes = surface->GetShapeSet();
        int cur_size = (int) surface_shapes.size();
        int cur_id = 0;
        std::cout << " - Num bodies to update: " << cur_size << std::endl;
        while (cur_id < cur_size) {
          emp_assert(surface_shapes[cur_id] != nullptr && surface_shapes[cur_id].HasOwner());
          std::cout << "   * Looking at a shape!" << std::endl;
          if (surface_shapes[cur_id]->GetOwnerPtr()->GetDestroyFlag()) {
            std::cout << "   * Deleting a shape." << std::endl;
            delete surface_shapes[cur_id]->GetOwnerPtr(); // Delete the body.
            delete surface_shapes[cur_id];                // Delete the shape.
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
          } else {
            surface_shapes[cur_id]->GetOwnerPtr()->BodyUpdate();
            cur_id++;
          }
        }
        surface_shapes.resize(cur_size);
      }
      template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      void UpdateSurfaceBodies(int i = 0) {
        UpdateSurfaceBodies<FIRST_TYPE>(i);
        UpdateSurfaceBodies<SECOND_TYPE, MORE_TYPES...>(++i);
      }

    public:
      CirclePhysics2D()
        : configured(false)
      { ; }

      CirclePhysics2D(double width, double height, emp::Random *r, double surface_friction) {
        ConfigPhysics(width, height, r, surface_friction);
      }

      ~CirclePhysics2D() {
        emp_assert(configured);
        for (auto * surface : surface_set) delete surface;
        delete max_pos;
      }

      // Call GetTypeID<type_name>() to get the ID associated with owner type type_name.
      template<typename T>
      constexpr static int GetTypeID() { return get_type_index<T, BODY_TYPES...>(); }
      // Call GetTypeID(owner) to get the ID associated with 'owner'.
      template <typename T>
      constexpr static int GetTypeID(const T &) { return get_type_index<T, BODY_TYPES...>(); }

      double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
      double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }
      emp::vector<Surface*> & GetSurfaceSet() { return surface_set; }
      const emp::vector<Surface*> & GetConstSurfaceSet() const { return surface_set; }

      CirclePhysics2D & Clear() {
        for (auto * surface : surface_set) surface->Clear();
        return *this;
      }

      // Config needs to be able to be called multiple times..
      // Configure physics. This must be called if default constructor was
      // used when creating this.
      void ConfigPhysics(double width, double height, emp::Random *r, double surface_friction) {
        if (configured) { for (auto * surface : surface_set) delete surface; }
        BuildSurfaceSet<BODY_TYPES...>(width, height, surface_friction);
        max_pos = new Point<double>(width, height);
        random_ptr = r;
        configured = true;
      }

      // Callback registration.
      // TODO: Ensure that this interface is consistent with the rest of Empirical.
      void RegisterOnUpdateCallback(std::function<void()> callback) {
        on_update_sig.AddAction(callback);
      }
      void RegisterOnCollisionCallback(std::function<void(Body2D_Base *, Body2D_Base *)> callback) {
        on_collision_sig.AddAction(callback);
      }

      template <typename BODY_TYPE>
      CirclePhysics2D & AddBody(BODY_TYPE * in_body) {
        emp_assert(configured);
        this->FindSurfaceApplyFun<BODY_TYPE, BODY_TYPES...>(in_body,
                                      [in_body](Surface2D<OwnedShape<Circle, BODY_TYPE>> * s_ptr) {
                                          s_ptr->AddShape(in_body->GetShapePtr());
                                      });
        return *this;
      }

      template <typename BODY_TYPE>
      CirclePhysics2D & RemoveBody(BODY_TYPE * in_body) {
        emp_assert(configured);
        this->FindSurfaceApplyFun<BODY_TYPE, BODY_TYPES...>(in_body,
                                      [in_body](Surface2D<OwnedShape<Circle, BODY_TYPE>> * s_ptr) {
                                          s_ptr->RemoveShape(in_body->GetShapePtr());
                                      });
        return *this;
      }

      // Test for collisions in *this* physics.
      void TestCollisions() {

      }

      // Progress physics by a single time step.
      void Update() {
        emp_assert(configured);
        std::cout << "Physics update." << std::endl;
        on_update_sig.Trigger();

        // Update all bodies. Remove those marked for removal.
        UpdateSurfaceBodies<BODY_TYPES...>();
      }

  };
}

#endif
