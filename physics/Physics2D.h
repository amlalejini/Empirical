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

#include "tools/Random.h"
#include "tools/assert.h"
#include "tools/functions.h"
#include "tools/meta.h"

// For each owner, create a Surface and add it to surface set.
// Surface set will be indexed by the order in which things were passed into the template.
// Note: OWNER_TYPES should be a **SET** -- no repeat types passed in.
// If repeats are passed in, things *should* work fine. But only the first surface of that type will ever
// be used.

// TODO: eventually make number of surfaces generic
namespace emp {

  // Simple physics with CircleBody2D bodies.
  template <typename... SHAPE_TYPES>
  class SimplePhysics2D {
    protected:
      //using Shape_t = Circle;
      emp::vector<Surface2D<OwnedShape<Shape, Body2D_Base> *> surface_set;

      Point<double> *max_pos;   // Max position across all surfaces.
      bool configured;          // Have the physics been configured yet?
      emp::Random *random_ptr;

      //Signal<BODY_TYPE *, BODY_TYPE *> on_collision_sig;
      Signal<> on_update_sig;

      // Make a new surface for each OWNER_TYPE passed in.
      template <>
      void BuildSurfaceSet(double surface_width, double surface_height, double surface_friction) {
        std::cout << "Done building surface set!" << std::endl;
      }
      template <typename CUR_TYPE, typename... MORE_TYPES>
      void BuildSurfaceSet(double surface_width, double surface_height, double surface_friction) {
        surface_set.push_back(new Surface2D<OwnedShape<CUR_TYPE, Body2D_Base>>(surface_width, surface_height, surface_friction));
        this->BuildSurfaceSet<MORE_TYPES...>(surface_width, surface_height, surface_friction);
      }

    public:
      SimpleCirclePhysics2D()
        : configured(false)
      { ; }

      SimpleCirclePhysics2D(double width, double height, emp::Random *r, double surface_friction) {
        ConfigPhysics(width, height, r, surface_friction);
      }

      ~SimpleCirclePhysics2D() {
        emp_assert(configured);
        for (auto * surface : surface_set) delete surface;
        delete max_pos;
      }

      // Call GetTypeID<type_name>() to get the ID associated with owner type type_name.
      template<typename T>
      constexpr static int GetTypeID() { return get_type_index<T, SHAPE_TYPES...>(); }
      // Call GetTypeID(owner) to get the ID associated with 'owner'.
      template <typename T>
      constexpr static int GetTypeID(const T &) { return get_type_index<T, SHAPE_TYPES...>(); }

      double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
      double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }

      SimpleCirclePhysics2D & Clear() {
        for (auto * surface : surface_set) surface->Clear();
        return *this;
      }

      // Config needs to be able to be called multiple times..
      // Configure physics. This must be called if default constructor was
      // used when creating this.
      void ConfigPhysics(double width, double height, emp::Random *r, double surface_friction) {
        if (configured) {
          // If already configured, delete existing bits and remake them.
          for (auto * surface : surface_set) delete surface;
        }
        this->BuildSurfaceSet<SHAPE_TYPES...>(width, height, surface_friction);
        max_pos = new Point<double>(width, height);
        random_ptr = r;
        configured = true;
      }

      template<typename BODY_TYPE>
      SimpleCirclePhysics2D & AddBody(BODY_TYPE * in_body) {
        emp_assert(configured);
        int idx = GetTypeID(in_body->GetConstOwner());

        surface_set[idx].AddShape(new OwnedShape<Shape_t, Body<Shape_t, OWNER> >(in_body->GetShapePtr(), in_body));
      }

      // SimpleCirclePhysics2D & RemoveBody(BODY_TYPE * in_body) {
      //   emp_assert(in_body->HasOwner(), configured);
      //   org_surface->RemoveBody(in_body);
      //   return *this;
      // }

      // Test for collisions in *this* physics.
      void TestCollisions() {

      }

      // Progress physics by a single time step.
      void Update() {
        emp_assert(configured);
        std::cout << "Physics update." << std::endl;
      }

  };


  // // Simple physics with CircleBody2D bodies.
  // template <typename... BODY_OWNER_TYPES>
  // class SimpleCirclePhysics2D {
  //   protected:
  //     // using OrgSurface_t = Surface2D<OwnedShape<Circle, Body2D_Base>>;
  //     // using ResourceSurface_t = Surface2D<OwnedShape<Circle, Body2D_Base>>;
  //     // ResourceSurface_t *resource_surface;
  //     // OrgSurface_t *org_surface;
  //     using Shape_t = Circle;
  //     emp::vector<Surface2D<OwnedShape<Shape_t, Body2D_Base> *> surface_set;
  //
  //     Point<double> *max_pos;   // Max position across all surfaces.
  //     bool configured;          // Have the physics been configured yet?
  //     emp::Random *random_ptr;
  //
  //     //Signal<BODY_TYPE *, BODY_TYPE *> on_collision_sig;
  //     Signal<> on_update_sig;
  //
  //     // Make a new surface for each OWNER_TYPE passed in.
  //     template <>
  //     void BuildSurfaceSet(double surface_width, double surface_height, double surface_friction) {
  //       std::cout << "Done building surface set!" << std::endl;
  //     }
  //     template <typename CUR_TYPE, typename... MORE_TYPES>
  //     void BuildSurfaceSet(double surface_width, double surface_height, double surface_friction) {
  //       surface_set.push_back(new Surface2D<OwnedShape<Shape_t, CUR_TYPE>(surface_width, surface_height, surface_friction));
  //       this->BuildSurfaceSet<MORE_TYPES...>(surface_width, surface_height, surface_friction);
  //     }
  //
  //   public:
  //     SimpleCirclePhysics2D()
  //       : configured(false)
  //     { ; }
  //
  //     SimpleCirclePhysics2D(double width, double height, emp::Random *r, double surface_friction) {
  //       ConfigPhysics(width, height, r, surface_friction);
  //     }
  //
  //     ~SimpleCirclePhysics2D() {
  //       emp_assert(configured);
  //       for (auto * surface : surface_set) delete surface;
  //       delete max_pos;
  //     }
  //
  //     // Call GetTypeID<type_name>() to get the ID associated with owner type type_name.
  //     template<typename T>
  //     constexpr static int GetTypeID() { return get_type_index<T, BODY_OWNER_TYPES...>(); }
  //     // Call GetTypeID(owner) to get the ID associated with 'owner'.
  //     template <typename T>
  //     constexpr static int GetTypeID(const T &) { return get_type_index<T, BODY_OWNER_TYPES...>(); }
  //
  //     const OrgSurface_t & GetOrgSurface() const { emp_assert(configured); return *org_surface; }
  //     const ResourceSurface_t & GetResourceSurface() const { emp_assert(configured); return *resource_surface; }
  //     //const emp::vector<Surface2D<BODY_TYPE> *> & GetSurfaceSet() const { return surface_set; }
  //
  //     double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
  //     double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }
  //
  //     SimpleCirclePhysics2D & Clear() {
  //       for (auto * surface : surface_set) surface->Clear();
  //       return *this;
  //     }
  //
  //     // Config needs to be able to be called multiple times..
  //     // Configure physics. This must be called if default constructor was
  //     // used when creating this.
  //     void ConfigPhysics(double width, double height, emp::Random *r, double surface_friction) {
  //       if (configured) {
  //         // If already configured, delete existing bits and remake them.
  //         for (auto * surface : surface_set) delete surface;
  //       }
  //       this->BuildSurfaceSet<BODY_OWNER_TYPES...>(width, height, surface_friction);
  //       max_pos = new Point<double>(width, height);
  //       random_ptr = r;
  //       configured = true;
  //     }
  //
  //     // template<typename OWNER>
  //     // SimpleCirclePhysics2D & AddBody(Body2D_Base * in_body, OWNER * body_owner) {
  //     //
  //     // }
  //
  //     template<typename BODY_TYPE>
  //     SimpleCirclePhysics2D & AddBody(BODY_TYPE * in_body) {
  //       emp_assert(configured);
  //       int idx = GetTypeID(in_body->GetConstOwner());
  //
  //       surface_set[idx].AddShape(new OwnedShape<Shape_t, Body<Shape_t, OWNER> >(in_body->GetShapePtr(), in_body));
  //     }
  //
  //     // SimpleCirclePhysics2D & RemoveBody(BODY_TYPE * in_body) {
  //     //   emp_assert(in_body->HasOwner(), configured);
  //     //   org_surface->RemoveBody(in_body);
  //     //   return *this;
  //     // }
  //
  //     // Test for collisions in *this* physics.
  //     void TestCollisions() {
  //
  //     }
  //
  //     // Progress physics by a single time step.
  //     void Update() {
  //       emp_assert(configured);
  //       std::cout << "Physics update." << std::endl;
  //     }
  //
  //     // TODO: get OWNERXsurface
  //     // TODO: get OWNERxBodySet
  //     // emp::vector<BODY_TYPE *> & GetBodySet() {
  //     //   emp_assert(configured);
  //     //   return org_surface->GetBodySet();
  //     // }
  //     //
  //     // emp::vector<BODY_TYPE *> & GetResourceBodySet() {
  //     //   emp_assert(configured);
  //     //   return resource_surface->GetBodySet();
  //     // }
  //     //
  //     // const emp::vector<BODY_TYPE *> & GetConstOrgBodySet() const {
  //     //   emp_assert(configured);
  //     //   return org_surface->GetConstBodySet();
  //     // }
  //     //
  //     // const emp::vector<BODY_TYPE *> & GetConstResourceBodySet() const {
  //     //   emp_assert(configured);
  //     //   return resource_surface->GetConstBodySet();
  //     // }
  //
  // };
}

#endif
