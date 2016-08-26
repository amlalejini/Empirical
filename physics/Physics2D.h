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

  // TODO: check that all owned shape ty

  // Simple physics with CircleBody2D bodies.
  // BODY_TYPES cannot be empty.
  // TODO: guarantee that BODY_TYPES is not empty
  template <typename... BODY_TYPES>
  class CirclePhysics2D {
    protected:
      using Shape_t = Circle;
      using collision_resolution_fun_type = std::function<void(Body2D_Base *, Body2D_Base *)>;
      emp::vector<Surface*> surface_set;
      Point<double> *max_pos;       // Max position across all surfaces.
      bool configured;              // Have the physics been configured yet?
      emp::Random *random_ptr;
      double max_radius;

      Signal<Body2D_Base *, Body2D_Base *> on_collision_sig;
      Signal<> on_update_sig;

      bool CollideBodies(Body2D_Base * body1, Body2D_Base * body2) {
        // If bodies are linked, no collision.
        if (body1->IsLinked(*body2)) return false;
        // Body-touching math.
        Point<double> dist = body1->GetShapePtr()->GetCenter() - body2->GetShapePtr()->GetCenter();
        double sq_pair_dist = dist.SquareMagnitude();
        const double radius_sum = body1->GetShapePtr()->GetRadius() + body2->GetShapePtr()->GetRadius();
        const double sq_min_dist = radius_sum * radius_sum;
        // If bodies aren't touching, no collision.
        if (sq_pair_dist >= sq_min_dist) return false;
        // Collision!
        body1->TriggerCollision(body2);         // Give bodies chance to resolve first.
        body2->TriggerCollision(body1);
        on_collision_sig.Trigger(body1, body2); // Give whoever registered with collision signal chance to resolve.
        if (body1->IsColliding() || body2->IsColliding())
          this->collision_resolution_fun(body1, body2); // Fallback to default collision function.
        return true;
      }

      // Default collision resolution (called if none of the signals result in collision resolution)]
      // Simple bounce.
      void DefaultCollisionResolution(Body2D_Base * body1, Body2D_Base * body2) {
        std::cout << "Default Collision Resolution!" << std::endl;
        // TODO: this is redundant. Make a collision info struct and pass that around.
        Point<double> dist = body1->GetShapePtr()->GetCenter() - body2->GetShapePtr()->GetCenter();
        double sq_pair_dist = dist.SquareMagnitude();
        const double radius_sum = body1->GetShapePtr()->GetRadius() + body2->GetShapePtr()->GetRadius();
        const double sq_min_dist = radius_sum * radius_sum;

        // If the shapes are on top of each other, we have a problem. Shift one!
        if (sq_pair_dist == 0.0) {
          body2->GetShapePtr()->Translate(Point<double>(0.01, 0.01));
          dist = body1->GetShapePtr()->GetCenter() - body2->GetShapePtr()->GetCenter();
          sq_pair_dist = dist.SquareMagnitude();
        }
        // Re-adjust position to remove overlap.
        const double true_dist = sqrt(sq_pair_dist);
        const double overlap_dist = ((double) radius_sum) - true_dist;
        const double overlap_frac = overlap_dist / true_dist;
        const Point<double> cur_shift = dist * (overlap_frac / 2.0);
        body1->AddShift(cur_shift);   // Split the re-adjustment between the two colliding bodies.
        body2->AddShift(-cur_shift);
        // Resolve collision using impulse resolution.
        const double coefficient_of_restitution = 1.0;
        const Point<double> collision_normal(dist / true_dist);
        const Point<double> rel_velocity(body1->GetVelocity() - body2->GetVelocity());
        const double velocity_along_normal = (rel_velocity.GetX() * collision_normal.GetX()) + (rel_velocity.GetY() * collision_normal.GetY());
        // If velocities are separating, no need to resolve anything further.
        if (velocity_along_normal > 0) {
          // Mark collision as resolved.
          body1->ResolveCollision(); body2->ResolveCollision();
          return;
        }
        double j = -(1 + coefficient_of_restitution) * velocity_along_normal; // Calculate j, the impulse scalar.
        j /= body1->GetInvMass() + body2->GetInvMass();
        const Point<double> impulse(collision_normal * j);
        // Apply the impulse.
        body1->SetVelocity(body1->GetVelocity() + (impulse * body1->GetInvMass()));
        body2->SetVelocity(body2->GetVelocity() - (impulse * body2->GetInvMass()));
        // Mark collision as resolved.
        body1->ResolveCollision(); body2->ResolveCollision();
      }

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

      template <typename FIRST_TYPE>
      void UpdateSurfaceBodies(int i = 0) {
        auto * surface = static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]);
        auto & surface_shapes = surface->GetShapeSet();
        int cur_size = (int) surface_shapes.size();
        int cur_id = 0;
        while (cur_id < cur_size) {
          emp_assert(surface_shapes[cur_id] != nullptr && surface_shapes[cur_id].HasOwner());
          if (surface_shapes[cur_id]->GetOwnerPtr()->GetDestroyFlag()) {
            delete surface_shapes[cur_id]->GetOwnerPtr(); // Delete the body.
            delete surface_shapes[cur_id];                // Delete the shape.
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
          } else {
            surface_shapes[cur_id]->GetOwnerPtr()->BodyUpdate(0.0015, 0.25); // TODO: get rid of magic numbers (friction, change_rate)
            if (surface_shapes[cur_id]->GetRadius() > max_radius)
              max_radius = surface_shapes[cur_id]->GetRadius();
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

      template <typename FIRST_TYPE>
      void FinalizeSurfaceBodies(int i = 0) {
        auto * surface = static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]);
        auto & surface_shapes = surface->GetShapeSet();
        int cur_size = (int) surface_shapes.size();
        int cur_id = 0;
        while (cur_id < cur_size) {
          emp_assert(surface_shapes[cur_id] != nullptr && surface_shapes[cur_id].HasOwner());
          if (surface_shapes[cur_id]->GetOwnerPtr()->GetDestroyFlag() ||
              surface_shapes[cur_id]->GetOwnerPtr()->ExceedsStressThreshold()) {
            delete surface_shapes[cur_id]->GetOwnerPtr(); // Delete the body.
            delete surface_shapes[cur_id];                // Delete the shape.
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
          } else {
            surface_shapes[cur_id]->GetOwnerPtr()->FinalizePosition(*max_pos);
            cur_id++;
          }
        }
        surface_shapes.resize(cur_size);
      }
      template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      void FinalizeSurfaceBodies(int i = 0) {
        FinalizeSurfaceBodies<FIRST_TYPE>(i);
        FinalizeSurfaceBodies<SECOND_TYPE, MORE_TYPES...>(++i);
      }

      // Test for collisions in *this* physics.
      template <typename FIRST_TYPE>
      void TestCollisions(const int num_cols, const int num_rows, const int max_col, const int max_row,
                          const int num_sectors, const double sector_width, const double sector_height,
                          emp::vector<emp::vector<Body2D_Base *>> & sector_set, int i = 0) {
        // Calculate number of sectors to use (currently no more than 1024).
        auto * surface = static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]);
        auto & surface_shapes = surface->GetShapeSet();
        for (auto *shape : surface_shapes) {
          emp_assert(shape != nullptr);
          // Determine which sector the current body is in.
          const int cur_col = emp::to_range<int>(shape->GetCenter().GetX()/sector_width, 0, max_col);
          const int cur_row = emp::to_range<int>(shape->GetCenter().GetY()/sector_height, 0, max_row);
          // See if this body may collide with any of the bodies previously put into sectors.
          for (int k = std::max(0, cur_col-1); k <= std::min(cur_col+1, max_col); k++) {
            for (int j = std::max(0, cur_row-1); j <= std::min(cur_row+1, max_row); j++) {
              const int sector_id = k + num_cols * j;
              if (sector_set[sector_id].size() == 0) continue; // no need to test this body with nothing!
              for (auto *body2 : sector_set[sector_id]) {
                CollideBodies(shape->GetOwnerPtr(), body2);
              }
            }
          }
          // Add this body to the current sector for future collision tests.
          const int cur_sector = cur_col + cur_row * num_cols;
          emp_assert(cur_sector < (int) sector_set.size());
          sector_set[cur_sector].push_back(shape->GetOwnerPtr());
        }

      }
      template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      void TestCollisions(const int num_cols, const int num_rows, const int max_col, const int max_row,
                          const int num_sectors, const double sector_width, const double sector_height,
                          emp::vector<emp::vector<Body2D_Base *>> & sector_set, int i = 0) {

        TestCollisions<FIRST_TYPE>(num_cols, num_rows, max_col, max_row, num_sectors, sector_width,
                                   sector_height, sector_set, i);
        TestCollisions<SECOND_TYPE, MORE_TYPES...>(num_cols, num_rows, max_col, max_row, num_sectors, sector_width,
                                   sector_height, sector_set, ++i);
      }

      template <typename FIRST_TYPE>
      void DrawOnCanvasImpl(web::Canvas canvas, const emp::vector<std::string> & color_map, int i = 0) {
        auto * surface = static_cast<Surface2D<OwnedShape<Circle, FIRST_TYPE>>*>(surface_set[i]);
        auto & surface_shapes = surface->GetShapeSet();
        for (auto * shape : surface_shapes) {
          canvas.Circle(*shape, "", color_map[shape->GetColorID()]);
        }
      }
      template <typename FIRST_TYPE, typename SECOND_TYPE, typename... MORE_TYPES>
      void DrawOnCanvasImpl(web::Canvas canvas, const emp::vector<std::string> & color_map, int i = 0) {
        DrawOnCanvasImpl<FIRST_TYPE>(canvas, color_map, i);
        DrawOnCanvasImpl<SECOND_TYPE, MORE_TYPES...>(canvas, color_map, ++i);
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

      collision_resolution_fun_type collision_resolution_fun;

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
        collision_resolution_fun = [this](Body2D_Base * body1, Body2D_Base * body2)
                                   { this->DefaultCollisionResolution(body1, body2); };
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
      // Set the fallback collision resolution fun (if all signal triggers fail to resolve collision).
      void SetCollisionResolutionFun(std::function<void(Body2D_Base *, Body2D_Base *)> fun) {
        collision_resolution_fun = fun;
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

      // Progress physics by a single time step.
      void Update() {
        emp_assert(configured);
        on_update_sig.Trigger();
        // Reset max_radius.
        max_radius = 0.0;
        // Update all bodies. Remove those marked for removal.
        UpdateSurfaceBodies<BODY_TYPES...>();
        // Test for collisions.
        //  - If nothing exists, no need to test for collisions.
        if (max_radius > 0.0) { // TODO: wrap this up in a TestCollisions function (hide all of the grossness).
          const int num_cols = std::min<int>(GetWidth() / (max_radius * 2.0), 32);
          const int num_rows = std::min<int>(GetHeight() / (max_radius * 2.0), 32);
          const int max_col = num_cols - 1;
          const int max_row = num_rows - 1;
          const int num_sectors = num_cols * num_rows;
          // Calculate sector size.
          const double sector_width = GetWidth() / (double) num_cols;
          const double sector_height = GetHeight() / (double) num_rows;
          emp::vector<emp::vector<Body2D_Base *>> sector_set(num_sectors);

          TestCollisions<BODY_TYPES...>(num_cols, num_rows, max_col, max_row, num_sectors, sector_width,
                                     sector_height, sector_set);
        }
        // Finalize positions and test for stress-induced removal.
        FinalizeSurfaceBodies<BODY_TYPES...>();
      }

      // TODO: move this out of physics eventually
      void DrawOnCanvas(web::Canvas canvas, const emp::vector<std::string> & color_map) {
        canvas.Clear();
        canvas.Rect(0, 0, max_pos->GetX(), max_pos->GetY(), "black");
        DrawOnCanvasImpl<BODY_TYPES...>(canvas, color_map);
      }

  };
}

#endif
