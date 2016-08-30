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
      using Tracker_t = TypeTracker<BODY_OWNERS*...>;
      using Surface_t = Surface2D< OwnedShape<Shape_t, Body<Shape_t>> >;
      using collision_resolution_fun_type = std::function<void(Body<Shape_t> *, Body<Shape_t> *)>;
      collision_resolution_fun_type collision_resolution_fun;
      Tracker_t body_owner_tt;
      Surface_t *surface;
      Point<double> *max_pos;       // Max position across all surfaces.
      bool configured;              // Have the physics been configured yet?
      Random *random_ptr;
      double max_radius;

      Signal<> on_update_sig;

      //////////////////////////////////////////////////////////////////////////////////////////////
      //                             Internal functions.
      //////////////////////////////////////////////////////////////////////////////////////////////
      void UpdateSurfaceBodies() {
        auto & surface_shapes = surface->GetShapeSet();
        int cur_size = (int) surface_shapes.size();
        int cur_id = 0;
        double f = surface->GetFriction();
        while (cur_id < cur_size) {
          emp_assert(surface_shapes[cur_id] != nullptr && surface_shapes[cur_id].HasOwner());
          // If this shape no longer has an owner (a body), delete it.
          if (!surface_shapes[cur_id]->HasOwner()) {
            delete surface_shapes[cur_id];
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
            continue;
          // If the Owner of the shape's destruction has been flagged, remove the shape.
          } else if (surface_shapes[cur_id]->GetOwnerPtr()->GetDestroyFlag()) {
            surface_shapes[cur_id]->GetOwnerPtr()->DetachShape();
            delete surface_shapes[cur_id];
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
            continue;
          // Everything is okay, go ahead and update body.
          } else {
            surface_shapes[cur_id]->GetOwnerPtr()->BodyUpdate(f, 0.25); // TODO: get rid of magic number for change rate.
            if (surface_shapes[cur_id]->GetRadius() > max_radius)
              max_radius = surface_shapes[cur_id]->GetRadius();
            cur_id++;
          }
        }
        surface_shapes.resize(cur_size);
      }

      void TestCollisions() {
        const int num_cols = std::min<int>(GetWidth() / (max_radius * 2.0), 32);
        const int num_rows = std::min<int>(GetHeight() / (max_radius * 2.0), 32);
        const int max_col = num_cols - 1;
        const int max_row = num_rows - 1;
        const int num_sectors = num_cols * num_rows;
        // Calculate sector size.
        const double sector_width = GetWidth() / (double) num_cols;
        const double sector_height = GetHeight() / (double) num_rows;
        vector<vector<Body<Shape_t> *>> sector_set(num_sectors);

        auto & surface_shapes = surface->GetShapeSet();
        for (auto *shape : surface_shapes) {
          emp_assert(shape != nullptr);
          // Determine which sector the current body is in.
          const int cur_col = to_range<int>(shape->GetCenter().GetX()/sector_width, 0, max_col);
          const int cur_row = to_range<int>(shape->GetCenter().GetY()/sector_height, 0, max_row);
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

      bool CollideBodies(Body<Shape_t> * body1, Body<Shape_t> * body2) {
        // If bodies are linked, no collision.
        if (body1->IsLinked(*body2)) return false;
        // Phase one collision detection: Body-touching math.
        Point<double> dist = body1->GetShapePtr()->GetCenter() - body2->GetShapePtr()->GetCenter();
        double sq_pair_dist = dist.SquareMagnitude();
        const double radius_sum = body1->GetShapePtr()->GetRadius() + body2->GetShapePtr()->GetRadius();
        const double sq_min_dist = radius_sum * radius_sum;
        // No touching, no collision.
        if (sq_pair_dist >= sq_min_dist) return false;
        // Collision!
        body1->TriggerCollision(body2); // Give bodies the opportunity to respond to the collision.
        body2->TriggerCollision(body1);
        // Perhaps there is a funciton registered to handle this.
        if (body1->IsColliding() || body2->IsColliding()) {
          body_owner_tt.RunFunction(body1->GetTrackedOwnerPtr(), body2->GetTrackedOwnerPtr());
        }
        // All else fails to resolve, fallback to default behavior.
        if (body1->IsColliding() || body2->IsColliding()) {
          this->collision_resolution_fun(body1, body2);
        }
        return true;
      }

      void FinalizeSurfaceBodies() {
        auto & surface_shapes = surface->GetShapeSet();
        int cur_size = (int) surface_shapes.size();
        int cur_id = 0;
        while (cur_id < cur_size) {
          emp_assert(surface_shapes[cur_id] != nullptr && surface_shapes[cur_id].HasOwner());
          // If shape's body owner has been marked for destruction or exceeds its stress threshold,
          // detach shape from body and delete the shape.
          if (surface_shapes[cur_id]->GetOwnerPtr()->GetDestroyFlag() ||
              surface_shapes[cur_id]->GetOwnerPtr()->ExceedsStressThreshold()) {
            surface_shapes[cur_id]->GetOwnerPtr()->MarkForDestruction(); // In case this wasn't already noted.
            surface_shapes[cur_id]->GetOwnerPtr()->DetachShape();
            delete surface_shapes[cur_id];                // Delete the shape.
            cur_size--;
            surface_shapes[cur_id] = surface_shapes[cur_size];
            continue;
          } else {
            surface_shapes[cur_id]->GetOwnerPtr()->FinalizePosition(*max_pos);
            cur_id++;
          }
        }
        surface_shapes.resize(cur_size);
      }

      // Default collision resolution (called if none of the signals result in collision resolution)]
      // Simple bounce.
      void DefaultCollisionResolution(Body<Shape_t> * body1, Body<Shape_t> * body2) {
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

    public:
      CirclePhysics2D()
        : configured(false)
      {
        emp_assert(sizeof...(BODY_OWNERS) > 0);
      }

      CirclePhysics2D(double width, double height, Random *r, double surface_friction) {
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
      Surface_t * GetSurfacePtr() { return surface; }
      Surface_t & GetSurface() { return *surface; }
      const Surface_t & GetConstSurface() const { return *surface; }

      // emp::vector<Surface*> & GetSurfaceSet() { return surface_set; }
      // const emp::vector<Surface*> & GetConstSurfaceSet() const { return surface_set; }

      CirclePhysics2D & Clear() {
        surface->Clear();
        return *this;
      }

      // Config needs to be able to be called multiple times..
      // Configure physics. This must be called if default constructor was
      // used when creating this.
      void ConfigPhysics(double width, double height, Random *r, double surface_friction) {
        if (configured) { delete surface; }
        collision_resolution_fun = [this](Body<Shape_t> * body1, Body<Shape_t> * body2)
                                   { this->DefaultCollisionResolution(body1, body2); };
        surface = new Surface_t(width, height, surface_friction);
        max_pos = new Point<double>(width, height);
        random_ptr = r;
        configured = true;
      }

      // Callback registration.
      // TODO: Ensure that this interface is consistent with the rest of Empirical.
      void RegisterOnUpdateCallback(std::function<void()> callback) {
        on_update_sig.AddAction(callback);
      }
      template<typename T1, typename T2>
      void RegisterCollisionHandler( std::function<void(T1*, T2*)> fun) {
        emp_assert(GetTypeID<T1>() >= 0 && GetTypeID<T2>() >= 0);
        body_owner_tt.AddFunction(fun);
        if (GetTypeID<T1>() != GetTypeID<T2>()) {
          std::function <void(T2*, T1*)> f2 =
            [fun](T2 *t2, T1 *t1) { fun(t1, t2); };
          body_owner_tt.AddFunction(f2);
        }
      }
      void SetDefaultCollisionResolutionFun(std::function<void(Body<Shape_t> * body1, Body<Shape_t> * body2)> fun) {
        collision_resolution_fun = fun;
      }

      template <typename OWNER_TYPE>
      CirclePhysics2D & AddBody(OWNER_TYPE * body_owner) {
        emp_assert(configured);
        Body<Shape_t> * body_ptr = body_owner->GetBodyPtr();
        // TODO: double check with Charles that this does not have adverse effects in population manager.
        body_ptr->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner));
        surface->AddShape(body_ptr->GetShapePtr());
        body_ptr->SetPhysicsBodyTypeID(GetTypeID(*body_owner));
        body_ptr->SetShapeResponsibility(false); // Physics will be responsible for shapes.
        return *this;
      }

      template <typename BODY_TYPE>
      CirclePhysics2D & RemoveBody(BODY_TYPE * body_owner) {
        emp_assert(configured);
        Body<Shape_t> * body_ptr = body_owner->GetBodyPtr();
        body_ptr->DetachTrackedOwner();
        surface->RemoveShape(body_ptr->GetShapePtr());
        return *this;
      }

      // Progress physics by a single time step.
      void Update() {
        emp_assert(configured);
        on_update_sig.Trigger();
        // Reset max_radius.
        max_radius = 0.0;
        // Update all bodies. Remove those marked for removal. Will also find the max radius.
        UpdateSurfaceBodies();
        // Test for collisions.
        //  - If nothing exists, no need to test for collisions.
        if (max_radius > 0.0) TestCollisions();
        // Finalize positions and test for stress-induced removal.
        FinalizeSurfaceBodies();
      }
  };

}

#endif
