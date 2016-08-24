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

// TODO: access to color ID of shape

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

    // Information about other bodies that this body is linked to.
    emp::vector<BodyLink*> from_links;  // Active links initiated by body.
    emp::vector<BodyLink*> to_links;    // Active links targeting body.

    void RemoveFromLink(int link_id) {
      emp_assert(link_id >= 0 && link_id < (int) from_links.size());
      from_links[link_id] = from_links.back();
      from_links.pop_back();
    }
    void RemoveToLink(int link_id) {
      emp_assert(link_id >= 0 && link_id < (int) to_links.size());
      to_links[link_id] = to_links.back();
      to_links.pop_back();
    }

    Body2D_Base() : mass(1.0), inv_mass(1 / mass), pressure(0.0), max_pressure(1.0) { ; }

  public:
    virtual ~Body2D_Base() {
      std::cout << "Body2D_Base::~Body2D_Base" << std::endl;
      // Remove any remaining links from this body.
      while (from_links.size()) RemoveLink(from_links[0]);
      while (to_links.size()) RemoveLink(to_links[0]);
    }

    virtual const Point<double> & GetVelocity() { return velocity; }
    virtual double GetMass() const { return mass; }
    virtual double GetInvMass() const { return inv_mass; }
    virtual double GetPressure() const { return pressure; }
    virtual double GetMaxPressure() const { return max_pressure; }

    virtual void SetVelocity(double x, double y) { velocity.Set(x, y); }
    virtual void SetVelocity(const Point<double> & v) { velocity = v; }
    virtual void SetMass(double m) { mass = m; mass == 0.0 ? inv_mass = 0 : inv_mass = 1.0 / mass; }
    virtual void SetPressure(double p) { pressure = p; }
    virtual void SetMaxPressure(double mp) { max_pressure = mp; }

    virtual void IncSpeed(const Point<double> & offset) { velocity += offset; }

    // Shift to be applied.
    virtual void AddShift(const Point<double> & s) { shift += s; total_abs_shift += s.Abs(); }

    // Creating, testing, and unlinking other organisms.
    virtual bool IsLinkedFrom(const Body2D_Base & link_body) const {
      for (auto * cur_link : from_links) if (cur_link->to == &link_body) return true;
      return false;
    }
    virtual bool IsLinkedTo(const Body2D_Base & link_body) const { return link_body.IsLinkedFrom(*this); }
    virtual bool IsLinked(const Body2D_Base & link_body) const {
      return IsLinkedFrom(link_body) || IsLinkedTo(link_body);
    }
    virtual int GetLinkCount() const { return (int) (from_links.size() + to_links.size()); }

    // Add link FROM this TO link_body.
    virtual void AddLink(BODY_LINK_TYPE type, Body2D_Base & link_body, double cur_dist, double target_dist, double link_strength = 0) {
      emp_assert(!IsLinked(link_body));  // Don't link twice!

      // Build connections in both directions.
      auto * new_link = new BodyLink(type, this, &link_body, cur_dist, target_dist, link_strength);
      from_links.push_back(new_link);
      link_body.to_links.push_back(new_link);
    }

    virtual void RemoveLink(BodyLink * link) {
      if (link->to == this) {
        link->from->RemoveLink(link);
        return;
      }
      // Remove the FROM link.
      for (int i = 0; i < (int) from_links.size(); i++) {
        if (from_links[i]->to == link->to) { RemoveFromLink(i); break; }
      }
      // Remove the TO link.
      const int to_size = (int) link->to->to_links.size();
      for (int i = 0; i < to_size; i++) {
        if (link->to->to_links[i]->from == this) { link->to->RemoveToLink(i); break; }
      }
      delete link;
    }

    virtual const BodyLink & FindLink(const Body2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      for (auto * link : from_links) if ( link->to == &link_body) return *link;
      return link_body.FindLink(*this);
    }

    virtual BodyLink & FindLink(Body2D_Base & link_body)  {
      emp_assert(IsLinked(link_body));
      for (auto * link : from_links) if ( link->to == &link_body) return *link;
      return link_body.FindLink(*this);
    }

    virtual emp::vector<BodyLink *> GetLinksToByType(BODY_LINK_TYPE link_type) {
      emp::vector<BodyLink *> links;
      for (auto *link : this->to_links) {
        if (link->type == link_type) links.push_back(link);
      }
      return links;
    }

    virtual emp::vector<BodyLink *> GetLinksFromByType(BODY_LINK_TYPE link_type) {
      emp::vector<BodyLink *> links;
      for (auto *link : this->from_links) {
        if (link->type == link_type) links.push_back(link);
      }
      return links;
    }

    virtual double GetLinkDist(const Body2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      return FindLink(link_body).cur_dist;
    }
    virtual double GetTargetLinkDist(const Body2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      return FindLink(link_body).target_dist;
    }
    virtual void ShiftLinkDist(Body2D_Base & link_body, double change) {
      auto & link = FindLink(link_body);
      link.cur_dist += change;
    }

  };

  // Example: a body could be in the shape of a circle and be owned by a resource.
  template<typename SHAPE_TYPE, typename OWNER_TYPE = std::nullptr_t>
  class Body : public Body2D_Base {
  protected:
    using Shape_t = OwnedShape<SHAPE_TYPE, Body<SHAPE_TYPE, OWNER_TYPE>>;
    Shape_t * shape_ptr; // circle, rectangle, etc.
    OWNER_TYPE * owner_ptr; // organism, resource, etc.
    bool has_owner;



  public:
    template <typename... ARGS>
    Body(ARGS... args) :
      owner_ptr(nullptr),
      has_owner(false)
    {
      shape_ptr = new Shape_t(this, std::forward<ARGS>(args)...);
    }

    template <typename... ARGS>
    Body(OWNER_TYPE * o_ptr, ARGS... args) :
      owner_ptr(o_ptr),
      has_owner(true)
    {
      shape_ptr = new Shape_t(this, std::forward<ARGS>(args)...);
    }

    ~Body() {

    }

    Shape_t * GetShapePtr() { return shape_ptr; }
    Shape_t & GetShape() { return *shape_ptr; }
    const Shape_t & GetConstShape() const { return *shape_ptr; }
    OWNER_TYPE * GetBodyOwnerPtr() { return owner_ptr; }
    OWNER_TYPE & GetBodyOwner() { return owner_ptr; }
    const OWNER_TYPE & GetConstBodyOwner() { return owner_ptr; }

    // TODO: configure body function?
    // TODO: expose useful shape functions
    const Angle & GetOrientation() const { return shape_ptr->GetOrientation(); }
    const uint32_t GetColorID() const { return shape_ptr->GetColorID(); }

    void SetColorID(uint32_t in_id) { shape_ptr->SetColorID(in_id); }

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
