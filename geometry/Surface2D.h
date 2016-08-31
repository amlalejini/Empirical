//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//
//  This file defines a templated class to represent a 2D suface capable of maintaining data
//  about which 2D shapes are currently on that surface and rapidly identifying if they are
//  overlapping.
//  TODO: more documentation
//
//  Member functions include:
//
//


#ifndef EMP_SURFACE_2D_H
#define EMP_SURFACE_2D_H

#include "tools/vector.h"
#include "tools/functions.h"
#include "Shape2d.h"
#include <iostream>
#include <algorithm>
#include <functional>

namespace emp {

  class Surface {
  public:
    virtual ~Surface() { ; }

    virtual double GetWidth() const = 0;
    virtual double GetHeight() const = 0;
    virtual void Clear() = 0;

  };

  template <typename SHAPE_TYPE>
  class Surface2D : public Surface {
  private:
    const Point<double> max_pos;        // Lower-left corner of the surface.
    emp::vector<SHAPE_TYPE *> shape_set;  // Set of all bodies on surface
    double friction;  // TODO: should friction be here? Surfaces don't need to know anything about physics..
  public:
    Surface2D(double _width, double _height, double surface_friction = 0.00125)
      : max_pos(_width, _height),
        friction(surface_friction)
    { ; }

    ~Surface2D() {
      Clear();
    }

    double GetWidth() const override { return max_pos.GetX(); }
    double GetHeight() const override { return max_pos.GetY(); }
    double GetFriction() const { return friction; }
    const Point<double> & GetMaxPosition() const { return max_pos; }

    SHAPE_TYPE & operator[](int i) { return shape_set[i]; }

    std::vector<SHAPE_TYPE *> & GetShapeSet() { return shape_set; }
    const std::vector<SHAPE_TYPE *> & GetConstShapeSet() const { return shape_set; }

    void SetFriction(double friction) { this->friction = friction; }

    // Add a single shape.  Surface now controls this shape and must delete it.
    void AddShape(SHAPE_TYPE *shape) {
      shape_set.push_back(shape);     // Add shape to master list
    }

    void RemoveShape(SHAPE_TYPE *shape) {
      shape_set.erase(std::remove_if(shape_set.begin(), shape_set.end(),
                                    [shape](SHAPE_TYPE *shape2){ return shape == shape2; }));
    }

    // Clear all bodies on the surface.
    void Clear() override {
      for (auto * shape : shape_set) {
        delete shape;
      }
      shape_set.resize(0);
    }

  };
};

#endif
