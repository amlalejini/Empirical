//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  Class to manage shapes in a 2D plane. Shapes may have owners (useful for say,
//  physics) or may not have owners (when one simply wants to draw a circle).
//
//  All shapes have a bounding circle that is guaranteed to contain the entirety
//  of the shape.

#ifndef EMP_SHAPE_2D_H
#define EMP_SHAPE_2D_H

#include "Point2D.h"
#include "Angle2D.h"

namespace emp {
  // Every shape should define a center and a radius from that center that
  // encompasses the entirety of the shape.
  class Shape {
  protected:
    Point<double> center;
    double radius;
    Angle orientation;
    uint32_t color_id;

    Shape(const Point<double> & _c, double _r=0) : center(_c), radius(_r) { ; }
    Shape(double _x, double _y, double _r=0) : center(_x,_y), radius(_r) { ; }
    Shape(double _r=0) : center(0.0, 0.0), radius(_r) { ; }

  public:
    virtual ~Shape() { ; }

    virtual const Point<double> & GetCenter() const { return center; }
    virtual double GetCenterX() const { return center.GetX(); }
    virtual double GetCenterY() const { return center.GetY(); }
    virtual double GetRadius() const { return radius; }
    virtual double GetSquareRadius() const { return radius * radius; }
    virtual const Angle & GetOrientation() const { return orientation; }

    virtual Shape & SetCenter(const Point<double> & new_center) { center = new_center; return *this; }
    virtual Shape & SetCenterX(double new_x) { center.SetX(new_x); return *this; }
    virtual Shape & SetCenterY(double new_y) { center.SetY(new_y); return *this; }
    virtual Shape & SetRadius(double new_radius) { radius = new_radius; return *this; }
    virtual Shape & SetOrientation(Angle new_orientation) { orientation = new_orientation; return *this; }
    virtual Shape & Translate(Point<double> shift) { center += shift; return *this; }

    uint32_t GetColorID() const { return color_id; }

    void SetColorID(uint32_t in_id) { color_id = in_id; }

  };

  class Circle  : public Shape {
  protected:

  public:
    Circle(const Point<double> & _c, double _r=0) : Shape(_c, _r) { ; }
    Circle(double _x, double _y, double _r=0) : Shape(_x, _y, _r) { ; }
    Circle(double _r=0) : Shape(_r) { ; }

    Circle & SetCenter(const Point<double> & new_center) override { center = new_center; return *this; }
    Circle & SetCenterX(double new_x) override { center.SetX(new_x); return *this; }
    Circle & SetCenterY(double new_y) override { center.SetY(new_y); return *this; }
    Circle & SetRadius(double new_radius) override { radius = new_radius; return *this; }
    Circle & SetOrientation(Angle new_orientation) override { orientation = new_orientation; return *this; }
    Circle & Translate(Point<double> shift) override { center += shift; return *this; }

    bool Contains(const Point<double> & point) const {
      return center.SquareDistance(point) < GetSquareRadius();
    }
    bool Contains(const Circle & other) const {
      const double max_dist = other.center.Distance(center) + other.GetRadius();
      return max_dist < GetRadius();
    }
    bool HasOverlap(const Circle & other) const {
      const double min_dist = radius + other.radius;
      return center.SquareDistance(other.center) < (min_dist * min_dist);
    }
  };

}

#endif
