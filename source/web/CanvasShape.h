/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2015-2017
 *
 *  @file  CanvasShape.h
 *  @brief Define simple shapes to draw on a canvas.
 *
 *  Canvas shapes can be definied in detail, describing how they modify a canvas.
 *
 *  Other, more specific actions defined here are:
 *    CanvasCircle
 *    CanvasRect
 */


#ifndef EMP_WEB_CANVAS_SHAPE_H
#define EMP_WEB_CANVAS_SHAPE_H

#include <string>

#include "../base/vector.h"
#include "../geometry/Circle2D.h"

#include "CanvasAction.h"

namespace emp {
namespace web {

  /// Define an arbitrary shape to draw on a canvas (base clase)
  class CanvasShape : public CanvasAction {
  protected:
    double x; double y;      ///< Anchor point for this shape.
    std::string fill_color;  ///< Internal color to fill shape with.
    std::string line_color;  ///< Border color for shape.

  public:
    CanvasShape(double _x, double _y, const std::string & fc="", const std::string & lc="")
      : x(_x), y(_y), fill_color(fc), line_color(lc) { ; }
    virtual ~CanvasShape() { ; }

    /// Shift the position of this shape.
    void MoveTo(double _x, double _y) { x=_x; y=_y; }

    /// Change the fill color of this shape.
    void SetFillColor(const std::string & color) { fill_color = color; }

    /// Change the stroke color of this shape.
    void SetLineColor(const std::string & color) { line_color = color; }

    /// Actually change the color on screen.
    void ApplyColor() {
      Fill(fill_color);
      Stroke(line_color);
    }
  };

  /// Track a circle shape to be drawn on a canvas.
  class CanvasCircle : public CanvasShape {
    double radius;  ///< Circle radius
  public:
    CanvasCircle(double _x, double _y, double _r,
                 const std::string & fc="", const std::string & lc="")
      : CanvasShape(_x, _y, fc, lc), radius(_r) { ; }

    CanvasCircle(emp::Circle circle, const std::string & fc="", const std::string & lc="")
      : CanvasShape(circle.GetCenterX(), circle.GetCenterY(), fc, lc)
      , radius(circle.GetRadius()) { ; }

    void Apply() {
      EM_ASM_ARGS({
          emp_i.ctx.beginPath();
          emp_i.ctx.arc($0, $1, $2, 0, Math.PI*2);
        }, x, y, radius);  // Draw the circle
        ApplyColor();
    }
    CanvasAction * Clone() const { return new CanvasCircle(*this); }
  };

  /// Track a rectangle shape to be drawn on a canvas.
  class CanvasRect : public CanvasShape {
    double w;  ///< Rectangle widgth.
    double h;  ///< Rectangle height.
  public:
    CanvasRect(double _x, double _y, double _w, double _h,
               const std::string & fc="", const std::string & lc="")
      : CanvasShape(_x, _y, fc, lc), w(_w), h(_h) { ; }

    void Apply() {
      EM_ASM_ARGS({
          emp_i.ctx.beginPath();
          emp_i.ctx.rect($0, $1, $2, $3);
        }, x, y, w, h);  // Draw the rectangle
      ApplyColor();
    }
    CanvasAction * Clone() const { return new CanvasRect(*this); }
  };

  /// Clear a rectangular area in a canvas.
  class CanvasClearRect : public CanvasShape {
    double w;  ///< Rectangle widgth.
    double h;  ///< Rectangle height.
  public:
    CanvasClearRect(double _x, double _y, double _w, double _h)
      : CanvasShape(_x, _y), w(_w), h(_h) { ; }

    void Apply() {
      EM_ASM_ARGS({
          emp_i.ctx.clearRect($0, $1, $2, $3);
        }, x, y, w, h);  // Draw the rectangle
    }
    CanvasAction * Clone() const { return new CanvasClearRect(*this); }
  };

  /// An arbitrary-sized polygon to be drawn on a canvas.
  class CanvasPolygon : public CanvasShape {
  private:
    emp::vector<Point> points;  ///< Series of points defining the perimiter of the Polygon.
  public:
    CanvasPolygon(const std::string & fc="", const std::string & lc="")
      : CanvasShape(0, 0, fc, lc) { ; }
    CanvasPolygon(const emp::vector<Point> & p, const std::string & fc="", const std::string & lc="")
      : CanvasShape(0, 0, fc, lc), points(p) { ; }
    CanvasPolygon(double _x, double _y, const std::string & fc="", const std::string & lc="")
      : CanvasShape(_x, _y, fc, lc) { ; }

    CanvasPolygon & AddPoint(double x, double y) { points.emplace_back(x,y); return *this; }
    CanvasPolygon & AddPoint(Point p) { points.emplace_back(p); return *this; }

    void Apply() {
      EM_ASM_ARGS({
        emp_i.ctx.translate($0,$1);
        emp_i.ctx.beginPath();
        emp_i.ctx.moveTo($2, $3);
      }, x, y, points[0].GetX(), points[0].GetY());  // Setup the polygon
      for (size_t i = 1; i < points.size(); i++) {
        EM_ASM_ARGS({
          emp_i.ctx.lineTo($0, $1);
        }, points[i].GetX(), points[i].GetY());  // Draw the lines for the polygon
      }
      EM_ASM_ARGS({
        emp_i.ctx.closePath();
        emp_i.ctx.translate($0,$1);
      }, -x, -y);  // Close the polygon
      ApplyColor();
    }
    CanvasAction * Clone() const { return new CanvasPolygon(*this); }
  };

  /// A line segment on the canvas.
  class CanvasLine : public CanvasShape {
  private:
    double x2;  /// X-position for second point of line segment.
    double y2;  /// Y-position for second point of line segment.
  public:
    CanvasLine(double _x1, double _y1, double _x2, double _y2,
               const std::string & lc="")
      : CanvasShape(_x1, _y1, "", lc), x2(_x2), y2(_y2) { ; }

    void Apply() {
      EM_ASM_ARGS({
        emp_i.ctx.beginPath();
        emp_i.ctx.moveTo($0, $1);
        emp_i.ctx.lineTo($2, $3);
        emp_i.ctx.closePath();
      }, x, y, x2, y2);
      // ApplyColor();
      Stroke(line_color);
    }
    CanvasAction * Clone() const { return new CanvasLine(*this); }
  };

  /// Text to be written on a canvas.
  class CanvasText : public CanvasShape {
  protected:
    std::string text;  ///< Specific text to be written.
    bool center;       ///< Should this text be centered (or is anchor on left side)?
  public:
    CanvasText(double x, double y, const std::string & _text,
               const std::string & fc="", const std::string & lc="")
      : CanvasShape(x, y, fc, lc), text(_text), center(false) { ; }

    void Apply() {
      if (center) {
        EM_ASM({ emp_i.ctx.textAlign = "center"; });
        EM_ASM({ emp_i.ctx.textBaseline = "middle"; });
      }
      EM_ASM_ARGS({
        emp_i.ctx.fillStyle = Pointer_stringify($3);
        var text = Pointer_stringify($2);
        emp_i.ctx.fillText(text,$0,$1);
      }, x, y, text.c_str(), fill_color.c_str());
    }

    /// Center this text.
    void Center(bool c=true) { center = c; }

    /// Identify if text is centered.
    bool GetCenter() const { return center; }

    CanvasAction * Clone() const { return new CanvasText(*this); }
  };

}
}

#endif
