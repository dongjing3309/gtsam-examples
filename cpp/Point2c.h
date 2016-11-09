/**
 * @file Point2c.h
 * @brief a custom 2D point class, use traits make it optimizable in GTSAM
 * @date Nov 8, 2016
 * @author Jing Dong
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>

#include <cmath>
#include <iostream>


namespace gtsamexamples {

// A minimal 2D point class, 'c' meas custom
struct Point2c {
  double x;
  double y;
  
  // convenience constructor
  Point2c(double xi, double yi) : x(xi), y(yi) {}
};

} // namespace gtsamexamples


/** 
 * traits for Point2c
 *
 * Any type compatible with GTSAM mush have (in its traits):
 *   - Print function with an optional begining string, in format:
 *     static void Print(const T& m, const std::string& str = "");
 *   - Equal function with optional tolerance, in format:
 *     static bool Equals(const T& m1, const T& m2, double tol = 1e-8);
 *
 * A manifold type must have: 
 *   - int dimension (not fully support dynamic dimensionality yet)
 *   - Typedefs TangentVector, where TangentVector = Eigen::Matrix<double, dimension, 1>
 *   - Local coordinate function, in format:
 *     static TangentVector Local(const Class& origin, const Class& other);
 *   - Retraction back to manifold, in format:
 *     static Class Retract(const Class& origin, const TangentVector& v);
 *
 * A lie group types must have:
 *   - Identity function
 *   - Logmap function, with optional jacobians
 *   - Expmap function, with optional jacobians
 *   - Compose function, with optional jacobians
 *   - Between function, with optional jacobians
 *   - Inverse function, with optional jacobians
 *
 * For lie group types, other than traits, operator * or (+ and -) should be defined
 * for compose / between operation. Can be defined inside or outside class.
 * In this example we defined operator * outside class in the end
 */


// traits must in namespace gtsam
namespace gtsam {

template<>
struct traits<gtsamexamples::Point2c> {

  // strcutural category: this is a lie group
  // avaible options: manifold_tag, group_tag, lie_group_tag
  typedef lie_group_tag structure_category;

  /**
   * Basic (Testable)
   */
   
  // print
  static void Print(const gtsamexamples::Point2c& m, const std::string& str = "") {
    std::cout << str << "(" << m.x << ", " << m.y << ")" << std::endl;
  }
  
  // equality with optional tol
  static bool Equals(const gtsamexamples::Point2c& m1, const gtsamexamples::Point2c& m2, 
      double tol = 1e-8) {
    if (fabs(m1.x - m2.x) < tol && fabs(m1.y - m2.y) < tol)
      return true;
    else
      return false;
  }

  /**
   * Manifold
   */

  // use enum dimension
  enum { dimension = 2 };
  static int GetDimension(const gtsamexamples::Point2c&) { return dimension; }
  
  // Typedefs needed
  typedef gtsamexamples::Point2c ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  
  // Local coordinate of Point2c is naive (since vectorspace)
  static TangentVector Local(const gtsamexamples::Point2c& origin, 
      const gtsamexamples::Point2c& other) {
    return Vector2(other.x - origin.x, other.y - origin.y);
  }
  
  // Retraction back to manifold of Point2c is naive (since vectorspace)
  static gtsamexamples::Point2c Retract(const gtsamexamples::Point2c& origin, 
      const TangentVector& v) {
    return gtsamexamples::Point2c(origin.x + v(0), origin.y + v(1));
  }

  /**
   * Lie group
   */
  
  // indicate this group using operator *, 
  // if uses +/- then use option additive_group_tag 
  typedef multiplicative_group_tag group_flavor;
   
  // typedefs
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  
  static gtsamexamples::Point2c Identity() { 
    return gtsamexamples::Point2c(0, 0);
  }
  
  static TangentVector Logmap(const gtsamexamples::Point2c& m, 
      ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Matrix2::Identity(); 
    return Vector2(m.x, m.y);
  }

  static gtsamexamples::Point2c Expmap(const TangentVector& v, 
      ChartJacobian Hv = boost::none) {
    if (Hv) *Hv = Matrix2::Identity(); 
    return gtsamexamples::Point2c(v(0), v(1));
  }

  static gtsamexamples::Point2c Compose(const gtsamexamples::Point2c& m1, 
      const gtsamexamples::Point2c& m2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Matrix2::Identity();
    if (H2) *H2 = Matrix2::Identity(); 
    return gtsamexamples::Point2c(m1.x + m2.x, m1.y + m2.y);
  }

  static gtsamexamples::Point2c Between(const gtsamexamples::Point2c& m1, 
      const gtsamexamples::Point2c& m2, //
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Matrix2::Identity();
    if (H2) *H2 = Matrix2::Identity(); 
    return gtsamexamples::Point2c(m2.x - m1.x, m2.y - m1.y);
  }

  static gtsamexamples::Point2c Inverse(const gtsamexamples::Point2c& m, //
      ChartJacobian H = boost::none) {
    if (H) *H = -Matrix2::Identity(); 
    return gtsamexamples::Point2c(-m.x, -m.y);
  }
};

} // namespace gtsam


namespace gtsamexamples {

  // operator *
  Point2c operator*(const Point2c& m1, const Point2c& m2) {
    return Point2c(m1.x + m2.x, m1.y + m2.y);
  }
}





