//
// Created by chrisj on 4/14/26.
//

//Eigen library, used in many places
#ifndef MATH_HPP
#define MATH_HPP
#include <eigen/Eigen/Dense> //for matrices and vectors

namespace kwantrace {
  const double pi=3.1415926535897932; ///< Circle constant

  inline double deg2rad(double deg) { return deg * pi / 180.0; } ///<Convert degrees to radians @param deg measure in degrees @return same measure in radians
  inline double rad2deg(double rad) { return rad * 180.0 / pi; } ///<Convert radians to degrees @param rad measure in radians @return same measure in degrees

  //Use these sparingly -- prefer radians as native angle unit when possible,
  //but prefer these over ad-hoc radian/degree conversion in the code. As usual,
  //you the user use functions, and let the compiler deal with inlining.
  //These are mostly included to make scene definition language more POV-Ray like.
  inline double sind(double angle) {return std::sin(deg2rad(angle));}        ///< Degree-mode sine @param angle angle in degrees @return sine of angle
  inline double cosd(double angle) {return std::cos(deg2rad(angle));}        ///< Degree-mode cosine @param angle angle in degrees @return cosine of angle
  inline double tand(double angle) {return std::tan(deg2rad(angle));}        ///< Degree-mode tangent @param angle angle in degrees @return tangent of angle
  inline double asind(double arg) {return rad2deg(std::asin(arg));}          ///< Degree-mode inverse sine @param arg sine of angle @return angle in degrees
  inline double acosd(double arg) {return rad2deg(std::acos(arg));}          ///< Degree-mode inverse cosine @param arg cosine of angle @return angle in degrees
  inline double atand(double arg) {return rad2deg(std::atan(arg));}          ///< Degree-mode inverse tangent @param arg tangent of angle @return angle in degrees
  inline double atan2d(double y, double x) {return rad2deg(std::atan2(y,x));}///< Degree-mode quadrant inverse tangent @param y numerator of tangent of angle @param x denominator of tangent of angle @return angle in degrees in correct quadrant from -180&deg; to +180&deg;

  using RayColor    = Eigen::Vector3d;              ///< Vector representing the color of a ray -- IE the color which will be painted on the pixel buffer
  using ObjectColor = Eigen::Matrix<double, 5, 1>;  ///< Vector representing the intrinsic color of the object. Five components to match POV-Ray's filter and transmit.

}
#endif // MATH_HPP
