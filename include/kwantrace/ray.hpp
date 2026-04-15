//
// Created by chrisj on 4/15/26.
//

#ifndef KWANTRACE26_RAY_HPP
#define KWANTRACE26_RAY_HPP

namespace kwantrace {
  /** A mathematical ray, starting at an initial point \f$\vec{r}_0\f$ and continuing in direction
   * \f$\vec{v}\f$. This can be used as a vector function of a single parameter \f$\vec{r}(t)=\vec{r}_0+\vec{v}t\f$.
   *
   * The direction vector does not have to be unit length, but if it is unit length, then the \f$t\f$
   * parameter just represents the distance along the ray.
   *
   * Note that it doesn't make sense to have a zero direction vector. Such a ray would never leave the initial point,
   * no matter what the parameter was. This class will work perfectly fine, but most likely
   * all intersect functions will be asked to divide by zero at some point.
   *
   * This class is mostly a container for the vector coefficients, which are directly used by the intersection
   * routines in kwantrace::Primitive.intersect() . Those routines are responsible for determining if a given
   * ray actually hits anything.
   *
   * The ray does support a couple of operators, to handle evaluating the ray at a given parameter,
   * transforming with a matrix, and advancing the ray (generating a new ray which starts at a given parameter
   * from the old ray).
   */
  class Ray {
  public:
    Position r0; ///< Ray initial point
    Direction v; ///< Ray direction

    /** Construct an array from the given initial position and direction
     *
     * @param Lr0 Initial point
     * @param Lv direction
     */
    constexpr Ray(Position Lr0, Direction Lv) noexcept : r0(Lr0), v(Lv) {};
    /** Construct a ray
     *
     * @param x0 initial x coordinate
     * @param y0 initial y coordinate
     * @param z0 initial z coordinate
     * @param vx x direction
     * @param vy y direction
     * @param vz z direction
     */
    constexpr Ray(double x0, double y0, double z0, double vx, double vy, double vz) noexcept : r0(Position(x0, y0, z0)),
                                                                            v(Direction(vx, vy, vz)) {}

    /** Construct a ray with a zero initial position and *nonzero* velocity \f$\hat{x}\f$.
     */
    Ray() : r0(Position(0, 0, 0)), v(Direction(1, 0, 0)) {}
    /** Transform this ray with a matrix. The position and direction vectors have to be handled differently,
     * since the initial point is a position which participates in translation, while the direction does not.
     * This is handled by the differently-overloaded multiplication operator for each kind of vector.
     *
     * @param M Matrix
     * @return Reference to this ray
     *
     * Note that the following two expressions are equivalent:
     *   r*=Mwb;
     *   r=Mwb*r;
     */
    Ray& operator*=(const Eigen::Matrix4d &M) noexcept {
      r0 = M*r0; //Transform the initial point such that this vector *is* subject to translation
      v = M*v; //Transform the direction such that this vector *is not* subject to translation
      return *this;
    }
    /** Advance this ray a certain amount
     *
     * @param dt Amount to advance the ray
     * @return Ray has its initial point advanced, so ray(t)==oldray(t+dt)
     */
    Ray& operator+=(const double dt) noexcept {
      r0+=v*dt;
      return *this;
    }
    //! Evaluate the ray
    /**
     *
     * @param t Parameter to evaluate the ray at
     * @return Point on ray at given parameter
     */
    Position operator()(double t) noexcept {
      return static_cast<Eigen::Vector3d>(r0 + v * t);
    }

    /** Transform a ray with a matrix. Note that only left-multiplication is
     * supported -- right-multiplication shouldn't even compile.
     *
     * @param M Matrix to transform with
     * @param ray Ray to transform
     * @return A copy of the ray which has been transformed by the given matrix
     */
    friend Ray operator*(const Eigen::Matrix4d &M, Ray ray) {
      ray *= M;
      return ray;
    }
    /** Advance a ray by a given amount
     *
     * @param ray Ray to transform
     * @param dt amount to advance the parameter
     * @return A copy of the ray with the parameter advanced.
     *
     * Given `Ray rp=r+4.7;` the expression `rp(t)==r(t+4.7)` will be true (except for
     * limited floating point precision)
     */
    friend Ray operator+(Ray ray, double dt) {
      ray += dt;
      return ray;
    }
    /** Advance a ray by a given amount
     *
     * @param ray Ray to transform
     * @param dt amount to advance the parameter
     * @return A copy of the ray with the parameter advanced.
     *
     * Given `Ray rp=r+4.7;` the expression `rp(t)==r(t+4.7)` will be true (except for
     * limited floating point precision)
     */
    friend Ray operator+(double dt, Ray ray) {
      ray += dt;
      return ray;
    }

  };

}
#endif //KWANTRACE26_RAY_HPP