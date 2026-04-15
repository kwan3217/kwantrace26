//
// Created by chrisj on 4/14/26.
//

export module kwantrace:transformation;

import :pdvector;
#include <eigen3/Eigen/Dense> //for matrices and vectors

export namespace kwantrace {
    /** Represent a generator of an arbitrary affine transformation. It can have any members it needs,
 * but must be able to take its members and generate a Matrix4d on demand. Members are intended
 * to be changed (IE properties).
 *
 * I really didn't want to make this one -- I hoped to be able to use the Eigen transformations
 * directly, but things like translate, rotate, scale etc don't have a common base class, so they
 * can't be grouped in a common container.
 *
 * Besides, if I do it this way, I have a good spot for my thesis on the Point-Toward transformation.
 *
 * I have to call this "Transformation" instead of "Transformer" because otherwise I will be
 * thinking about robots in disguise...
 */
  class Transformation {
  public:
    /** Construct the matrix for this transformation. This can
     * read any of the parameters in the class, but is declared
     * const and therefore can't write anything. This is so that
     * it can be called during a render when there might be multiple
     * threads and no synchronization.
     * @return Matrix representing the transformation
     */
    virtual Eigen::Matrix4d matrix() const = 0;
    /** Prepare the transformation for render. At this point, the
     *  properties of the transformation are set for the frame, but
     *  haven't been used yet. This is the time to set up caches, etc.
     *  It's not const so that it *can* write caches etc.
     */
    virtual void prepareRender() {};
    virtual ~Transformation()=default; ///<Allow there to be subclasses
  };

  /** Transformation with a scalar parameter */
  class ScalarTransformation:public Transformation {
  private:
    double amount; ///< Parameter value.
  public:
    /** Construct a new transformation with a scalar parameter.
     *
     * @param Lamount Initial parameter value
     */
    ScalarTransformation(double Lamount=0):amount(Lamount) {};
  };

  /** Transformation with a vector parameter. */
  class VectorTransformation:public Transformation {
  private:
    Eigen::Vector3d amount; ///< Vector parameter for transform
  public:
    /**
     * Initialize a vector transformation.
     * @param Lamount Initial value for vector transform parameter
     */
    VectorTransformation(const Eigen::Vector3d Lamount=Eigen::Vector3d::Zero()):amount(Lamount) {};
    /**
     * Initialize a vector transformation.
     * @param x Initial X component
     * @param y Initial Y component
     * @param z Initial Z component
     */
    VectorTransformation(double x, double y, double z):amount(x,y,z) {};
  };

  /** Represent a translation. The vector represents the coordinates of origin of the body frame, in the world frame. */
  class Translation:public VectorTransformation {
  public:
    using VectorTransformation::VectorTransformation;
    virtual Eigen::Matrix4d matrix() const override {
      Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
      result(0,3)=amount.x();
      result(1,3)=amount.y();
      result(2,3)=amount.z();
      return result;
    }
  };

  /** Represent a non-uniform scaling, IE one that can be different along the three body axes.
   * The vector represents the stretch factor in each direction. Since Bad Things happen if
   * you specify a scale of zero (matrices are no longer full-rank and therefore no longer
   * invertible) we adopt the POV-Ray convention and interpret requests to scale as zero to
   * be requests to scale as 1. In this code, we do the substitution silently, unlike in
   * POV-Ray where you get a warning if you do that.
   *
   */
  class Scaling:public VectorTransformation {
  public:
    using VectorTransformation::VectorTransformation;
    virtual Eigen::Matrix4d matrix() const override {
      Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
      result(0,0)=amount.x()==0?1:amount.x();
      result(1,1)=amount.y()==0?1:amount.y();
      result(2,2)=amount.z()==0?1:amount.z();
      return result;
    }
  };

  /** Represent a uniform scaling in all directions. You could use
   * a vector Scaling, but then you would have to change all three
   * components of the scaling vector to keep the scaling uniform. */
  class UniformScaling:public ScalarTransformation {
  public:
    using ScalarTransformation::ScalarTransformation;
    virtual Eigen::Matrix4d matrix() const override {
      Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
      result(0,0)=amount==0?1:amount;
      result(1,1)=amount==0?1:amount;
      result(2,2)=amount==0?1:amount;
      return result;
    }
  };
  /** Calculate the rotation matrix around a body axis
   *
   * @param axis Axis index, x=0, y=1, z=2
   * @param angle Angle to rotate in radians
   * @return Rotation matrix representing a physical rotation of an object about an axis by the given angle in a right-handed sense.
   */
  Eigen::Matrix4d rot(int axis, double angle) {
    Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
    double c = cos(angle);
    double s = sin(angle);
    result((axis+1)%3, (axis+1)%3)= c; result((axis+1)%3, (axis+2)%3)= -s;
    result((axis+2)%3, (axis+1)%3)= s; result((axis+2)%3, (axis+2)%3)=  c;
    return result;
  }

  /** Represents a right-handed physical rotation around a coordinate frame axis.
   * Right-handed means if you wrap the fingers of your *right* hand around the
   * rotation axis, with your thumb pointed in the positive direction of the axis,
   * your fingers will wrap around the axis in the positive sense.
   *
   * \image html Right-hand_grip_rule.png
   *
   * For instance, if an object is pointed down the x axis and you rotate it +90&deg;
   * around the z axis, the object will then be pointed down the y axis.
   *
   * @tparam axis Axis to rotate around -- x=0, y=1, z=2
   */
  template<int axis>
  class RotateScalar:public ScalarTransformation {
  public:
    using ScalarTransformation::ScalarTransformation;
    /** Construct a rotation, optionally specifying the angle in degrees
     *
     * @param Lamount Angle to rotate
     * @param isDegrees If true, Lamount is specified in degrees. Otherwise it is specified in radians.
     */
    RotateScalar(double Lamount, bool isDegrees=true):ScalarTransformation(isDegrees?deg2rad(Lamount):Lamount) {};
    virtual Eigen::Matrix4d matrix() const override {
      return rot(axis,amount);
    }
    double getd() const {return rad2deg(amount);}        ///< Get the parameter, assuming it is an angle. @return Parameter angle in degrees
    void setd(double Lamount) {amount=deg2rad(Lamount);} ///< Set the parameter, assuming it is an angle. @param[in] Lamount new parameter angle in degrees
  };
  typedef RotateScalar<0> RotateX;  ///< Specialized RotateScalar for X axis
  typedef RotateScalar<1> RotateY;  ///< Specialized RotateScalar for Y axis
  typedef RotateScalar<2> RotateZ;  ///< Specialized RotateScalar for Z axis

  /** Represents a right-handed physical rotation around each coordinate frame axis in turn.
   * This is in a sense an Euler angle rotation, but in a rather inflexible way -- if you
   * want a Euler angle rotation, chain together three RotateScalar objects around the axes
   * you want in the order you want.
   *
   * This represents a rotation around the x axis by the amount specified by the x component
   * of the parameter, followed by a rotation around the y axis as specified by the y
   * component of the parameter, followed by the same for z. The order is not flexible.
   *
   * This emulates a POV-Ray rotate with a vector parameter (except for being right-handed rotations).
   */
  class RotateVector:public VectorTransformation {
  public:
    using VectorTransformation::VectorTransformation;
    /** Construct a RotateVector, optionally specifying the angles in degrees */
    RotateVector(
      double Lx, ///< Amount to rotate around X axis
      double Ly, ///< Amount to rotate around Y axis
      double Lz, ///< Amount to rotate around Z axis
      bool isDegrees=true  ///< True if Lx, Ly, and Lz are specified in degrees
    ):VectorTransformation(
      isDegrees?deg2rad(Lx):Lx,
      isDegrees?deg2rad(Ly):Ly,
      isDegrees?deg2rad(Lz):Lz) {
    };
    /** Construct a RotateVector, optionally specifying the angles in degrees */
    RotateVector(
      const Eigen::Vector3d Lamount, ///< Amount to rotate around each axis
      bool isDegrees                 ///< True if components of Lamount are in degrees
    ):VectorTransformation(
      isDegrees?deg2rad(Lamount.x()):Lamount.x(),
      isDegrees?deg2rad(Lamount.y()):Lamount.y(),
      isDegrees?deg2rad(Lamount.z()):Lamount.z()) {
    };
    virtual Eigen::Matrix4d matrix() const override {
      Eigen::Matrix4d result=rot(0,amount.x());
      result=rot(1,amount.y())*result;
      result=rot(2,amount.z())*result;
      return result;
    }
    double getXd() const {return rad2deg(getX());} ///< Get the X component of the rotation in degrees @return value of X component
    void setXd(double Lx) {setX(deg2rad(Lx));}  ///< Set the X component of the rotation in degrees @param[in] Lx value of X component
    double getYd() const {return rad2deg(getY());} ///< Get the Y component of the rotation in degrees @return value of Y component
    void setYd(double Ly) {setX(deg2rad(Ly));}  ///< Set the Y component of the rotation in degrees @param[in] Ly value of Y component
    double getZd() const {return rad2deg(getZ());} ///< Get the Z component of the rotation in degrees @return value of Z component
    void setZd(double Lz) {setZ(deg2rad(Lz));}  ///< Set the Z component of the rotation in degrees @param[in] Lz value of Z component
    Eigen::Vector3d getVd() const {return Eigen::Vector3d(getXd(),getYd(),getZd());} ///< Get the rotation in degrees @return copy of rotation vector converted to degrees
    void setVd(const Eigen::Vector3d Lamount) {setXd(Lamount.x());setYd(Lamount.y());setZd(Lamount.z());} ///< Set the parameter @param[in] Lamount New value of the parameter in degrees
  };

  /** Represent the Point-Toward transformation. This rotates an object such that
   * p_b in the body frame points at p_r in the world frame, and t_b in the body frame is towards
   * t_r in the world frame.
   *
   *
   */
  class PointToward:public Transformation {
  private:
    Eigen::Vector3d p_b; ///< Point vector in body frame
    Eigen::Vector3d p_r; ///< Point vector in world frame
    Eigen::Vector3d t_b; ///< Toward vector in body frame
    Eigen::Vector3d t_r; ///< Toward vector in world frame
  public:
    /** Construct a Point-Toward transformation */
    PointToward(
            const Eigen::Vector3d &Lp_b, ///< point vector in body frame
            const Eigen::Vector3d &Lp_r, ///< point vector in world frame
            const Eigen::Vector3d &Lt_b, ///< toward vector in body frame
            const Eigen::Vector3d &Lt_r  ///< toward vector in world frame
    ) : p_b(Lp_b), p_r(Lp_r), t_b(Lt_b), t_r(Lt_r) {}
    Eigen::Vector3d getPb() const                     {return p_b;} ///<Get copy of p_b vector @return copy of p_b vector
               void setPb(const Eigen::Vector3d& Lpb) {p_b=Lpb;}    ///<Set p_b vector @param[in] Lpb New p_b vector
    Eigen::Vector3d getPr() const                     {return p_r;} ///<Get copy of p_b vector @return copy of p_r vector
               void setPr(const Eigen::Vector3d& Lpr) {p_b=Lpr;}    ///<Set p_b vector @param[in] Lpr New p_r vector
    Eigen::Vector3d getTb() const                     {return t_b;} ///<Get copy of t_b vector @return copy of t_b vector
               void setTb(const Eigen::Vector3d& Ltb) {p_b=Ltb;}    ///<Set t_b vector @param[in] Ltb New t_b vector
    Eigen::Vector3d getTr() const                     {return t_r;} ///<Get copy of t_r vector @return copy of t_r vector
               void setTr(const Eigen::Vector3d& Ltr) {p_b=Ltr;}    ///<Set t_r vector @param[in] Ltr New t_r vector
    /**
     * Calculate the matrix representing this Point-Toward transformation.
     * @return Matrix representing the point-toward transformation.
     */
    virtual Eigen::Matrix4d matrix() const override {
      return calcPointToward(p_b, p_r, t_b, t_r);
    }
    /** Do the actual work of calculating a point-toward transformation
     *
     * @param p_b primary (point) direction in body frame
     * @param p_r primary (point) direction in reference frame
     * @param t_b secondary (toward) constraint in body frame
     * @param t_r secondary (toward) constraint in reference frame
     * @return Matrix representing the point-toward transformation.
     *
     * ## Problem Statement
     * \f$
     *    \def\M#1{{[\mathbf{#1}]}}
     *    \def\MM#1#2{{[\mathbf{#1}{#2}]}}
     *    \def\T{^\mathsf{T}}
     *    \def\operatorname#1{{\mbox{#1}}}
     * \f$
     * Given a rigid body with vectors from its origin to direction (normalized vector)
     * \f$\hat{p}_b\f$ and \f$\hat{t}_b\f$ in the body frame, and an external frame
     * centered on the same origin with directions \f$\hat{p}_r\f$ and \f$\hat{t}_r\f$,
     * find the physical rotation that points \f$\hat{p}_b\f$ and \f$\hat{p}_r\f$
     * at the same direction, while simultaneously pointing \f$\hat{t}_b\f$ as close as
     * possible to \f$\hat{t}_r\f$.
     *
     * ### Example
     * The Space Shuttle has a thrust vector which is not parallel to any of the body axes.
     * We wish to point the thrust vector in the correct direction in the reference system,
     * while simultaneously flying heads-down, which is equivalent to pointing the tail
     * towards the ground. In this case, \f$\hat{p}_b\f$ is the thrust vector in the body
     * frame, \f$\hat{p}_r\f$ is the guidance-calculated thrust vector in the reference
     * frame, \f$\hat{t}_b\f$ is the body axis which points heads-up, say \f$\hat{z}_b\f$,
     * and \f$\hat{t}_r\f$ is the vector from the spacecraft location towards the center
     * of the Earth.
     *
     * \image html 320px-Point_constraint.svg.png
     *
     * \image html 320px-Toward_constraint.svg.png
     *
     * ## Solution
     * We are going to do this with matrices. The solution matrix is going to be called
     * \f$\MM{M}{_{rb}}\f$ and will transform *to* the reference frame *from* the body frame.
     *
     * First, it is obviously impossible to in general satisfy both the "point" constraint
     * \f$\hat{p}_r=\MM{M}{_{rb}}\hat{p}_b\f$ and the toward constraint \f$\hat{t}_r=\MM{M}{_{rb}}\hat{t}_b\f$.
     * Satisfying both is possible if and only if the angle between \f$\hat{p}_r\f$ and \f$\hat{t}_r\f$
     * is the same as the angle between \f$\hat{p}_b\f$ and \f$\hat{t}_b\f$. When these
     * angles do not match, the point constraint will be perfectly satisfied, and the
     * angle between the body and reference toward vectors will be as small as possible.
     * Using geometric intuition, it is obvious but not proven here that the angle is
     * minimum when the point vector, transformed body toward vector, and reference toward
     * vector are all in the same plane. This means that we can create a third vector
     * \f$\hat{s}=\operatorname{normalize}(\hat{p} \times \hat{t})\f$. This vector is
     * normal to the plane containing point and toward in both frames, so when the plane
     * is the same, these vectors match. Therefore we have another constraint which can
     * be perfectly satisfied, \f$\hat{s}_r=\MM{M}{_{rb}}\hat{s}_b\f$.
     * So, we have:
     *
     * \f$\begin{bmatrix}\hat{p}_r && \hat{s}_r\end{bmatrix}=\MM{M}{_{rb}}\begin{bmatrix}\hat{p}_b && \hat{s}_b\end{bmatrix}\f$
     *
     * This isn't quite enough data, it works out to nine unknowns and six equations.
     * We can add one more constraint by considering the vector \f$\hat{u}\f$ perpendicular
     * to both \f$\hat{p}\f$ and \f$\hat{s}\f$:
     *
     * \f$\hat{u}=\hat{p} \times \hat{s}\f$
     *
     * We already know that this will be unit length since \f$\hat{p}\f$ and \f$\hat{s}\f$
     * are already perpendicular. Since these three vectors are perpendicular in both frames,
     * only an orthogonal matrix can transform all three vectors and maintain the angles
     * between them, so this third vector is equivalent to adding an orthogonality constraint.
     *
     * \f$\M{R}=\begin{bmatrix}\hat{p}_r && \hat{s}_r\ && \hat{u}_r \end{bmatrix}\f$
     *
     * (treating the vectors as column vectors)
     *
     * \f$\begin{eqnarray*}
     *    \M{B}&=&\begin{bmatrix}\hat{p}_b && \hat{s}_b\ && \hat{u}_b \end{bmatrix} \\
     *    \M{R}&=&\MM{M}{_{rb}}\M{B} \\
     *    \M{R}\M{B}^{-1}&=&\MM{M}{_{rb}}\end{eqnarray*}\f$
     *
     *The above calls for a matrix inverse, but who has time for that? Since all the columns of
     * \f$\M{B}\f$ (and \f$\M{R}\f$ for that matter) are unit length and perpendicular to each
     * other, the matrix is orthogonal, which means that its inverse is its transpose.
     *
     * \f$\M{R}\M{B}^T=\MM{M}{_{rb}}\f$
     *
     * And that's the solution. Note that if you need \f$\MM{M}{_{br}}\f$, it is also a transpose
     * since this answer is still an orthonormal (IE rotation) matrix.
     */
    static Eigen::Matrix4d calcPointToward(
      Direction p_b,
      Direction p_r,
      Direction t_b,
      Direction t_r
    ) {
      Eigen::Matrix3d R, B;
      Direction s_r = (p_r.cross(t_r)).normalized();
      Direction u_r = (p_r.cross(s_r)).normalized();
      R << p_r.normalized(), s_r, u_r;
      Direction s_b = (p_b.cross(t_b)).normalized();
      Direction u_b = (p_b.cross(s_b)).normalized();
      B << p_b.normalized(), s_b, u_b;
      Eigen::Matrix4d M_rb = Eigen::Matrix4d::Identity();
      M_rb.block<3, 3>(0, 0) = R * B.transpose();
      return M_rb;
    }
    /** Exercise pointToward().
     * \image html Space_Shuttle_Coordinate_System.jpg
     * \f$
     *    \def\M#1{{[\mathbf{#1}]}}
     *    \def\MM#1#2{{[\mathbf{#1}{#2}]}}
     *    \def\T{^\mathsf{T}}
     *    \def\operatorname#1{{\mbox{#1}}}
     * \f$
     *
     * The space shuttle has a thrust axis 13&deg; below the X axis, so:
     * \f$\hat{p}_b=\begin{bmatrix}\cos 13^\circ \\ 0 \\ -\sin 13^\circ \end{bmatrix}
     *   =\begin{bmatrix}0.974370 \\ 0.000000 \\ -0.224951 \end{bmatrix}\f$
     *
     * The heads-up vector is \f$\hat{t}_b=\hat{z}_b\f$. At a particular instant,
     * the guidance command says to point the thrust vector 30&deg; above the horizon
     * at an azimuth of 80&deg; east of North. We'll take the local topocentric horizon
     * frame as the reference frame, with \f$\hat{x}_r\f$ in the horizon plane pointing
     * east, \f$\hat{y}_r\f$ pointing north, and \f$\hat{z}_r\f$ pointing up. In this
     * frame, the guidance command is:
     *
     * \f$\hat{p}_r=\begin{bmatrix}\cos 30^\circ \sin 80^\circ \\
     *                             \cos 30^\circ \cos 80^\circ \\
     *                             \sin 30^\circ\end{bmatrix}=\begin{bmatrix}0.852869 \\
     *                                                                       0.150384 \\
     *                                                                       0.500000\end{bmatrix}\f$
     *
     * The vehicle is also commanded to the heads-down attitude, which means that
     * \f$\hat{t}_r=-\hat{z}_r\f$. These are all the inputs we need.
     *
     * \f$\hat{s}_b=\operatorname{normalize}(\hat{p}_b \times \hat{t}_b)=\begin{bmatrix} 0 \\
     *                                                                                  -1 \\
     *                                                                                   0 \end{bmatrix}\f$
     *
     * \f$\hat{u}_b=\operatorname{normalize}(\hat{p}_b \times \hat{s}_b)=\begin{bmatrix} -0.224951 \\
     *                                                                                    0.000000 \\
     *                                                                                   -0.974370 \end{bmatrix}\f$
     *\f$\hat{s}_r=\operatorname{normalize}(\hat{p}_r \times \hat{t}_r)=\begin{bmatrix} -0.173648 \\
     *                                                                                   0.984808 \\
     *                                                                                   0.000000 \end{bmatrix}\f$
     *
     *\f$\hat{u}_r=\operatorname{normalize}(\hat{p}_r \times \hat{s}_r)=\begin{bmatrix} -0.492404 \\
     *                                                                                  -0.086824 \\
     *                                                                                  -0.866025 \end{bmatrix}\f$
     *
     * \f$\M{R}=\begin{bmatrix}\hat{p}_r && \hat{s}_r\ && \hat{u}_r \end{bmatrix}=\begin{bmatrix}0.852869&&-0.173648&&-0.492404\\
     *                                                                                   0.150384&& 0.984808&&-0.086824\\
     *                                                                                   0.500000&& 0.000000&& 0.866025\end{bmatrix}\f$
     *
     * \f$\M{B}=\begin{bmatrix}\hat{p}_b && \hat{s}_b\ && \hat{u}_b \end{bmatrix}=\begin{bmatrix}0.974370&& 0.000000&&-0.224951\\
     *                                                                                           0.000000&&-1.000000&&-0.000000\\
     *                                                                                          -0.224951&& 0.000000&&-0.974370\end{bmatrix}\f$
     *\f$\M{M_{br}}=\M{R}\M{B}^{-1}=\begin{bmatrix}0.941776&& 0.173648&& 0.287930\\
     *                                             0.166061&&-0.984808&& 0.050770\\
     *                                             0.292372&& 0.000000&&-0.956305\end{bmatrix}\f$
     *
     * There is the solution, but does it work?
     *
     * \f$\begin{eqnarray*}\M{M_{br}}\hat{p}_b&=&\begin{bmatrix} 0.852869\\ 0.150384\\ 0.500000\end{bmatrix}&=&\hat{p}_r \\
     *                     \M{M_{br}}\hat{s}_b&=&\begin{bmatrix}-0.173648\\ 0.984808\\ 0.000000\end{bmatrix}&=&\hat{s}_r \\
     *                     \M{M_{br}}\hat{u}_b&=&\begin{bmatrix}-0.492404\\-0.086824\\ 0.866025\end{bmatrix}&=&\hat{u}_r \\
     *                     \M{M_{br}}\hat{t}_b&=&\begin{bmatrix} 0.287930\\ 0.050770\\-0.956305\end{bmatrix}, \operatorname{vangle}(\M{M_{br}}\hat{t}_b,\hat{t}_r)=17^\circ\end{eqnarray*}\f$
     *
     * That's a decisive yes.
     */
    static void exercisePointToward() {
      Direction p_b(cosd(13),
                    0,
                    -sind(13));
      std::cout << "p_b:"<< std::endl << p_b << std::endl;
      Direction t_b(0,0,1);
      std::cout << "t_b:"<< std::endl << t_b << std::endl;
      Direction p_r(cosd(30)*sind(80),
                    cosd(30)*cosd(80),
                    sind(30)         );
      std::cout << "p_r:"<< std::endl << p_r << std::endl;
      Direction t_r(0,0,-1);
      std::cout << "t_r:"<< std::endl << t_r << std::endl;
      Direction s_b=p_b.cross(t_b).normalized();
      std::cout << "s_b:"<< std::endl << s_b << std::endl;
      Direction u_b=p_b.cross(s_b).normalized();
      std::cout << "u_b:"<< std::endl << u_b << std::endl;
      Direction s_r=p_r.cross(t_r).normalized();
      std::cout << "s_r:"<< std::endl << s_r << std::endl;
      Direction u_r=p_r.cross(s_r).normalized();
      std::cout << "u_r:"<< std::endl << u_r << std::endl;
      Eigen::Matrix3d R;
      R << p_r,s_r,u_r;
      std::cout << "R:  "<< std::endl << R << std::endl;
      Eigen::Matrix3d B;
      B << p_b,s_b,u_b;
      std::cout << "B:  "<< std::endl << B << std::endl;
      Eigen::Matrix3d M_rb_direct=R*B.transpose();
      std::cout << "M_rb (direct):  "<< std::endl << M_rb_direct << std::endl;
      auto M_rb=calcPointToward(p_b,p_r,t_b,t_r);
      std::cout << "M_rb:  "<< std::endl << M_rb << std::endl;
      std::cout << "M_rb*p_b (should equal p_r):  "<< std::endl << M_rb*p_b << std::endl;
      std::cout << "M_rb*s_b (should equal s_r):  "<< std::endl << M_rb*s_b << std::endl;
      std::cout << "M_rb*u_b (should equal u_r):  "<< std::endl << M_rb*u_b << std::endl;
      std::cout << "M_rb*t_b (should be towards t_r):  "<< std::endl << M_rb*t_b << std::endl;
    }
  };


  /** Represent the Point-Toward transformation. This rotates an object such that
   * p_b in the body frame points at p_r in the world frame, and t_b in the body frame is towards
   * t_r in the world frame.
   */
  class LocationLookat:public Transformation {
  private:
    Position location; ///< Point vector in world frame
    Position look_at; ///< Point vector in world frame
    Direction p_b;      ///< Point vector in body frame
    Direction t_b;      ///< Toward vector in body frame
    Direction t_r;      ///< Toward vector in world frame
  public:
    /** Construct a Location-LookAt transformation
     *
     * @param Llocation Value to copy into location field
     * @param Llook_at Value to copy into look_at field
     * @param Lp_b Value to copy into p_b field
     * @param Lt_b Value to copy into t_b field
     * @param Lt_r Value to copy into t_r field
     */
    LocationLookat(
            const Position &Llocation,
            const Position &Llook_at,
            const Direction& Lp_b=Direction( 0, 0, 1),
            const Direction& Lt_b=Direction( 0, 1, 0),
            const Direction& Lt_r=Direction( 0, 0,-1)
    ):location(Llocation),look_at(Llook_at),p_b(Lp_b), t_b(Lt_b), t_r(Lt_r) {}

    Position getLocation() const                     {return location;} ///<Get copy of p_b vector @return copy of p_b vector
    void setLocation(const Position& Lloc) {location=Lloc;}    ///<Set p_b vector @param[in] Lloc New location vector
    Position getLook_at() const                     {return look_at;} ///<Get copy of t_b vector @return copy of t_b vector
    void setLook_at(const Position& Llook) {look_at=Llook;}    ///<Set t_b vector @param[in] Llook New look vector
    Direction getPb() const                     {return p_b;} ///<Get copy of p_b vector @return copy of p_b vector
    void setPb(const Direction& Lpb) {p_b=Lpb;}    ///<Set p_b vector @param[in] Lpb New p_b vector
    Direction getTb() const                     {return t_b;} ///<Get copy of t_b vector @return copy of t_b vector
    void setTb(const Direction& Ltb) {p_b=Ltb;}    ///<Set t_b vector @param[in] Ltb New t_b vector
    Direction getTr() const                     {return t_r;} ///<Get copy of t_r vector @return copy of t_r vector
    void setTr(const Direction& Ltr) {p_b=Ltr;}    ///<Set t_r vector @param[in] Ltr New t_r vector

    /** Creates a matrix which places an object at location and points it at look_at
     *
     * @param location Location in world frame which body frame origin maps to
     * @param look_at Look-at point in world frame.
     * @param p_b Primary direction in body frame, will be mapped to direction (look_at-location)
     * @param t_b Secondary direction in body frame, will be mapped as close as possible to t_r
     * @param t_r Secondary direction in world frame, referred to as `sky` in POV-Ray. Default value is actually more like
       * `ground` than `sky`.
     * @return Transformation matrix which does the job
     */
    static Eigen::Matrix4d calc(
      const Position& location,
      const Position& look_at,
      const Direction& p_b= Direction(0, 0, 1),
      const Direction& t_b= Direction(0, 1, 0),
      const Direction& t_r = Direction(0, 0, -1)
    ) {
      Eigen::Matrix4d result= PointToward::calc(p_b, Direction(look_at - location), t_b, t_r); //Use point-toward to point at the target
      result=Translation::calc(location)*result; //Translate back to location
      return result;
    }
    Eigen::Matrix4d matrix() const override {
      return calc(location, look_at, p_b, t_b, t_r);
    }
  };

}