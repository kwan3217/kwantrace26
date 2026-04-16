//
// Created by jeppesen on 2/4/21.
//

#ifndef KWANTRACE_FIELD_HPP
#define KWANTRACE_FIELD_HPP

namespace kwantrace {
  /** A field -- a function which takes a vector and returns a vector
   *
   * @tparam N Number of components of the vector result
   * @tparam T Type of the vector components of the result
   */
  template<int N, typename T=double>
  class Field:public Transformable {
  private:
    typedef Eigen::Matrix<T,N,1> OutVector; ///< Alias for the out type
    /** Calculate the value of the field at a point
     * @param r Position to evaluate the field at, in local space
     * @return Value of the field at this point
     */
    virtual OutVector fieldLocal(const Position& r) const =0;
  public:
    /** Evaluate the function at a point in world space
     *
     * @param r  Position to evaluate the field at, in local space
     * @return value of the field at this point
     */
    OutVector operator()(const Position& r) const {return fieldLocal(Mbw * r);};
    virtual ~Field()=default; ///< Allow subclassing

    /** Evaluate the function at a point in world space
     *
     * @param x X coordinate in world space
     * @param y Y coordinate in world space
     * @param z Z coordinate in world space
     * @return value of the field at this point
     */
    OutVector operator()(double x, double y, double z) {return *this(Position(x, y, z));};
  };
  /** Typedef Alias */
  typedef Field<5,double> ColorField;
  /** Constant color field -- has constant color everywhere in space */
  class ConstantColor: public ColorField {
  private:
    ObjectColor value; ///< Value of field
    /** \copydoc Field::fieldLocal()
     *
     * This just returns a constant color no matter which point in space we evaluate
     */
    ObjectColor fieldLocal(const Position& r) const override {return value;}
  public:
    /** Construct a constant color field
     * @param Lvalue Constant color
     */
    ConstantColor(ObjectColor Lvalue):value(Lvalue) {};
    /** Construct a constant color field
     *
     * @param r red component of color
     * @param g green component of color
     * @param b blue component of color
     * @param f filter component of color
     * @param t transmit component of color
     */
    ConstantColor(double r=0, double g=0, double b=0, double f=0, double t=0) {value<< r,g,b,f,t;};
  };
}

#endif //KWANTRACE_FIELD_HPP
