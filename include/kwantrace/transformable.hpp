/* KwanTrace - C++ Ray Tracing Library
Copyright (C) 2021 by kwan3217

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef KWANTRACE_TRANSFORMABLE_HPP
#define KWANTRACE_TRANSFORMABLE_HPP


namespace kwantrace {
  ///! List of pointers to transforms
  using TransformList=std::vector<std::unique_ptr<Transformation>>;
  /** Entity that can be transformed. In general, we use the POV-Ray model where each transformation
   * is thought of as *physically moving* the Transformable. For instance, if we start with a Transformable
   * that is located at the origin and do a translate(1,2,3), the object will then be located at x=1, y=2, z=3.
   *
   * Also like POV-Ray, we treat all transformations as being about the origin, not about the center of the
   * given object which might not be at the origin any more. For instance:
   *
   *    * If an object is already 5 units from the origin and you call scale(3,3,3)
   *         * the object will then be 15 units from the origin.
   *    * If an object is at <5,0,0> and pointing down the x axis, and you call rotateZ(90)
   *         * it will be pointing down the Y axis, but also at <0,5,0>.
   *
   * With a little bit of thought, we can see what kind of *frame* transformation corresponds to a
   * *physical* move. Let's look at a translation by \f$\vec{r}\f$. In the body frame, we look at
   * the origin. In the world frame, that same point has coordinates of \f$\vec{r}\f$
   *
   * This is designed to be efficient, with as much effort done at scene construction and prepareRender() as
   * possible, to save as much time effort during the render. This makes sense, because the render will be
   * called literally millions of times. You may chain literally any number of transformations, and only pay
   * the cost at prepareRender(). During the render, the cost of 0, 1, or 1000 transformations are all the same.
   */
  class Transformable {
  private:
    /** Combine transformations in the transformation list. In terms of physical transformations, it is as if
     * the the transforms in the list are performed in order.
     *
     * \internal In actuality, the transforms are converted to
     * matrices, and then combined by matrix multiplication with the transformations in order from the right.
     * This is the traditional way to combine matrices, and is required if you are then going to use M*v
     * to transform a column vector.
     * @return Matrix representing the combination of all transformations performed in order.
     */
    Eigen::Matrix4d combine() const {
      Eigen::Matrix4d result{Eigen::Matrix4d::Identity()};
      for (auto&& trans:transformList) {
        result = trans->matrix() * result;
      }
      return result;
    }
    /** List of pointers to physical transformations to be performed, in order. The transformations themselves
     * can be changed through their pointer, but prepareRender must be called to actually apply the transformation
     */
    TransformList transformList;
  public:
    Eigen::Matrix4d Mwb; ///< World-from-body transformation matrix, only valid between a call to prepareRender and any changes to any transforms in the list
    Eigen::Matrix4d Mbw; ///< Body-from-world transformation matrix, only valid between a call to prepareRender and any changes to any transforms in the list
    Eigen::Matrix4d MwbN;///< World-from-body transformation matrix for surface normals, only valid between a call to prepareRender and any changes to any transforms in the list
    virtual ~Transformable()=default; ///< Allow there to be subclasses
    /** Prepare for rendering
     *
     * \internal This is done by calling combine() to combine all of the transformations, and
     *    then computing ancillary matrices Mwb, Mbw, and MwbN, which will also be needed.
     */
    virtual void prepareRender() {
      Mwb = combine();
      Mbw = Mwb.inverse();
      MwbN = Mbw.transpose();
    }

    /** Construct a new transformation and add it to the end of the chain
     *
     * @param[in] transform A transformation
     * @return a reference to this transformation. The transformation may be modified through
     *   this reference, but prepareRender() must be called in order to make the changes active.
     *   Normally, Scene::render() does this so user code doesn't have to.
     */
    template<typename T, typename... Args>
       requires std::derived_from<T,Transformation>
    T& transform(Args&&... args) {
      auto ptr=std::make_unique<T>(std::forward<Args>(args)...);
      T& ref=*ptr;
      transformList.push_back(std::move(ptr));
      return ref;
    }
  };
}

#endif //KWANTRACE_TRANSFORMABLE_HPP
