#ifndef CERES_SLAM_TEXTURE_H_
#define CERES_SLAM_TEXTURE_H_

#include <memory>

namespace ceres_slam {

//! Diffuse texture
template <typename Scalar>
class Texture {
   public:
    //! Pointer type
    typedef std::shared_ptr<Texture> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const Texture> ConstPtr;
    //! Colour type
    typedef Scalar Colour;

    //! Default constructor
    Texture() : Texture(0) {}
    //! Construct from Phong parameter matrix
    Texture(Colour col) : col_(col) {}

    //! Return the texture colour (mutable)
    inline Colour& col() { return col_; }
    //! Return the texture colour (const)
    inline const Colour& col() const { return col_; }

    //! Return a pointer to the underlying data
    inline Colour* data() { return &col_; }

   private:
    Colour col_;
};

}  // namespace ceres_slam

#endif /* end of include guard: CERES_SLAM_TEXTURE_H_ */
