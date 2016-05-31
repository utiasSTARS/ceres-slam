#ifndef CERES_SLAM_GEOMETRY_VERTEX3D_H_
#define CERES_SLAM_GEOMETRY_VERTEX3D_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils/utils.h>
#include <ceres_slam/geometry/geometry.h>

#include "material.h"
#include "texture.h"

namespace ceres_slam {

//! Vertex object for use with shading
template <typename Scalar>
class Vertex3D {
   public:
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Material pointer type
    typedef typename Material<Scalar>::Ptr MaterialPtr;
    //! Texture type
    typedef typename Texture<Scalar>::Ptr TexturePtr;
    //! Vertex dimension
    static const int dim = Point::dim + Vector::dim + Material<Scalar>::dim;

    //! Default constructor
    Vertex3D()
        : position_(Point()),
          normal_(Vector()),
          material_ptr_(nullptr),
          texture_ptr_(nullptr) {}
    //! Construct from position, normal, and phong parameters
    Vertex3D(const Point& position, const Vector& normal,
             const MaterialPtr material, const TexturePtr texture)
        : position_(position),
          normal_(normal),
          material_ptr_(material),
          texture_ptr_(texture) {}

    //! Return the position of the vertex (mutable)
    inline Point& position() { return position_; }
    //! Return the position of the vertex (const)
    inline const Point& position() const { return position_; }

    //! Return the normal vector at the vertex (mutable)
    inline Vector& normal() { return normal_; }
    //! Return the normal vector at the vertex (const)
    inline const Vector& normal() const { return normal_; }

    //! Return a pointer to the material at the vertex (mutable)
    inline MaterialPtr& material() { return material_ptr_; }
    //! Return a pointer to the material at the vertex (const)
    inline const MaterialPtr& material() const { return material_ptr_; }

    //! Return the diffuse texture at the vertex (mutable)
    inline TexturePtr& texture() { return texture_ptr_; }
    //! Return the diffuse texture at the vertex (const)
    inline const TexturePtr& texture() const { return texture_ptr_; }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->position().str() << "," << this->normal().str() << ","
           << this->material()->phong_params().format(CommaInitFmt) << ","
           << this->texture()->col();
        return ss.str();
    }

    //! Ostream operator for vertices
    friend std::ostream& operator<<(std::ostream& os,
                                    const Vertex3D<Scalar>& v) {
        os << "Vertex" << std::endl
           << "Position: " << v.position() << std::endl
           << "Normal: " << v.normal() << std::endl
           << "Material: " << *(v.material()) << std::endl
           << "Texture: " << v.texture()->col();
        return os;
    }

   private:
    //! Vertex position
    Point position_;
    //! Vertex surface normal
    Vector normal_;
    //! Vertex material
    MaterialPtr material_ptr_;
    //! Vertex texture
    TexturePtr texture_ptr_;
};

}  // namespace ceres_slam

#endif  // CERES_SLAM_GEOMETRY_VERTEX3D_H_
