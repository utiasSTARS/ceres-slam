#ifndef CERES_SLAM_GEOMETRY_VERTEX3D_H_
#define CERES_SLAM_GEOMETRY_VERTEX3D_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry/point3d.h>
#include <ceres_slam/geometry/vector3d.h>

namespace ceres_slam {

//! Vertex object for use with shading
template <typename Scalar>
class Vertex3D {
public:
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Colour type
    typedef Scalar Colour;
    //! Phong illumination parameters
    typedef Eigen::Matrix<Scalar, 1, 2, Eigen::RowMajor> PhongParams;
    //! Vertex dimension
    static const int dim = Point::dim + Vector::dim + 2;

    //! Default constructor
    Vertex3D() : Vertex3D( Point(), Vector(), PhongParams::Zero() ) { }
    //! Construct from position, normal, and phong parameters
    Vertex3D(Point position, Vector normal, PhongParams phong_params) :
        position_(position),
        normal_(normal),
        phong_params_(phong_params) { }

    //! Return the position of the vertex (mutable)
    inline Point& position() { return position_; }
    //! Return the position of the vertex (const)
    inline const Point& position() const { return position_; }

    //! Return the normal vector at the vertex (mutable)
    inline Vector& normal() { return normal_; }
    //! Return the normal vector at the vertex (const)
    inline const Vector& normal() const { return normal_; }

    //! Return the Phong parameter matrix (mutable)
    inline PhongParams& phong_params() { return phong_params_; }
    //! Return the Phong parameter matrix (const)
    inline const PhongParams& phong_params() const { return phong_params_; }

    //! Return the ambient component of the vertex reflectance (mutable)
    inline Colour& ambient() { return phong_params_(0); }
    //! Return the ambient component of the vertex reflectance (mutable)
    inline const Colour& ambient() const { return phong_params_(0); }

    //! Return the diffuse component of the vertex reflectance (mutable)
    inline Colour& diffuse() { return phong_params_(1); }
    //! Return the diffuse component of the vertex reflectance (const)
    inline const Colour& diffuse() const { return phong_params_(1); }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->position().str() << ","
           << this->normal().str() << ","
           << this->phong_params().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for vertices
    friend std::ostream& operator<<( std::ostream& os,
                                     const Vertex3D<Scalar>& v ) {
        os << "Vertex" << std::endl
           << "Position: " << v.position() << std::endl
           << "Normal: " << v.normal() << std::endl
           << "Phong parameters: " << std::endl
           << v.phong_params() << std::endl;
        return os;
    }
private:
    //! Vertex position
    Point position_;
    //! Vertex surface normal
    Vector normal_;
    //! Phong illumination parameters stored column-wise in a matrix.
    /*!
        col(0) is ambient, col(1) is diffuse,
        col(2) will be specular, col(3) will be shininess.
    */
    PhongParams phong_params_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_VERTEX3D_H_
