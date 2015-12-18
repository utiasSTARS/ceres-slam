#ifndef CERES_SLAM_GEOMETRY_VECTOR3D_H_
#define CERES_SLAM_GEOMETRY_VECTOR3D_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry/homogeneous3d.h>

namespace ceres_slam {

//! Vector in 3D space
template <typename Scalar>
class Vector3D : public Homogeneous3D<Scalar> {
public:
    //! Pointer type
    typedef std::shared_ptr<Vector3D> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const Vector3D> ConstPtr;
    //! Cartesian type
    typedef typename Homogeneous3D<Scalar>::Cartesian Cartesian;
    //! Homogeneous type
    typedef typename Homogeneous3D<Scalar>::Homogeneous Homogeneous;

    //! Default constructor
    Vector3D() : Vector3D(Cartesian::Zero()) { }
    //! Copy constructor
    Vector3D( const Vector3D& other ) : Vector3D(other.cartesian()) { }
    //! Construct from a 3-vector
    Vector3D( const Cartesian& cartesian ) :
        Homogeneous3D<Scalar>(cartesian, Scalar(0)) { }
    //! Construct from 3 scalars
    Vector3D( const Scalar i, const Scalar j, const Scalar k ) :
        Homogeneous3D<Scalar>(i, j, k, Scalar(0)) { }
    //! Construct from a 3-element POD array
    Vector3D( const Scalar s[3] ) :
        Vector3D(s[0], s[1], s[2]) { }
    //! Conversion constructor from a general homogeneous quantity
    Vector3D( const Homogeneous3D<Scalar>& h ) :
        Vector3D(h.cartesian()) {
        if( abs<Scalar>(h.scale())
            >= Scalar(std::numeric_limits<Scalar>::epsilon()) ) {
            std::cerr << "Warning: implicit cast from Homogeneous3D to Vector3D"
                      << " with scale = " << h.scale() << " != 0" << std::endl;
        }
    }

    //! Compute the squared norm of the vector
    inline const Scalar squaredNorm() const {
        return this->cartesian().squaredNorm();
    }
    //! Compute the norm of the vector
    inline const Scalar norm() const { return this->cartesian().norm(); }
    //! Normalize the vector
    inline void normalize() { this->cartesian_.normalize(); }
    //! Compute the dot product of two vectors
    inline const Scalar dot( const Vector3D& other ) const {
        return this->cartesian().dot(other.cartesian());
    }

    //! Ostream operator for Vector3D
    friend std::ostream& operator<<( std::ostream& os,
                                     const Vector3D<Scalar>& v ) {
        os << "Vector3D("
           << v.homogeneous().format(CommaInitFmt)
           << ")";
        return os;
    }
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_VECTOR3D_H_
