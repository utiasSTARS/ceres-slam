#ifndef CERES_SLAM_GEOMETRY_POINT3D_H_
#define CERES_SLAM_GEOMETRY_POINT3D_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry/homogeneous3d.h>

namespace ceres_slam {

//! Point in 3D space
template <typename Scalar>
class Point3D : public Homogeneous3D<Scalar> {
public:
    //! Pointer type
    typedef std::shared_ptr<Point3D> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const Point3D> ConstPtr;
    //! Cartesian type
    typedef typename Homogeneous3D<Scalar>::Cartesian Cartesian;
    //! Homogeneous type
    typedef typename Homogeneous3D<Scalar>::Homogeneous Homogeneous;

    //! Default constructor
    Point3D() : Point3D(Cartesian::Zero()) { }
    //! Copy constructor
    Point3D( const Point3D& other ) : Point3D(other.cartesian()) { }
    //! Construct from a 3-vector
    Point3D( const Cartesian& cartesian ) :
        Homogeneous3D<Scalar>(cartesian, Scalar(1)) { }
    //! Construct from 3 scalars
    Point3D( const Scalar x, const Scalar y, const Scalar z ) :
        Homogeneous3D<Scalar>(x, y, z, Scalar(1)) { }
    //! Construct from a 3-element POD array
    Point3D( const Scalar s[3] ) :
        Point3D(s[0], s[1], s[2]) { }
    //! Conversion constructor from a general homogeneous quantity
    Point3D( const Homogeneous3D<Scalar>& h ) :
        Point3D(h.cartesian()) {
        // if( abs<Scalar>(h.scale()) <= std::numeric_limits<Scalar>::epsilon() ) {
        //     std::cerr << "Warning: implicit cast from Homogeneous3D to Point3D"
        //               << " with scale = " << h.scale() << " == 0" << std::endl;
        // }
    }

    //! Ostream operator for Point3D
    friend std::ostream& operator<<( std::ostream& os,
                                     const Point3D<Scalar>& p ) {
        os << "Point3D("
           << p.homogeneous().format(CommaInitFmt)
           << ")";
        return os;
    }
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_POINT3D_H_
