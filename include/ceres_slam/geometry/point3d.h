#ifndef CERES_SLAM_GEOMETRY_POINT3D_H_
#define CERES_SLAM_GEOMETRY_POINT3D_H_

#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {
    template <typename _Scalar, int _Options = 0> class Point3D;
} // namespace ceres_slam

///////////////////////////////////////////////////////////////////////////////
// Inherit CTRP traits from the base class so we can use Eigen::Map directly
///////////////////////////////////////////////////////////////////////////////
namespace Eigen { namespace internal {

template <typename _Scalar, int _Options>
struct traits<ceres_slam::Point3D<_Scalar,_Options> >
        : traits<Eigen::Matrix<_Scalar, 3, 1>> {
    typedef _Scalar Scalar;
};

template <typename _Scalar, int _Options>
struct traits<Map<ceres_slam::Point3D<_Scalar>,_Options> >
        : traits<Map<Eigen::Matrix<_Scalar, 3, 1>,_Options> > {
    typedef _Scalar Scalar;
};

template <typename _Scalar, int _Options>
struct traits<Map<const ceres_slam::Point3D<_Scalar,_Options> > >
        : traits<Map<const Eigen::Matrix<_Scalar, 3, 1>,_Options> > {
    typedef _Scalar Scalar;
};

} } // namespace Eigen::internal


///////////////////////////////////////////////////////////////////////////////
// Implementation
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {

//! Point in 3D space
template <typename _Scalar, int _Options>
class Point3D : public Eigen::Matrix<_Scalar, 3, 1> {
    //! Base class definition
    typedef Eigen::Matrix<_Scalar, 3, 1> Base;

public:
    //! Scalar type
    typedef typename Eigen::internal::traits<Point3D>::Scalar Scalar;

    //! Default constructor
    Point3D() : Base() { }

    //! Constructor to construct Point3D from Eigen expressions
    template <typename OtherDerived>
    Point3D(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) { }

    //! Assignment of Eigen expressions to Point3D
    using Base::operator=;

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for Point3D
    friend std::ostream& operator<<( std::ostream& os,
                                     const Point3D<Scalar>& p ) {
        os << "Point3D(" << p.str() << ")";
        return os;
    }
};

} // namespace ceres_slam


namespace Eigen {
//! Specialization of Eigen::Map for Point3D
template <typename _Scalar, int _Options>
class Map<ceres_slam::Point3D<_Scalar>,_Options>
    : public Map<Eigen::Matrix<_Scalar, 3, 1>,_Options> {
    //! Base class definition
    typedef Map<Eigen::Matrix<_Scalar, 3, 1>,_Options> Base;

public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;

    //! Pass through to base class map constructor
    Map(Scalar* data) : Base(data) { };

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(ceres_slam::CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator
    friend std::ostream& operator<<( std::ostream& os,
                                 const Map<ceres_slam::Point3D<Scalar> >& p ) {
        os << "Point3D(" << p.str() << ")";
        return os;
    }
};

//! Specialization of Eigen::Map for const Point3D
template <typename _Scalar, int _Options>
class Map<const ceres_slam::Point3D<_Scalar>,_Options>
    : public Map<const Eigen::Matrix<_Scalar, 3, 1>,_Options> {
    //! Base class definition
    typedef Map<const Eigen::Matrix<_Scalar, 3, 1>,_Options> Base;

public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;

    //! Pass through to base class map constructor
    Map(Scalar* data) : Base(data) { };

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(ceres_slam::CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator
    friend std::ostream& operator<<( std::ostream& os,
                                const Map<const ceres_slam::Point3D<Scalar> >& p ) {
        os << "Point3D(" << p.str() << ")";
        return os;
    }
};

} // namespace Eigen

#endif // CERES_SLAM_GEOMETRY_POINT3D_H_
