#ifndef CERES_SLAM_GEOMETRY_VECTOR3D_H_
#define CERES_SLAM_GEOMETRY_VECTOR3D_H_

#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {
template <typename _Scalar, int _Options = 0>
class Vector3D;
}  // namespace ceres_slam

///////////////////////////////////////////////////////////////////////////////
// Inherit CTRP traits from the base class so we can use Eigen::Map directly
///////////////////////////////////////////////////////////////////////////////
namespace Eigen {
namespace internal {

template <typename _Scalar, int _Options>
struct traits<ceres_slam::Vector3D<_Scalar, _Options>>
    : traits<Eigen::Matrix<_Scalar, 3, 1>> {
    typedef _Scalar Scalar;
};

template <typename _Scalar, int _Options>
struct traits<Map<ceres_slam::Vector3D<_Scalar>, _Options>>
    : traits<Map<Eigen::Matrix<_Scalar, 3, 1>, _Options>> {
    typedef _Scalar Scalar;
};

template <typename _Scalar, int _Options>
struct traits<Map<const ceres_slam::Vector3D<_Scalar, _Options>>>
    : traits<Map<const Eigen::Matrix<_Scalar, 3, 1>, _Options>> {
    typedef _Scalar Scalar;
};
}
}  // namespace Eigen::internal

///////////////////////////////////////////////////////////////////////////////
// Implementation
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {

//! Vector in 3D space
template <typename _Scalar, int _Options>
class Vector3D : public Eigen::Matrix<_Scalar, 3, 1> {
    //! Base class definition
    typedef Eigen::Matrix<_Scalar, 3, 1> Base;

   public:
    //! Scalar type
    typedef typename Eigen::internal::traits<Vector3D>::Scalar Scalar;

    //! Dimension of vector
    static const int dim = 3;
    //! Variance type
    typedef Eigen::Matrix<Scalar, dim, 1> Variance;
    //! Covariance matrix type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor> Covariance;

    //! Default constructor
    Vector3D() : Base() {}

    //! Constructor to construct Vector3D from Eigen expressions
    template <typename OtherDerived>
    Vector3D(const Eigen::MatrixBase<OtherDerived>& other)
        : Base(other) {}

    //! Assignment of Eigen expressions to Vector3D
    using Base::operator=;

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for Vector3D
    friend std::ostream& operator<<(std::ostream& os,
                                    const Vector3D<Scalar>& v) {
        os << "Vector3D(" << v.str() << ")";
        return os;
    }
};

}  // namespace ceres_slam

namespace Eigen {
//! Specialization of Eigen::Map for Vector3D
template <typename _Scalar, int _Options>
class Map<ceres_slam::Vector3D<_Scalar>, _Options>
    : public Map<Eigen::Matrix<_Scalar, 3, 1>, _Options> {
    //! Base class definition
    typedef Map<Eigen::Matrix<_Scalar, 3, 1>, _Options> Base;

   public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;

    //! Dimension of vector
    static const int dim = 3;
    //! Variance type
    typedef Eigen::Matrix<Scalar, dim, 1> Variance;
    //! Covariance matrix type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor> Covariance;

    //! Pass through to base class map constructor
    Map(Scalar* data) : Base(data){};

    //! Assignment of Eigen expressions to Point3D
    using Base::operator=;

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(ceres_slam::CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator
    friend std::ostream& operator<<(
        std::ostream& os, const Map<ceres_slam::Vector3D<Scalar>>& v) {
        os << "Vector3D(" << v.str() << ")";
        return os;
    }
};

//! Specialization of Eigen::Map for const Vector3D
template <typename _Scalar, int _Options>
class Map<const ceres_slam::Vector3D<_Scalar>, _Options>
    : public Map<const Eigen::Matrix<_Scalar, 3, 1>, _Options> {
    //! Base class definition
    typedef Map<const Eigen::Matrix<_Scalar, 3, 1>, _Options> Base;

   public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;

    //! Dimension of vector
    static const int dim = 3;
    //! Variance type
    typedef Eigen::Matrix<Scalar, dim, 1> Variance;
    //! Covariance matrix type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor> Covariance;

    //! Pass through to base class map constructor
    Map(const Scalar* data) : Base(data){};

    //! Assignment of Eigen expressions to Point3D
    using Base::operator=;

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->format(ceres_slam::CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator
    friend std::ostream& operator<<(
        std::ostream& os, const Map<const ceres_slam::Vector3D<Scalar>>& v) {
        os << "Vector3D(" << v.str() << ")";
        return os;
    }
};

}  // namespace Eigen

#endif  // CERES_SLAM_GEOMETRY_VECTOR3D_H_
