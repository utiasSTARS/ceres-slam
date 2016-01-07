#ifndef CERES_SLAM_GEOMETRY_SO3GROUP_H_
#define CERES_SLAM_GEOMETRY_SO3GROUP_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres_slam/geometry/point3d.h>
#include <ceres_slam/geometry/vector3d.h>

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {
    template<typename _Scalar, int _Options = 0> class SO3Group;
}

///////////////////////////////////////////////////////////////////////////////
// Eigen traits for querying derived types in CTRP hierarchy
///////////////////////////////////////////////////////////////////////////////
namespace Eigen { namespace internal {

template<typename _Scalar, int _Options>
struct traits<ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Matrix<Scalar, 3, 3, RowMajor> MatrixType;
};

template<typename _Scalar, int _Options>
struct traits<Map<ceres_slam::SO3Group<_Scalar, _Options> > >
        : traits<ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<Matrix<Scalar, 3, 3, RowMajor>, _Options> MatrixType;
};

template<typename _Scalar, int _Options>
struct traits<Map<const ceres_slam::SO3Group<_Scalar, _Options> > >
        : traits<const ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<const Matrix<Scalar, 3, 3, RowMajor>, _Options> MatrixType;
};

} }

///////////////////////////////////////////////////////////////////////////////
// Implementation
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {

//! SO3Group base class (storage agnostic)
template <typename Derived>
class SO3GroupBase {
public:
    //! Scalar type
    typedef typename Eigen::internal::traits<Derived>::Scalar Scalar;
    //! Transformation matrix type
    typedef typename Eigen::internal::traits<Derived>::MatrixType
        TransformationMatrix;

    //! Degrees of freedom (3 for rotation)
    static const int dof = 3;
    //! Dimension of transformation matrix
    static const int dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof, Eigen::RowMajor> AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef Eigen::Matrix<Scalar, dim, dof, Eigen::RowMajor>
        PerturbationJacobian;

    //! Return a reference to the underlying matrix
    inline
    TransformationMatrix& matrix() {
        return static_cast<Derived*>(this)->matrix();
    };
    //! Return a const reference to the underlying matrix
    inline
    const TransformationMatrix& matrix() const {
        return static_cast<const Derived*>(this)->matrix();
    };

    //! Assignment operator
    template<typename OtherDerived> inline
    SO3GroupBase<Derived>& operator=(const SO3GroupBase<OtherDerived>& other) {
        this->matrix() = other.matrix();
        return *this;
    }

    //! Return the inverse transformation
    inline const SO3Group<Scalar> inverse() const {
        return SO3Group<Scalar>(this->matrix().transpose());
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->matrix().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for SO(3)
    friend std::ostream& operator<<( std::ostream& os, const Derived& d ) {
        os << "SO(3) Rotation Matrix" << std::endl << d.matrix();
        return os;
    }
};

//! SO3Group default type
template<typename _Scalar, int _Options>
class SO3Group : public SO3GroupBase<SO3Group<_Scalar,_Options> > {
    //! Base class definition
    typedef SO3GroupBase<SO3Group<_Scalar,_Options> > Base;

public:
    //! Scalar type
    typedef typename Eigen::internal::traits<SO3Group>::Scalar Scalar;
    //! Transformation matrix type
    typedef typename Eigen::internal::traits<SO3Group>::MatrixType
        TransformationMatrix;

    //! Degrees of freedom (3 for rotation)
    static const int dof = Base::dof;
    //! Dimension of transformation matrix
    static const int dim = Base::dim;
    //! Point type
    typedef typename Base::Point Point;
    //! Vector type
    typedef typename Base::Vector Vector;
    //! Tangent vector type
    typedef typename Base::TangentVector TangentVector;
    //! Adjoint transformation type
    typedef typename Base::AdjointMatrix AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef typename Base::PerturbationJacobian PerturbationJacobian;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Default constructor (identity)
    inline
    SO3Group() { matrix_.setIdentity(); };

    //! Copy constructor
    template<typename OtherDerived> inline
    SO3Group(const SO3GroupBase<OtherDerived>& other)
            : matrix_(other.matrix) { }

    //! Construct from rotation matrix
    SO3Group(const TransformationMatrix& C) : matrix_(C) { }

    //! Return a reference to the underlying matrix
    inline
    TransformationMatrix& matrix() { return matrix_; };
    //! Return a const reference to the underlying matrix
    inline
    const TransformationMatrix& matrix() const { return matrix_; };

private:
    //! Internal storage
    TransformationMatrix matrix_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_SO3GROUP_H_
