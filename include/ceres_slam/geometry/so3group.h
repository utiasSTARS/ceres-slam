#ifndef CERES_SLAM_GEOMETRY_SO3GROUP_H_
#define CERES_SLAM_GEOMETRY_SO3GROUP_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // For SVD

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry.h>

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {
    template <typename _Scalar, int _Options = 0> class SO3Group;
} // namespace ceres_slam

///////////////////////////////////////////////////////////////////////////////
// Eigen traits for querying derived types in CTRP hierarchy
///////////////////////////////////////////////////////////////////////////////
namespace Eigen { namespace internal {

template <typename _Scalar, int _Options>
struct traits<ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Matrix<Scalar, 3, 3, RowMajor> StorageType;
};

template <typename _Scalar, int _Options>
struct traits<Map<ceres_slam::SO3Group<_Scalar, _Options> > >
        : traits<ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<Matrix<Scalar, 3, 3, RowMajor>, _Options> StorageType;
};

template <typename _Scalar, int _Options>
struct traits<Map<const ceres_slam::SO3Group<_Scalar, _Options> > >
        : traits<const ceres_slam::SO3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<const Matrix<Scalar, 3, 3, RowMajor>, _Options> StorageType;
};

} } // namespace Eigen::internal

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
    //! Internal storage type
    typedef typename Eigen::internal::traits<Derived>::StorageType StorageType;

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
    //! Transformation matrix type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor>
        TransformationMatrix;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof, Eigen::RowMajor> AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef Eigen::Matrix<Scalar, dim, dof, Eigen::RowMajor>
        PerturbationJacobian;

    //! Return a reference to the underlying matrix
    inline
    StorageType& matrix() {
        return static_cast<Derived*>(this)->matrix();
    }
    //! Return a const reference to the underlying matrix
    inline
    const StorageType& matrix() const {
        return static_cast<const Derived*>(this)->matrix();
    }
    //! Return a pointer to the raw matrix data
    inline
    Scalar* data() { return this->matrix().data(); }
    //! Return a const pointer to the raw matrix data
    inline
    const Scalar* data() const { return this->matrix().data(); }

    //! Return a copy of this, casted to OtherScalar
    template <typename OtherScalar>
    inline SO3Group<OtherScalar> cast() const {
        return SO3Group<OtherScalar>(this->matrix().cast<OtherScalar>());
    }

    //! Assignment operator
    template <typename OtherDerived> inline
    SO3GroupBase<Derived>& operator=(const SO3GroupBase<OtherDerived>& other) {
        this->matrix() = other.matrix();
        return *this;
    }

    //! Return the inverse transformation
    inline
    const SO3Group<Scalar> inverse() const {
        return SO3Group<Scalar>(this->matrix().transpose() );
    }

    //! Normalize the underlying matrix to ensure it is a valid rotation
    inline
    void normalize() {
        Eigen::JacobiSVD<TransformationMatrix>
            svd(this->matrix(), Eigen::ComputeThinU | Eigen::ComputeThinV);

        TransformationMatrix middle = TransformationMatrix::Identity();
        middle(2,2) = svd.matrixV().determinant() * svd.matrixU().determinant();

        this->matrix() = svd.matrixU() * middle * svd.matrixV().transpose();
    }

    //! Multiplication operator for two group elements
    inline
    const SO3Group<Scalar> operator*(const SO3Group<Scalar>& other) const {
        SO3Group<Scalar> result(this->matrix() * other.matrix() );
        result.normalize();
        return result;
    }

    //! Transform a 3D point.
    /*!
        Transform a 3D point and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline
    const Point transform(const Point& p,
            PerturbationJacobian* jacobian_ptr = nullptr) const {

        Point p_transformed = this->matrix() * p;

        if(jacobian_ptr != nullptr) {
            PerturbationJacobian& jacobian = *jacobian_ptr;
            jacobian = -wedge(p_transformed);
        }

        return p_transformed;
    }
    //! Transform a 3D point.
    inline
    const Point transform(const Eigen::Map<Point>& p,
            PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Point(p), jacobian_ptr);
    }
    //! Transform a 3D point.
    inline
    const Point transform(const Eigen::Map<const Point>& p,
            PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Point(p), jacobian_ptr);
    }
    //! Multiplication operator for group element and point
    inline
    const Point operator*(const Point& p) const {
        return this->transform(p);
    }
    //! Multiplication operator for group element and point
    inline
    const Point operator*(const Eigen::Map<Point>& p) const {
        return this->transform(p);
    }
    //! Multiplication operator for group element and point
    inline
    const Point operator*(const Eigen::Map<const Point>& p) const {
        return this->transform(p);
    }

    //! Transform a 3D vector.
    /*!
        Transform a 3D vector and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline
    const Vector transform(const Vector& v,
        PerturbationJacobian* jacobian_ptr = nullptr) const {

        Vector v_transformed = this->matrix() * v;

        if(jacobian_ptr != nullptr) {
            PerturbationJacobian& jacobian = *jacobian_ptr;
            jacobian = -wedge(v_transformed);
        }

        return v_transformed;
    }
    //! Transform a 3D vector.
    inline
    const Vector transform(const Eigen::Map<Vector>& v,
            PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Vector(v), jacobian_ptr);
    }
    //! Transform a 3D vector.
    inline
    const Vector transform(const Eigen::Map<const Vector>& v,
            PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Vector(v), jacobian_ptr);
    }
    //! Multiplication operator for group element and vector
    inline
    const Vector operator*(const Vector& v) const {
        return this->transform(v);
    }
    //! Multiplication operator for group element and vector
    inline
    const Vector operator*(const Eigen::Map<Vector>& v) const {
        return this->transform(v);
    }
    //! Multiplication operator for group element and vector
    inline
    const Vector operator*(const Eigen::Map<const Vector>& v) const {
        return this->transform(v);
    }

    //! Return the identity element of SO(3)
    inline static
    const SO3Group<Scalar> Identity() {
        return SO3Group<Scalar>();
    }

    //! SO(3) wedge operator as defined by Barfoot
    /*!
        This is the inverse operator to SO3::vee.
    */
    inline static
    const TransformationMatrix wedge(const TangentVector& phi) {
        TransformationMatrix Phi;
        Phi <<  static_cast<Scalar>(0), -phi(2),                  phi(1),
                phi(2),                  static_cast<Scalar>(0), -phi(0),
               -phi(1),                  phi(0),        static_cast<Scalar>(0);
        return Phi;
    }

    //! SO(3) vee operator as defined by Barfoot
    /*!
        This is the inverse operator to SO3::wedge.
    */
    inline static const TangentVector vee( const TransformationMatrix& Phi ) {
        TangentVector phi;
        phi << Phi(2,1) - Phi(1,2),
               Phi(0,2) - Phi(2,0),
               Phi(1,0) - Phi(0,1);
        return static_cast<Scalar>(0.5) * phi;
    }

    //! Exponential map for SO(3)
    /*!
        Computes a rotation matrix from axis-angle tangent vector.

        This is the inverse operation to SO3Group::log.
    */
    inline static const SO3Group<Scalar> exp( const TangentVector& phi ) {
        Scalar angle = phi.norm();

        // If angle is close to zero, use first-order Taylor expansion
        if(angle <= std::numeric_limits<Scalar>::epsilon()) {
            return SO3Group<Scalar>(TransformationMatrix::Identity()
                                    + wedge(phi));
        }

        TangentVector axis = phi / angle;
        Scalar cp = cos(angle);
        Scalar sp = sin(angle);

        TransformationMatrix mat = cp * TransformationMatrix::Identity()
                        + (static_cast<Scalar>(1) - cp) * axis * axis.transpose()
                        + sp * wedge(axis);
        return SO3Group<Scalar>(mat);
    }

    //! Logarithmic map for SO(3)
    /*!
        Computes an axis-angle tangent vector from a rotation matrix.

        This is the inverse operation to SO3Group::exp.
    */
    inline static const TangentVector log( const SO3Group<Scalar>& C ) {
        // Normalize C to ensure it is a valid rotation matrix
        SO3Group<Scalar> C_normalized = C;
        C_normalized.normalize();
        // std::cout << "C" << std::endl << C << std::endl;
        // std::cout << "C_normalized" << std::endl << C_normalized << std::endl;

        // Get the rotation angle from the trace of C
        Scalar angle = acos(static_cast<Scalar>(0.5)
                        * C_normalized.matrix().trace()
                        - static_cast<Scalar>(0.5));
        // std::cerr << angle << std::endl;

        // If angle is close to zero, use first-order Taylor expansion
        if(angle <= std::numeric_limits<Scalar>::epsilon()) {
            return vee(C_normalized.matrix()
                        - TransformationMatrix::Identity());
        }

        // Compute the normalized axis
        TangentVector axis;
        axis(0) = C_normalized.matrix()(2,1) - C_normalized.matrix()(1,2);
        axis(1) = C_normalized.matrix()(0,2) - C_normalized.matrix()(2,0);
        axis(2) = C_normalized.matrix()(1,0) - C_normalized.matrix()(0,1);
        axis /= (static_cast<Scalar>(2) * sin(angle));

        return angle * axis;
    }

    //! Convert to a string
    inline
    const std::string str() const {
        std::stringstream ss;
        ss << this->matrix().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for SO(3)
    friend std::ostream& operator<<(std::ostream& os, const Derived& d) {
        os << "SO(3) Rotation Matrix" << std::endl << d.matrix();
        return os;
    }
};

//! SO3Group default type
template <typename _Scalar, int _Options>
class SO3Group : public SO3GroupBase<SO3Group<_Scalar, _Options> > {
    //! Base class definition
    typedef SO3GroupBase<SO3Group<_Scalar, _Options> > Base;

public:
    //! Scalar type
    typedef typename Eigen::internal::traits<
        SO3Group<_Scalar, _Options> >::Scalar Scalar;
    //! Internal storage type
    typedef typename Eigen::internal::traits<
        SO3Group<_Scalar, _Options> >::StorageType StorageType;

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
    //! Transformation matrix type
    typedef typename Base::TransformationMatrix TransformationMatrix;
    //! Adjoint transformation type
    typedef typename Base::AdjointMatrix AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef typename Base::PerturbationJacobian PerturbationJacobian;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Default constructor (identity)
    inline SO3Group() { matrix_.setIdentity(); }
    //! Copy constructor
    template <typename OtherDerived> inline
    SO3Group(const SO3GroupBase<OtherDerived>& other) :
            matrix_(other.matrix() ) { }
    //! Construct from rotation matrix
    SO3Group(const TransformationMatrix& matrix) :
            matrix_(matrix) { }

    //! Return a reference to the underlying matrix
    inline StorageType& matrix() { return matrix_; }
    //! Return a const reference to the underlying matrix
    inline const StorageType& matrix() const { return matrix_; }

private:
    //! Internal storage
    StorageType matrix_;
};

} // namespace ceres_slam

namespace Eigen {
//! Specialization of Eigen::Map for SO3Group
template <typename _Scalar, int _Options>
class Map<ceres_slam::SO3Group<_Scalar>, _Options>
    : public ceres_slam::SO3GroupBase<
        Map<ceres_slam::SO3Group<_Scalar>, _Options> > {

    //! Base class definition
    typedef ceres_slam::SO3GroupBase<
        Map<ceres_slam::SO3Group<_Scalar>, _Options> > Base;

public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;
    //! Internal storage type
    typedef typename internal::traits<Map>::StorageType StorageType;

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
    //! Transformation matrix type
    typedef typename Base::TransformationMatrix TransformationMatrix;
    //! Adjoint transformation type
    typedef typename Base::AdjointMatrix AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef typename Base::PerturbationJacobian PerturbationJacobian;

    // Inherit operators from base class
    using Base::operator=;
    using Base::operator*;

    //! Construct from POD array
    Map(Scalar* data) : matrix_(data) { }

    //! Return a reference to the underlying matrix
    inline StorageType& matrix() { return matrix_; }
    //! Return a const reference to the underlying matrix
    inline const StorageType& matrix() const { return matrix_; }

private:
    //! Internal storage
    StorageType matrix_;
};

//! Specialization of Eigen::Map for const SO3Group
template <typename _Scalar, int _Options>
class Map<const ceres_slam::SO3Group<_Scalar>, _Options>
    : public ceres_slam::SO3GroupBase<
        Map<const ceres_slam::SO3Group<_Scalar>, _Options> > {

    //! Base class definition
    typedef ceres_slam::SO3GroupBase<
        Map<const ceres_slam::SO3Group<_Scalar>, _Options> > Base;

public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;
    //! Internal storage type
    typedef typename internal::traits<Map>::StorageType StorageType;

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
    //! Transformation matrix type
    typedef typename Base::TransformationMatrix TransformationMatrix;
    //! Adjoint transformation type
    typedef typename Base::AdjointMatrix AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef typename Base::PerturbationJacobian PerturbationJacobian;

    // Inherit operators from base class
    using Base::operator=;
    using Base::operator*;

    //! Construct from POD array
    Map(const Scalar* data) : matrix_(data) { }

    //! Return a const reference to the underlying matrix
    inline const StorageType& matrix() const { return matrix_; }

private:
    //! Internal storage
    const StorageType matrix_;
};

}

#endif // CERES_SLAM_GEOMETRY_SO3GROUP_H_
