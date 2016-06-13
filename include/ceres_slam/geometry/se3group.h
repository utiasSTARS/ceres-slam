#ifndef CERES_SLAM_GEOMETRY_SE3GROUP_H_
#define CERES_SLAM_GEOMETRY_SE3GROUP_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres_slam/utils/utils.h>

#include "point3d.h"
#include "so3group.h"
#include "vector3d.h"

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {
template <typename _Scalar, int _Options = 0>
class SE3Group;
}  // namespace ceres_slam

///////////////////////////////////////////////////////////////////////////////
// Eigen traits for querying derived types in CTRP hierarchy
///////////////////////////////////////////////////////////////////////////////
namespace Eigen {
namespace internal {

template <typename _Scalar, int _Options>
struct traits<ceres_slam::SE3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef ceres_slam::SO3Group<Scalar> RotationStorageType;
    typedef ceres_slam::Vector3D<Scalar> TranslationStorageType;
};

template <typename _Scalar, int _Options>
struct traits<Map<ceres_slam::SE3Group<_Scalar, _Options> > >
    : traits<ceres_slam::SE3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<ceres_slam::SO3Group<Scalar>, _Options> RotationStorageType;
    typedef Map<ceres_slam::Vector3D<Scalar>, _Options> TranslationStorageType;
};

template <typename _Scalar, int _Options>
struct traits<Map<const ceres_slam::SE3Group<_Scalar, _Options> > >
    : traits<const ceres_slam::SE3Group<_Scalar, _Options> > {
    typedef _Scalar Scalar;
    typedef Map<const ceres_slam::SO3Group<Scalar>, _Options>
        RotationStorageType;
    typedef Map<const ceres_slam::Vector3D<Scalar>, _Options>
        TranslationStorageType;
};
}
}  // namespace Eigen::internal

///////////////////////////////////////////////////////////////////////////////
// Implementation
///////////////////////////////////////////////////////////////////////////////
namespace ceres_slam {

//! SE3Group base class (storage agnostic)
template <typename Derived>
class SE3GroupBase {
   public:
    //! Scalar type
    typedef typename Eigen::internal::traits<Derived>::Scalar Scalar;
    //! Rotation storage type
    typedef typename Eigen::internal::traits<Derived>::RotationStorageType
        RotationStorageType;
    //! Translation storage type
    typedef typename Eigen::internal::traits<Derived>::TranslationStorageType
        TranslationStorageType;

    //! Degrees of freedom (3 for rotation, 3 for translation)
    static const int dof = 6;
    //! Dimension of transformation matrix
    static const int dim = 4;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Rotation type
    typedef SO3Group<Scalar> SO3;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Transformation matrix type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor>
        TransformationMatrix;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof, Eigen::RowMajor> AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef Eigen::Matrix<Scalar, dim - 1, dof, Eigen::RowMajor>
        PerturbationJacobian;

    //! Return a reference to the underlying rotation
    inline RotationStorageType& rotation() {
        return static_cast<Derived*>(this)->rotation();
    }
    //! Return a const reference to the rotation part
    inline const RotationStorageType& rotation() const {
        return static_cast<const Derived*>(this)->rotation();
    }
    //! Return a reference to the translation part
    inline TranslationStorageType& translation() {
        return static_cast<Derived*>(this)->translation();
    }
    //! Return a const reference to the translation part
    inline const TranslationStorageType& translation() const {
        return static_cast<const Derived*>(this)->translation();
    }
    //! Return a pointer to the raw matrix data
    inline Scalar* data() {
        // translation_ and rotation_ data are adjacent in memory
        return this->translation().data();
    }
    //! Return a const pointer to the raw matrix data
    inline const Scalar* data() const {
        // translation_ and rotation_ data are adjacent in memory
        return this->translation().data();
    }

    //! Return the transformation in matrix form
    inline const TransformationMatrix matrix() const {
        TransformationMatrix matrix;
        matrix.setIdentity();
        matrix.block(0, 0, 3, 3) = this->rotation().matrix();
        matrix.block(0, 3, 3, 1) = this->translation();
        return matrix;
    }

    //! Return a copy of this, casted to OtherScalar
    template <typename OtherScalar>
    inline SE3Group<OtherScalar> cast() const {
        return SE3Group<OtherScalar>(this->translation().cast<OtherScalar>(),
                                     this->rotation().cast<OtherScalar>());
    }

    //! Assignment operator
    template <typename OtherDerived>
    inline SE3GroupBase<Derived>& operator=(
        const SE3GroupBase<OtherDerived>& other) {
        this->translation() = other.translation();
        this->rotation() = other.rotation();
        return *this;
    }

    //! Return the inverse transformation
    inline const SE3Group<Scalar> inverse() const {
        SE3Group<Scalar> result;
        result.rotation() = this->rotation().inverse();
        result.translation() =
            -(this->rotation().inverse() * this->translation());
        return result;
    }

    //! Normalize the underlying matrix to ensure it is a valid transformation
    inline void normalize() { this->rotation().normalize(); }

    //! Multiplication operator for two group elements
    inline const SE3Group<Scalar> operator*(
        const SE3Group<Scalar>& other) const {
        SE3Group<Scalar> result;
        result.rotation() = this->rotation() * other.rotation();
        result.translation() =
            this->rotation() * other.translation() + this->translation();
        return result;
    }

    //! Transform a 3D point.
    /*!
        Transform a 3D point and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline const Point transform(
        const Point& p, PerturbationJacobian* jacobian_ptr = nullptr) const {
        Point p_transformed = this->rotation() * p + this->translation();

        if (jacobian_ptr != nullptr) {
            PerturbationJacobian& jacobian = *jacobian_ptr;
            jacobian.block(0, 0, 3, 3) = SO3::TransformationMatrix::Identity();
            jacobian.block(0, 3, 3, 3) = SO3::wedge(-p_transformed);
        }

        return p_transformed;
    }
    //! Transform a 3D point.
    inline const Point transform(
        const Eigen::Map<Point>& p,
        PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Point(p), jacobian_ptr);
    }
    //! Transform a 3D point.
    inline const Point transform(
        const Eigen::Map<const Point>& p,
        PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Point(p), jacobian_ptr);
    }
    //! Multiplication operator for group element and point
    inline const Point operator*(const Point& p) const {
        return this->transform(p);
    }
    //! Multiplication operator for group element and point
    inline const Point operator*(const Eigen::Map<Point>& p) const {
        return this->transform(p);
    }
    //! Multiplication operator for group element and point
    inline const Point operator*(const Eigen::Map<const Point>& p) const {
        return this->transform(p);
    }

    //! Transform a 3D vector.
    /*!
        Transform a 3D vector and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline const Vector transform(
        const Vector& v, PerturbationJacobian* jacobian_ptr = nullptr) const {
        Vector v_transformed = this->rotation() * v;

        if (jacobian_ptr != nullptr) {
            PerturbationJacobian& jacobian = *jacobian_ptr;
            jacobian.block(0, 0, 3, 3) = SO3::TransformationMatrix::Zero();
            jacobian.block(0, 3, 3, 3) = SO3::wedge(-v_transformed);
        }

        return v_transformed;
    }
    //! Transform a 3D vector.
    inline const Vector transform(
        const Eigen::Map<Vector>& v,
        PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Vector(v), jacobian_ptr);
    }
    //! Transform a 3D vector.
    inline const Vector transform(
        const Eigen::Map<const Vector>& v,
        PerturbationJacobian* jacobian_ptr = nullptr) const {
        return transform(Vector(v), jacobian_ptr);
    }
    //! Multiplication operator for group element and vector
    inline const Vector operator*(const Vector& v) const {
        return this->transform(v);
    }
    //! Multiplication operator for group element and vector
    inline const Vector operator*(const Eigen::Map<Vector>& v) const {
        return this->transform(v);
    }
    //! Multiplication operator for group element and vector
    inline const Vector operator*(const Eigen::Map<const Vector>& v) const {
        return this->transform(v);
    }

    //! Return the identity element of SE(3)
    inline static const SE3Group<Scalar> Identity() {
        return SE3Group<Scalar>();
    }

    //! SE(3) wedge operator as defined by Barfoot
    /*!
        This is the inverse operation to SE3Group::vee.
    */
    inline static const TransformationMatrix wedge(const TangentVector& xi) {
        TransformationMatrix Xi;
        Xi.block(0, 0, 3, 3) = SO3::wedge(xi.tail(3));
        Xi.block(0, 3, 3, 1) = xi.head(3);
        Xi.bottomRows(1) << static_cast<Scalar>(0), static_cast<Scalar>(0),
            static_cast<Scalar>(0), static_cast<Scalar>(0);
        return Xi;
    }

    //! SE(3) vee operator as defined by Barfoot
    /*!
        This is the inverse operation to SE3Group::wedge.
    */
    inline static const TangentVector vee(const TransformationMatrix& Xi) {
        TangentVector xi;
        xi.head(3) = Xi.block(0, 3, 3, 1);
        xi.tail(3) = SO3::vee(Xi.block(0, 0, 3, 3));
        return xi;
    }

    //! Exponential map for SE(3)
    /*!
        Computes a transformation matrix from SE(3) tangent vector.

        This isn't quite right because the translational component
        should be multiplied by the inverse SO(3) Jacobian, but we don't really
        need this.

        This is the inverse operation to SE3Group::log.
    */
    inline static const SE3Group<Scalar> exp(const TangentVector& xi) {
        return SE3Group<Scalar>(xi.head(3), SO3::exp(xi.tail(3)));
    }

    //! Logarithmic map for SE(3)
    /*!
        Computes a SE(3) tangent vector from a transformation matrix.

        This isn't quite right because the translational component
        should be multiplied by the inverse SO(3) Jacobian, but we don't really
        need this.

        This is the inverse operation to SE3Group::exp.
    */
    inline static const TangentVector log(const SE3Group<Scalar>& T) {
        TangentVector xi;
        xi.head(3) = T.translation();
        xi.tail(3) = SO3::log(T.rotation());
        return xi;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->matrix().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for SE(3)
    friend std::ostream& operator<<(std::ostream& os, const Derived& d) {
        os << "SE(3) Transformation Matrix" << std::endl << d.matrix();
        return os;
    }
};

//! SE3Group default type
template <typename _Scalar, int _Options>
class SE3Group : public SE3GroupBase<SE3Group<_Scalar, _Options> > {
    //! Base class definition
    typedef SE3GroupBase<SE3Group<_Scalar, _Options> > Base;

   public:
    //! Scalar type
    typedef
        typename Eigen::internal::traits<SE3Group<_Scalar, _Options> >::Scalar
            Scalar;
    //! Rotation storage type
    typedef typename Eigen::internal::traits<
        SE3Group<_Scalar, _Options> >::RotationStorageType RotationStorageType;
    //! Translation storage type
    typedef typename Eigen::internal::traits<SE3Group<_Scalar, _Options> >::
        TranslationStorageType TranslationStorageType;

    //! Degrees of freedom (3 for rotation)
    static const int dof = Base::dof;
    //! Dimension of transformation matrix
    static const int dim = Base::dim;
    //! Point type
    typedef typename Base::Point Point;
    //! Vector type
    typedef typename Base::Vector Vector;
    //! Rotation type
    typedef typename Base::SO3 SO3;
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
    inline SE3Group() {
        rotation().matrix().setIdentity();
        translation().setZero();
    }
    //! Copy constructor
    template <typename OtherDerived>
    inline SE3Group(const SE3GroupBase<OtherDerived>& other)
        : translation_(other.translation()), rotation_(other.rotation()) {}
    //! Construct from transformation matrix
    SE3Group(const TransformationMatrix& matrix)
        : translation_(matrix.block(0, 3, 3, 1)),
          rotation_(matrix.block(0, 0, 3, 3)) {}
    //! Construct from a translation and a rotation
    SE3Group(const Vector& translation, const SO3& rotation)
        : translation_(translation), rotation_(rotation) {}

    //! Return a reference to the underlying rotation
    inline RotationStorageType& rotation() { return rotation_; }
    //! Return a const reference to the rotation part
    inline const RotationStorageType& rotation() const { return rotation_; }
    //! Return a reference to the translation part
    inline TranslationStorageType& translation() { return translation_; }
    //! Return a const reference to the translation part
    inline const TranslationStorageType& translation() const {
        return translation_;
    }

   private:
    //! Internal storage (translation)
    TranslationStorageType translation_;
    //! Internal storage (rotation)
    RotationStorageType rotation_;
};

}  // namespace ceres_slam

namespace Eigen {
//! Specialization of Eigen::Map for SE3Group
template <typename _Scalar, int _Options>
class Map<ceres_slam::SE3Group<_Scalar>, _Options>
    : public ceres_slam::SE3GroupBase<
          Map<ceres_slam::SE3Group<_Scalar>, _Options> > {
    //! Base class definition
    typedef ceres_slam::SE3GroupBase<
        Map<ceres_slam::SE3Group<_Scalar>, _Options> >
        Base;

   public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;
    //! Rotation storage type
    typedef
        typename internal::traits<Map>::RotationStorageType RotationStorageType;
    //! Translation storage type
    typedef typename internal::traits<Map>::TranslationStorageType
        TranslationStorageType;

    //! Degrees of freedom (3 for rotation)
    static const int dof = Base::dof;
    //! Dimension of transformation matrix
    static const int dim = Base::dim;
    //! Point type
    typedef typename Base::Point Point;
    //! Vector type
    typedef typename Base::Vector Vector;
    //! Rotation type
    typedef typename Base::SO3 SO3;
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
    Map(Scalar* array) : translation_(array), rotation_(array + 3) {}

    //! Return a reference to the rotation part
    inline RotationStorageType& rotation() { return rotation_; }
    //! Return a const reference to the rotation part
    inline const RotationStorageType& rotation() const { return rotation_; }
    //! Return a reference to the translation part
    inline TranslationStorageType& translation() { return translation_; }
    //! Return a const reference to the translation part
    inline const TranslationStorageType& translation() const {
        return translation_;
    }

   private:
    //! Internal storage (translation)
    TranslationStorageType translation_;
    //! Internal storage (rotation)
    RotationStorageType rotation_;
};

//! Specialization of Eigen::Map for const SE3Group
template <typename _Scalar, int _Options>
class Map<const ceres_slam::SE3Group<_Scalar>, _Options>
    : public ceres_slam::SE3GroupBase<
          Map<const ceres_slam::SE3Group<_Scalar>, _Options> > {
    //! Base class definition
    typedef ceres_slam::SE3GroupBase<
        Map<const ceres_slam::SE3Group<_Scalar>, _Options> >
        Base;

   public:
    //! Scalar type
    typedef typename internal::traits<Map>::Scalar Scalar;
    //! Rotation storage type
    typedef
        typename internal::traits<Map>::RotationStorageType RotationStorageType;
    //! Translation storage type
    typedef typename internal::traits<Map>::TranslationStorageType
        TranslationStorageType;

    //! Degrees of freedom (3 for rotation)
    static const int dof = Base::dof;
    //! Dimension of transformation matrix
    static const int dim = Base::dim;
    //! Point type
    typedef typename Base::Point Point;
    //! Vector type
    typedef typename Base::Vector Vector;
    //! Rotation type
    typedef typename Base::SO3 SO3;
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
    Map(const Scalar* array) : translation_(array), rotation_(array + 3) {}

    //! Return a const reference to the rotation part
    inline const RotationStorageType& rotation() const { return rotation_; }
    //! Return a const reference to the translation part
    inline const TranslationStorageType& translation() const {
        return translation_;
    }

   private:
    //! Internal storage (translation)
    TranslationStorageType translation_;
    //! Internal storage (rotation)
    RotationStorageType rotation_;
};

}  // namespace Eigen

#endif  // CERES_SLAM_GEOMETRY_SE3GROUP_H_
