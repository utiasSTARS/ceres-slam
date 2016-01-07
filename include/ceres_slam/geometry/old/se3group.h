#ifndef CERES_SLAM_GEOMETRY_SE3GROUP_H_
#define CERES_SLAM_GEOMETRY_SE3GROUP_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry/point3d.h>
#include <ceres_slam/geometry/vector3d.h>
#include <ceres_slam/geometry/so3group.h>

namespace ceres_slam {

//! SE(3) Lie group (3D rigid body transformations)
template <typename Scalar>
class SE3Group {
public:
    //! Pointer type
    typedef std::shared_ptr<SE3Group> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const SE3Group> ConstPtr;
    //! Degrees of freedom (3 for rotation + 3 for translation)
    static const int dof = 6;
    //! Dimension of transformation matrix
    static const int dim = 4;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Homogeneous quantity type
    typedef Homogeneous3D<Scalar> Homogeneous;
    //! SO(3) type
    typedef SO3Group<Scalar> SO3;
    //! Group transformation type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor>
        TransformationMatrix;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof, Eigen::RowMajor> AdjointMatrix;
    //! Transformed (Cartesian) point Jacobian matrix
    typedef Eigen::Matrix<Scalar, Point::dim, dof, Eigen::RowMajor>
        TransformedPointJacobian;

    //! Default constructor (identity)
    SE3Group() : SE3Group(TransformationMatrix::Identity()) { }
    //! Construct from a rotation and a translation
    SE3Group( const Vector& translation, const SO3& rotation ) :
        translation_(translation), rotation_(rotation) { }
    //! Construct from a matrix
    SE3Group( const TransformationMatrix& mat ) {
        translation_ = Vector(mat.block(0,3,3,1));
        rotation_ = SO3(mat.block(0,0,3,3));
    }

    //! Return the SO(3) rotation
    inline const SO3 rotation() const { return this->rotation_; }

    //! Return the translation vector
    inline const Vector translation() const { return this->translation_; }

    //! Return the transformation matrix
    inline const TransformationMatrix matrix() const {
        TransformationMatrix mat;
        mat.block(0,0,3,3) = this->rotation().matrix();
        mat.block(0,3,3,1) = this->translation().cartesian();
        mat.bottomRows(1) << Scalar(0), Scalar(0), Scalar(0), Scalar(1);
        return mat;
    }

    //! Return the inverse transformation
    inline const SE3Group inverse() const {
        TransformationMatrix inv;
        inv.block(0,0,3,3) = this->rotation().inverse().matrix();
        inv.block(0,3,3,1) =
            -(this->rotation().inverse() * this->translation()).cartesian();
        inv.bottomRows(1) << Scalar(0), Scalar(0), Scalar(0), Scalar(1);
        return SE3Group(inv);
    }

    //! Assignment operator
    inline SE3Group operator=( const SE3Group& other ) {
        translation_ = other.translation();
        rotation_ = other.rotation();
        return *this;
    }

    //! Transform a 3D homogeneous quantity.
    /*!
        Transform a 3D homogeneous quantity and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline const Homogeneous transform(
        const Homogeneous& h,
        TransformedPointJacobian* jacobian_ptr = nullptr ) const {

        Homogeneous h_transformed = this->rotation() * h
                                  + h.scale() * this->translation();

        if(jacobian_ptr != nullptr) {
            TransformedPointJacobian& jacobian = *jacobian_ptr;
            jacobian.block(0,0,3,3) =
                h.scale() * SO3::TransformationMatrix::Identity();
            jacobian.block(0,3,3,3) = SO3::wedge(-h_transformed.cartesian());
        }

        return h_transformed;
    }

    //! Multiplication operator for two group elements
    inline const SE3Group operator*( const SE3Group& other ) const {
        return SE3Group(
            this->rotation() * other.translation() + this->translation(),
            this->rotation() * other.rotation() );
    }

    //! Multiplication operator for group element and homogeneous coordinate
    inline const Homogeneous operator*( const Homogeneous& h ) const {
        return this->transform(h);
    }
    //! Multiplication operator for group element and vector (preserves type)
    inline const Vector operator*( const Vector& v ) const {
        return static_cast<Vector>(this->transform(v));
    }
    //! Multiplication operator for group element and point (preserves type)
    inline const Point operator*( const Point& p ) const {
        return static_cast<Point>(this->transform(p));
    }

    //! SE(3) Identity element
    inline static const SE3Group Identity() {
        return SE3Group(TransformationMatrix::Identity());
    }

    //! SE(3) vee operator as defined by Barfoot
    /*!
        This is the inverse operator to SE3::vee.
    */
    inline static const TransformationMatrix wedge( const TangentVector& xi ) {
        TransformationMatrix Xi;
        Xi.block(0,0,3,3) = SO3::wedge(xi.tail(3));
        Xi.block(0,3,3,1) = xi.head(3);
        Xi.bottomRows(1) << Scalar(0), Scalar(0), Scalar(0), Scalar(0);
        return Xi;
    }

    //! SE(3) vee operator as defined by Barfoot
    /*!
        This is the inverse operator to SE3::wedge.
    */
    inline static const TangentVector vee( const TransformationMatrix& Xi ) {
        TangentVector xi;
        xi.head(3) = Xi.block(0,3,3,1);
        xi.tail(3) = SO3::vee(Xi.block(0,0,3,3));
        return xi;
    }

    //! Exponential map for SE(3)
    /*!
        Computes a transformation matrix from SE(3) tangent vector.

        This isn't quite right because the translational component
        should be multiplied by the inverse SO(3) Jacobian, but we don't really
        need this.

        This is the inverse operator to SE3Group::log.
    */
    inline static const SE3Group exp( const TangentVector& xi ) {
        return SE3Group( Vector(xi.head(3)), SO3::exp(xi.tail(3)) );
    }

    //! Logarithmic map for SE(3)
    /*!
        Computes a SE(3) tangent vector from a transformation matrix.

        This isn't quite right because the translational component
        should be multiplied by the inverse SO(3) Jacobian, but we don't really
        need this.

        This is the inverse operator to SE3Group::exp.
    */
    inline static const TangentVector log( const SE3Group& T ) {
        TangentVector xi;
        xi.head(3) = T.translation().cartesian();
        xi.tail(3) = SO3::log(T.rotation());
        return xi;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << matrix().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for SE(3)
    friend std::ostream& operator<<( std::ostream& os,
                                     const SE3Group<Scalar>& se3 ) {
        os << "SE(3) Transformation Matrix" << std::endl << se3.matrix() << std::endl;
        return os;
    }

private:
    //! Internal storage for the translation
    Vector translation_;
    //! Internal storage for the rotation
    SO3 rotation_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_SE3GROUP_H_
