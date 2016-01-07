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

namespace ceres_slam {

//! SO(3) Lie group (3D rotations)
template <typename Scalar>
class SO3Group {
public:
    //! Pointer type
    typedef std::shared_ptr<SO3Group> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const SO3Group> ConstPtr;
    //! Degrees of freedom (3 for rotation)
    static const int dof = 3;
    //! Dimension of transformation matrix
    static const int dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Homogeneous quantity type
    typedef Homogeneous3D<Scalar> Homogeneous;
    //! Group transformation type
    typedef Eigen::Matrix<Scalar, dim, dim, Eigen::RowMajor>
        TransformationMatrix;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof, Eigen::RowMajor> AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef Eigen::Matrix<Scalar, Point::dim, dof, Eigen::RowMajor>
        TransformedPointJacobian;

    //! Default constructor (identity)
    SO3Group() : SO3Group(TransformationMatrix::Identity()) { }
    //! Copy constructor
    SO3Group( const SO3Group& other ) : mat_(other.matrix()) { };
    //! Construct from a matrix
    SO3Group( const TransformationMatrix& mat ) : mat_(mat) { }

    //! Assignment operator
    inline SO3Group operator=( const SO3Group& other ) {
        this->mat_ = other.matrix();
        return *this;
    }

    //! Return the transformation matrix
    inline const TransformationMatrix matrix() const { return this->mat_; }

    //! Return the inverse transformation
    inline const SO3Group inverse() const {
        return SO3Group(this->matrix().transpose());
    }

    //! Normalize the transformation matrix to ensure it is a valid element of
    //! SO(3)
    inline void normalize() {
        Eigen::Transform<Scalar, 3, Eigen::Affine> temp;
        temp.linear() = mat_;
        mat_ = temp.rotation();
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

        Homogeneous h_transformed(this->matrix() * h.cartesian(), h.scale());

        if(jacobian_ptr != nullptr) {
            TransformedPointJacobian& jacobian = *jacobian_ptr;
            jacobian = wedge(-(this->matrix() * h.cartesian()));
        }

        return h_transformed;
    }

    //! Multiplication operator for two group elements
    inline const SO3Group operator*( const SO3Group& other ) const {
        SO3Group result(this->matrix() * other.matrix());
        return result;
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

    //! SO(3) identity element
    inline static const SO3Group Identity() {
        return SO3Group(TransformationMatrix::Identity());
    }

    //! SO(3) wedge operator as defined by Barfoot
    /*!
        This is the inverse operator to SO3::vee.
    */
    inline static const TransformationMatrix wedge( const TangentVector& phi ) {
        TransformationMatrix Phi;
        Phi <<  Scalar(0), -phi(2),     phi(1),
                phi(2),     Scalar(0), -phi(0),
               -phi(1),     phi(0),     Scalar(0);
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
        return 0.5 * phi;
    }

    //! Exponential map for SO(3)
    /*!
        Computes a rotation matrix from axis-angle tangent vector.

        This is the inverse operation to SO3Group::log.
    */
    inline static const SO3Group exp( const TangentVector& phi ) {
        Scalar angle = phi.norm();

        // If angle is close to zero, use first-order Taylor expansion
        if(angle <= std::numeric_limits<Scalar>::epsilon()) {
            return SO3Group(TransformationMatrix::Identity() + wedge(phi));
        }

        TangentVector axis = phi / angle;
        Scalar cp = cos(angle);
        Scalar sp = sin(angle);

        TransformationMatrix mat = cp * TransformationMatrix::Identity()
                                    + (Scalar(1) - cp) * axis * axis.transpose()
                                    + sp * wedge(axis);
        return SO3Group(mat);
    }

    //! Logarithmic map for SO(3)
    /*!
        Computes an axis-angle tangent vector from a rotation matrix.

        This is the inverse operation to SO3Group::exp.
    */
    inline static const TangentVector log( const SO3Group& C ) {
        // Normalize C to ensure it is a valid rotation matrix
        SO3Group C_normalized = C;
        C_normalized.normalize();
        // std::cout << "C" << std::endl << C << std::endl;
        // std::cout << "C_normalized" << std::endl << C_normalized << std::endl;

        // Get the rotation angle from the trace of C
        Scalar angle = acos(Scalar(0.5) * C_normalized.matrix().trace()
                            - Scalar(0.5));
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
        axis /= (Scalar(2) * sin(angle));

        return angle * axis;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->matrix().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for SO(3)
    friend std::ostream& operator<<( std::ostream& os,
                                     const SO3Group<Scalar>& so3 ) {
        os << "SO(3) Rotation Matrix" << std::endl << so3.matrix() << std::endl;
        return os;
    }

private:
    //! Internal storage for the rotation
    TransformationMatrix mat_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_SO3GROUP_H_
