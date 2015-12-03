#ifndef CERES_SLAM_GEOMETRY_H_
#define CERES_SLAM_GEOMETRY_H_

#include <memory>
#include <iostream>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // For SO(3) log

namespace ceres_slam {

//! Base class for homogeneous points/vectors
template <typename Scalar>
class Homogeneous3D {
public:
    //! Pointer type
    typedef std::shared_ptr<Homogeneous3D> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<Homogeneous3D> ConstPtr;
    //! Dimension of the space
    static const int dim = 3;
    //! Euclidean type
    typedef Eigen::Matrix<Scalar, dim, 1> CartesianScalar;
    //! Homogeneous type
    typedef Eigen::Matrix<Scalar, dim+1, 1> HomogeneousScalar;

    //! Default constructor
    Homogeneous3D() : Homogeneous3D(HomogeneousScalar::Zero()) { }
    //! Copy constructor
    Homogeneous3D( const Homogeneous3D& other ) :
        Homogeneous3D(other.homogeneous) { }
    //! Construct from a 4-vector
    Homogeneous3D( const HomogeneousScalar& vector ) :
        Homogeneous3D(vector.head(3), vector(3)) { }
    //! Construct from a 3-vector and a scalar
    Homogeneous3D( const CartesianScalar& epsilon, const Scalar eta ) :
        _epsilon(epsilon), _eta(eta) { }
    //! Construct from 4 scalars
    Homogeneous3D( Scalar x, Scalar y, Scalar z, Scalar w ) :
        _epsilon(CartesianScalar(x, y, z)), _eta(w) { }

    //! Return the cartesian form of the point/vector
    const CartesianScalar cartesian() const { return _epsilon; }
    //! Return the homogeneous scale part of the point/vector
    const Scalar scale() const { return _eta; }
    //! Return the homogeneous form of the point
    const HomogeneousScalar homogeneous() const {
        HomogeneousScalar h;
        h.head(3) = cartesian();
        h(3) = scale();
        return h;
    }

    //! Ostream operator for homogeneous quantities
    friend std::ostream& operator<<(
        std::ostream& os, const Homogeneous3D<Scalar>& h ) {
        os << "Homogeneous quantity" << std::endl
           << h.homogeneous() << std::endl;
        return os;
    }
protected:
    //! Cartesian part
    CartesianScalar _epsilon;
    //! Homogeneous scale part
    Scalar _eta;
};

//! Vector in 3D space
template <typename Scalar>
class Vector3D : public Homogeneous3D<Scalar> {
public:
    //! Pointer type
    typedef std::shared_ptr<Vector3D> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<Vector3D> ConstPtr;
    //! Cartesian type
    typedef typename Homogeneous3D<Scalar>::CartesianScalar CartesianScalar;
    //! Homogeneous type
    typedef typename Homogeneous3D<Scalar>::HomogeneousScalar HomogeneousScalar;

    //! Default constructor
    Vector3D() : Vector3D(CartesianScalar::Zero()) { }
    //! Copy constructor
    Vector3D(const Vector3D& other) : Vector3D(other.cartesian()) { }
    //! Construct from a 3-vector
    Vector3D(const CartesianScalar& epsilon) :
        Homogeneous3D<Scalar>(epsilon, 0.) { }
    //! Construct from 3 scalars
    Vector3D( Scalar i, Scalar j, Scalar k ) :
        Homogeneous3D<Scalar>(i, j, k, 0.) { }

    //! Addition operator for two vectors
    const Vector3D operator+( const Vector3D<Scalar>& other ) const {
        return Vector3D( this->cartesian() + other.cartesian() );
    }
    //! Subtraction operator for two vectors
    const Vector3D operator-( const Vector3D<Scalar>& other ) const {
        return Vector3D( this->cartesian() - other.cartesian() );
    }

    //! Ostream operator for homogeneous quantities
    friend std::ostream& operator<<( std::ostream& os,
        const Vector3D<Scalar>& v ) {
        os << "Homogeneous vector" << std::endl
           << v.homogeneous() << std::endl;
        return os;
    }
};

//! Point in 3D space
template <typename Scalar>
class Point3D : public Homogeneous3D<Scalar> {
public:
    //! Pointer type
    typedef std::shared_ptr<Point3D> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<Point3D> ConstPtr;
    //! Cartesian type
    typedef typename Homogeneous3D<Scalar>::CartesianScalar CartesianScalar;
    //! Homogeneous type
    typedef typename Homogeneous3D<Scalar>::HomogeneousScalar HomogeneousScalar;

    //! Default constructor
    Point3D() : Point3D(CartesianScalar::Zero()) { }
    //! Copy constructor
    Point3D(const Point3D& other) : Point3D(other.cartesian()) { }
    //! Construct from a 3-vector
    Point3D(const CartesianScalar& epsilon) :
        Homogeneous3D<Scalar>(epsilon, 1.) { }
    //! Construct from 3 scalars
    Point3D( Scalar x, Scalar y, Scalar z ) :
        Homogeneous3D<Scalar>(x, y, z, 1.) { }

    //! Addition operator for point and vector
    const Point3D operator+( const Vector3D<Scalar>& other ) const {
        return Point3D( this->cartesian() + other.cartesian() );
    }
    //! Subtraction operator for point and vector
    const Point3D operator-( const Vector3D<Scalar>& other ) const {
        return Point3D( this->cartesian() - other.cartesian() );
    }
    //! Subtraction operator for point and point
    const Vector3D<Scalar> operator-( const Point3D& other ) const {
        return Vector3D<Scalar>( this->cartesian() - other.cartesian() );
    }

    //! Ostream operator for homogeneous quantities
    friend std::ostream& operator<<( std::ostream& os,
        const Point3D<Scalar>& p ) {
        os << "Homogeneous point" << std::endl
           << p.homogeneous() << std::endl;
        return os;
    }
};

//! SO3 Lie group (3D rotations)
template <typename Scalar>
class SO3 {
public:
    //! Pointer type
    typedef std::shared_ptr<SO3> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<SO3> ConstPtr;
    //! Degrees of freedom (3 for rotation)
    static const int dof = 3;
    //! Dimension of transformation matrix
    static const int dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Group transformation type
    typedef Eigen::Matrix<Scalar, dim, dim> TransformationMatrix;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof> AdjointMatrix;
    //! Transformed point Jacobian matrix
    typedef Eigen::Matrix<Scalar, dim, dof> TransformedPointJacobian;

    //! Default constructor (identity)
    SO3() : SO3(TransformationMatrix::Identity()) { }
    //! Copy constructor
    SO3( const SO3& other ) : _mat(other.matrix()) { };
    //! Construct from a matrix
    SO3( const TransformationMatrix& mat ) : _mat(mat) { }
    //! Matrix exponential: rotation matrix from axis-angle tangent vector
    inline
    static const SO3 exp( const TangentVector& a ) {
        Scalar phi = a.norm();
        Scalar cp = cos(phi);
        Scalar sp = sin(phi);

        TangentVector a_unit = a / phi;

        TransformationMatrix mat = cp * TransformationMatrix::Identity()
                                    + (1 - cp) * a_unit * a_unit.transpose()
                                    + sp * wedge(a_unit);
        return SO3(mat);
    }

    // TODO: Matrix logarithm: axis-angle tangent vector from rotation matrix

    //! Return the transformation matrix
    inline
    const TransformationMatrix matrix() const { return _mat; }

    //! Return the inverse of the transformation matrix
    inline
    const SO3 inverse() const {
        return SO3(matrix().transpose());
    }

    //! Assignment operator
    inline
    SO3 operator=( const SO3& other ) {
        _mat = other.matrix();
        return *this;
    }

    //! Multiply two group elements
    inline
    const SO3 operator*( const SO3& other ) const {
        SO3 result(matrix() * other.matrix());
        return result;
    }

    //! Transform a 3D point
    inline
    const Point operator*( const Point& pt ) const {
        return Point(matrix() * pt.cartesian());
    }

    //! Transform a 3D vector
    inline
    const Vector operator*( const Vector& vec ) const {
        // For normal vectors, this should technically be the inverse-transpose,
        // but they cancel out for rotation matrices
        return Vector(matrix() * vec.cartesian());
    }

    // TODO: Left Jacobian of an element of SO(3)

    //! SO(3) wedge operator as defined by Barfoot
    inline
    static const TransformationMatrix wedge( const TangentVector& phi ) {
        TransformationMatrix Phi;
        Phi <<  0.,    -phi(2), phi(1),
                phi(2), 0.,    -phi(0),
               -phi(1), phi(0), 0.;
        return Phi;
    }

    //! SO(3) vee operator as defined by Barfoot
    inline
    static const TangentVector vee( const TransformationMatrix& Phi ) {
        TangentVector phi;
        phi << Phi(2,1) - Phi(1,2),
               Phi(0,2) - Phi(2,0),
               Phi(1,0) - Phi(0,1);
        return 0.5 * phi;
    }

    //! SO(3) transformed point Jacobian
    inline
    static const TransformedPointJacobian transformed_point_jacobian(
        const Homogeneous3D<Scalar>& h) {
        TransformedPointJacobian result = -wedge(h.cartesian());
        return result;
    }

    //! Ostream operator for SO(3)
    friend std::ostream& operator<<( std::ostream& os,
        const SO3<Scalar>& so3 ) {
        os << "SO(3) Rotation Matrix" << std::endl << so3.matrix() << std::endl;
        return os;
    }

private:
    //! Internal storage
    TransformationMatrix _mat;
};

//! SE3 Lie group (3D rigid body transformations)
template <typename Scalar>
class SE3 {
public:
    //! Pointer type
    typedef std::shared_ptr<SE3> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<SE3> ConstPtr;
    //! Degrees of freedom (3 for rotation)
    static const int dof = 6;
    //! Dimension of transformation matrix
    static const int dim = 4;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Rotation type
    typedef SO3<Scalar> Rotation;
    //! Group transformation type
    typedef Eigen::Matrix<Scalar, dim, dim> TransformationMatrix;
    //! Tangent vector type
    typedef Eigen::Matrix<Scalar, dof, 1> TangentVector;
    //! Adjoint transformation type
    typedef Eigen::Matrix<Scalar, dof, dof> AdjointMatrix;
    //! Transformed (Cartesian) point Jacobian matrix
    typedef Eigen::Matrix<Scalar, dim, dof> TransformedPointJacobian;

    //! Default constructor (identity)
    SE3() : SE3(TransformationMatrix::Identity()) { }
    //! Construct from a rotation and a translation
    SE3( const Rotation& rotation, const Vector& translation )
        : _rotation(rotation), _translation(translation) { }
    //! Construct from a matrix
    SE3( const TransformationMatrix& mat ) {
        _rotation = Rotation(mat.block(0,0,3,3));
        _translation = Vector(mat.block(0,3,3,1));
    }

    // TODO: Matrix exponential: rotation matrix from axis-angle tangent vector
    // TODO: Matrix logarithm: axis-angle tangent vector from rotation matrix

    //! Return the SO3 rotation
    inline
    const Rotation rotation() const { return _rotation; }

    //! Return the translation vector
    inline
    const Vector translation() const { return _translation; }

    //! Return the transformation matrix
    inline
    const TransformationMatrix matrix() const {
        TransformationMatrix mat;
        mat.block(0,0,3,3) = rotation().matrix();
        mat.block(0,3,3,1) = translation().cartesian();
        mat.bottomRows(1) << 0., 0., 0., 1.;
        return mat;
    }

    //! Return the inverse of the transformation matrix
    inline
    const SE3 inverse() const {
        TransformationMatrix inv;
        inv.block(0,0,3,3) = rotation().inverse().matrix();
        inv.block(0,3,3,1) = rotation().inverse() * translation().cartesian();
        inv.bottomRows(1) << 0., 0., 0., 1.;
        return SE3(inv);
    }

    //! Assignment operator
    inline
    SE3 operator=( const SE3& other ) {
        _rotation = other.rotation();
        _translation = other.translation();
        return *this;
    }

    //! Multiply two group elements
    inline
    const SE3 operator*( const SE3& other ) {
        return SE3( rotation() * other.rotation(),
                    rotation() * other.translation() + translation() );
    }

    //! Transform a 3D point
    inline
    const Point operator*( const Point& pt ) {
        return Point(rotation() * pt + translation());
    }

    //! Transform a 3D vector
    inline
    const Vector operator*( const Vector& vec ) {
        // For normal vectors, this should technically be the inverse-transpose,
        // but they cancel out for rotation matrices
        return rotation() * vec;
    }

    //! SE(3) wedge operator as defined by Barfoot
    inline
    static const TransformationMatrix wedge( const TangentVector& xi ) {
        TransformationMatrix Xi;
        Xi.block(0,0,3,3) = SO3<Scalar>::wedge(xi.tail(3));
        Xi.block(0,3,3,1) = xi.head(3);
        Xi.bottomRows(1) << 0., 0., 0., 0.;
        return Xi;
    }

    //! SE(3) vee operator as defined by Barfoot
    inline
    static const TangentVector vee( const TransformationMatrix& Xi ) {
        TangentVector xi;
        xi.head(3) = Xi.block(0,3,3,1);
        xi.tail(3) = Rotation::vee(Xi.block(0,0,3,3));
        return xi;
    }

    //! SE(3) odot operator as defined by Barfoot
    //! (for a homogeneous point/vector p = [epsilon^T eta]^T)
    inline
    static const TransformedPointJacobian transformed_point_jacobian(
        const Homogeneous3D<Scalar>& h ) {
        TransformedPointJacobian result;
        result.block(0,0,3,3) =
            h.scale() * Rotation::TransformationMatrix::Identity();
        result.block(0,3,3,3) =
            Rotation::transformed_point_jacobian(h);
        result.bottomRows(1) << 0., 0., 0., 0., 0., 0.;
        return result;
    }

    //! Ostream operator for SE(3)
    friend std::ostream& operator<<(std::ostream& os, const SE3<Scalar>& se3) {
        os << "SE(3) Transformation Matrix" << std::endl << se3.matrix() << std::endl;
        return os;
    }

private:
    //! Internal storage
    Rotation _rotation;
    Vector _translation;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_H_
