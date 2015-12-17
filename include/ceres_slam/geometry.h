#ifndef CERES_SLAM_GEOMETRY_H_
#define CERES_SLAM_GEOMETRY_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

namespace ceres_slam {

//! String formatting for Eigen file IO
const Eigen::IOFormat CommaInitFmt(4, 1, ",", ",", "", "", "", "");

//! Base class for homogeneous coordinates in 3D space
template <typename Scalar>
class Homogeneous3D {
public:
    //! Pointer type
    typedef std::shared_ptr<Homogeneous3D> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const Homogeneous3D> ConstPtr;
    //! Dimension of the space
    static const int dim = 3;
    //! Cartesian type
    typedef Eigen::Matrix<Scalar, dim, 1> Cartesian;
    //! Homogeneous type
    typedef Eigen::Matrix<Scalar, dim+1, 1> Homogeneous;

    //! Default constructor
    Homogeneous3D() : Homogeneous3D(Homogeneous::Zero()) { }
    //! Copy constructor
    Homogeneous3D( const Homogeneous3D& other ) :
        Homogeneous3D(other.homogeneous()) { }
    //! Construct from a 4-vector
    Homogeneous3D( const Homogeneous& vector ) :
        Homogeneous3D(vector.head(3), vector(3)) { }
    //! Construct from a 3-vector and a scalar
    Homogeneous3D( const Cartesian& cartesian, const Scalar scale ) :
        cartesian_(cartesian), scale_(scale) { }
    //! Construct from 4 scalars
    Homogeneous3D( const Scalar x, const Scalar y,
                       const Scalar z, const Scalar w ) :
        cartesian_(Cartesian(x, y, z)), scale_(w) { }
    //! Construct from a 4-element POD array
    Homogeneous3D( const Scalar s[4] ) :
        Homogeneous3D(s[0], s[1], s[2], s[3]) { }

    //! Return the cartesian form of the point/vector
    const Cartesian cartesian() const {
        // Special case for homogeneous vectors (scale = 0)
        if(abs(scale_) <= std::numeric_limits<Scalar>::epsilon()){
            return cartesian_;
        }
        else {
            return cartesian_ / scale_;
        }
    }
    //! Return the homogeneous scale part of the point/vector
    const Scalar scale() const { return scale_; }
    //! Return the homogeneous form of the point
    const Homogeneous homogeneous() const {
        Homogeneous h;
        h.head(3) = cartesian();
        h(3) = scale();
        return h;
    }
    //! Return a const pointer to the underlying array for the cartesian part
    const Scalar* data() const { return cartesian_.data(); }
    //! Return a pointer to the underlying array for the cartesian part
    Scalar* data() { return cartesian_.data(); }

    //! Accessor operator, mutable
    Scalar& operator()(int i) { return cartesian_(i); }
    //! Accessor operator, const
    const Scalar& operator()(int i) const { return cartesian_(i); }

    //! Addition operator for homogeneous coordinates
    const Homogeneous3D operator+( const Homogeneous3D<Scalar>& other ) const {
        return Homogeneous3D( this->cartesian() + other.cartesian(),
                              this->scale() + other.scale() );
    }
    //! Subtraction operator for homogeneous coordinates
    const Homogeneous3D operator-( const Homogeneous3D<Scalar>& other ) const {
        return Homogeneous3D( this->cartesian() - other.cartesian(),
                              this->scale() - other.scale() );
    }
    //! Right-multiplication of homogeneous coordinate by scalar
    const Homogeneous3D operator*( const Scalar s ) const {
        return Homogeneous3D( s * this->cartesian(), s * this->scale() );
    }
    //! Left-multiplication of homogeneous coordinate by scalar
    /*!
        Needs to be a friend inside the class to keep the templating
        and avoid passing 'this' as the first argument.
    */
    friend const Homogeneous3D<Scalar> operator*(
        const Scalar s, const Homogeneous3D<Scalar>& h ) {
        return h * s;
    }
    //! Unary minus operator for homogeneous coordinates
    const Homogeneous3D operator-() const {
        return (*this) * Scalar(-1);
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << cartesian().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for homogeneous quantities
    friend std::ostream& operator<<( std::ostream& os,
                                     const Homogeneous3D<Scalar>& h ) {
        os << "Homogeneous3D("
           << h.homogeneous().format(CommaInitFmt) << ")";
        return os;
    }

protected:
    //! Cartesian part
    Cartesian cartesian_;
    //! Homogeneous scale part
    Scalar scale_;
};

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
        if(abs(h.scale()) >= std::numeric_limits<Scalar>::epsilon()) {
            std::cerr << "Warning: implicit cast from Homogeneous3D to Vector3D"
                      << " with scale = " << h.scale() << " != 0" << std::endl;
        }
    }

    //! Compute the squared norm of the vector
    const Scalar squaredNorm() const { return this->cartesian().squaredNorm(); }
    //! Compute the norm of the vector
    const Scalar norm() const { return this->cartesian().norm(); }
    //! Normalize the vector
    void normalize() { this->cartesian_.normalize(); }
    //! Compute the dot product of two vectors
    const Scalar dot( const Vector3D& other ) const {
        return this->cartesian().dot(other.cartesian());
    }

    //! Ostream operator for Vector3D
    friend std::ostream& operator<<( std::ostream& os,
                                     const Vector3D<Scalar>& v ) {
        os << "Vector3D(" << v.homogeneous().format(CommaInitFmt) << ")";
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
        if(abs(h.scale()) <= std::numeric_limits<Scalar>::epsilon()) {
            std::cerr << "Warning: implicit cast from Homogeneous3D to Point3D"
                      << " with scale = " << h.scale() << " == 0" << std::endl;
        }
    }

    //! Ostream operator for Point3D
    friend std::ostream& operator<<( std::ostream& os,
                                     const Point3D<Scalar>& p ) {
        os << "Point3D(" << p.homogeneous().format(CommaInitFmt) << ")";
        return os;
    }
};

//! Vertex object for use with shading
template <typename Scalar>
struct Vertex3D {
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Observation type
    typedef Scalar Colour;
    //! Vertex dimension
    static const int dim = Point::dim + Vector::dim + 2;

    //! Convenience constructor
    Vertex3D(Point p, Vector n, Scalar a, Scalar d) :
        position(p), normal(n), ambient(a), diffuse(d) { }

    Point position; //!< Vertex position
    Vector normal;  //!< Vertex surface normal
    Colour ambient; //!< Ambient reflectance
    Colour diffuse; //!< Diffuse reflectance

    //! Ostream operator for vertices
    friend std::ostream& operator<<( std::ostream& os,
                                     const Vertex3D<Scalar>& v ) {
        os << "Vertex" << std::endl
           << "Position: " << std::endl
           << v.position << std::endl
           << "Normal: " << std::endl
           << v.normal << std::endl
           << "Ambient reflectance: " << v.ambient << std::endl
           << "Diffuse reflectance: " << v.diffuse << std::endl;
        return os;
    }
};

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
    inline
    SO3Group operator=( const SO3Group& other ) {
        mat_ = other.matrix();
        return *this;
    }

    //! Return the transformation matrix
    inline
    const TransformationMatrix matrix() const { return mat_; }

    //! Return the inverse transformation
    inline
    const SO3Group inverse() const {
        return SO3Group(matrix().transpose());
    }

    //! Transform a 3D homogeneous quantity.
    /*!
        Transform a 3D homogeneous quantity and, if requested, compute the
        Jacobian of the transformed point w.r.t. a perturbation in the
        transformation parameters
    */
    inline
    const Homogeneous transform(
        const Homogeneous& h,
        TransformedPointJacobian* jacobian_ptr = nullptr ) const {

        Homogeneous h_transformed(matrix() * h.cartesian(), h.scale());

        if(jacobian_ptr != nullptr) {
            TransformedPointJacobian& jacobian = *jacobian_ptr;
            jacobian = wedge(-(matrix() * h.cartesian()));
        }

        return h_transformed;
    }

    //! Multiplication operator for two group elements
    inline
    const SO3Group operator*( const SO3Group& other ) const {
        SO3Group result(matrix() * other.matrix());
        return result;
    }
    //! Multiplication operator for group element and homogeneous coordinate
    inline
    const Homogeneous operator*( const Homogeneous& h ) const {
        return transform(h);
    }
    //! Multiplication operator for group element and vector (preserves type)
    inline
    const Vector operator*( const Vector& v ) const {
        return static_cast<Vector>(transform(v));
    }
    //! Multiplication operator for group element and point (preserves type)
    inline
    const Point operator*( const Point& p ) const {
        return static_cast<Point>(transform(p));
    }

    //! SO(3) wedge operator as defined by Barfoot
    /*!
        This is the inverse operator to SO3::vee.
    */
    inline
    static const TransformationMatrix wedge( const TangentVector& phi ) {
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
    inline
    static const TangentVector vee( const TransformationMatrix& Phi ) {
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
    inline
    static const SO3Group exp( const TangentVector& phi ) {
        Scalar angle = phi.norm();
        TangentVector axis = phi / angle;

        // Special case for phi == 0 (identity)
        if(angle <= std::numeric_limits<Scalar>::epsilon()) {
            return SO3Group(TransformationMatrix::Identity());
        }

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
    inline
    static const TangentVector log( const SO3Group& C ) {
        // Get the rotation angle from the trace of C
        Scalar angle = acos(Scalar(0.5) * C.matrix().trace() - Scalar(0.5));

        // Special case if angle is zero
        if(angle <= std::numeric_limits<Scalar>::epsilon()) {
            return TangentVector(Scalar(0),Scalar(0),Scalar(0));
        }

        // Compute the normalized axis
        TangentVector axis;
        axis(0) = C.matrix()(2,1) - C.matrix()(1,2);
        axis(1) = C.matrix()(0,2) - C.matrix()(2,0);
        axis(2) = C.matrix()(1,0) - C.matrix()(0,1);
        axis /= (Scalar(2) * sin(angle));

        return angle * axis;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << matrix().format(CommaInitFmt);
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
    inline
    const SO3 rotation() const { return rotation_; }

    //! Return the translation vector
    inline
    const Vector translation() const { return translation_; }

    //! Return the transformation matrix
    inline
    const TransformationMatrix matrix() const {
        TransformationMatrix mat;
        mat.block(0,0,3,3) = rotation().matrix();
        mat.block(0,3,3,1) = translation().cartesian();
        mat.bottomRows(1) << Scalar(0), Scalar(0), Scalar(0), Scalar(1);
        return mat;
    }

    //! Return the inverse transformation
    inline
    const SE3Group inverse() const {
        TransformationMatrix inv;
        inv.block(0,0,3,3) = rotation().inverse().matrix();
        inv.block(0,3,3,1) =
            -(rotation().inverse() * translation()).cartesian();
        inv.bottomRows(1) << Scalar(0), Scalar(0), Scalar(0), Scalar(1);
        return SE3Group(inv);
    }

    //! Assignment operator
    inline
    SE3Group operator=( const SE3Group& other ) {
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
    inline
    const Homogeneous transform(
        const Homogeneous& h,
        TransformedPointJacobian* jacobian_ptr = nullptr ) const {

        Homogeneous h_transformed;

        if(jacobian_ptr != nullptr) {
            typename SO3::TransformedPointJacobian rot_jacobian;
            Homogeneous h_rotated =
                rotation().transform(h, &rot_jacobian);

            h_transformed = h_rotated + h.scale() * translation();

            TransformedPointJacobian& jacobian = *jacobian_ptr;
            jacobian.block(0,0,3,3) =
                h.scale() * SO3::TransformationMatrix::Identity();
            jacobian.block(0,3,3,3) = rot_jacobian;
        }
        else {
            h_transformed = rotation() * h + h.scale() * translation();
        }

        return h_transformed;
    }

    //! Multiplication operator for two group elements
    inline
    const SE3Group operator*( const SE3Group& other ) const {
        return SE3Group( rotation() * other.translation() + translation(),
                         rotation() * other.rotation() );
    }

    //! Multiplication operator for group element and homogeneous coordinate
    inline
    const Homogeneous operator*( const Homogeneous& h ) const {
        return transform(h);
    }
    //! Multiplication operator for group element and vector (preserves type)
    inline
    const Vector operator*( const Vector& v ) const {
        return static_cast<Vector>(transform(v));
    }
    //! Multiplication operator for group element and point (preserves type)
    inline
    const Point operator*( const Point& p ) const {
        return static_cast<Point>(transform(p));
    }

    //! SE(3) vee operator as defined by Barfoot
    /*!
        This is the inverse operator to SE3::vee.
    */
    inline
    static const TransformationMatrix wedge( const TangentVector& xi ) {
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
    inline
    static const TangentVector vee( const TransformationMatrix& Xi ) {
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
    inline
    static const SE3Group exp( const TangentVector& xi ) {
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
    inline
    static const TangentVector log( const SE3Group& T ) {
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

#endif // CERES_SLAM_GEOMETRY_H_
