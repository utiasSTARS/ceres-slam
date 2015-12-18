#ifndef CERES_SLAM_GEOMETRY_HOMOGENEOUS3D_H_
#define CERES_SLAM_GEOMETRY_HOMOGENEOUS3D_H_

#include <memory>
#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>

#include <ceres_slam/utils.h>

namespace ceres_slam {

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
    inline const Cartesian cartesian() const {
        // Special case for homogeneous vectors (scale = 0)
        if( abs<Scalar>(scale_) <= std::numeric_limits<Scalar>::epsilon() ){
            return cartesian_;
        }
        else {
            return cartesian_ / scale_;
        }
    }
    //! Return the homogeneous scale part of the point/vector
    inline const Scalar scale() const { return scale_; }
    //! Return the homogeneous form of the point
    inline const Homogeneous homogeneous() const {
        Homogeneous h;
        h.head(3) = cartesian();
        h(3) = scale();
        return h;
    }
    //! Return a const pointer to the underlying array for the cartesian part
    inline const Scalar* data() const { return cartesian_.data(); }
    //! Return a pointer to the underlying array for the cartesian part
    inline Scalar* data() { return cartesian_.data(); }

    //! Accessor operator, mutable
    inline Scalar& operator()(int i) { return cartesian_(i); }
    //! Accessor operator, const
    inline const Scalar& operator()(int i) const { return cartesian_(i); }

    //! Addition operator for homogeneous coordinates
    inline
    const Homogeneous3D operator+( const Homogeneous3D<Scalar>& other ) const {
        return Homogeneous3D( this->cartesian() + other.cartesian(),
                              this->scale() + other.scale() );
    }
    //! Subtraction operator for homogeneous coordinates
    inline
    const Homogeneous3D operator-( const Homogeneous3D<Scalar>& other ) const {
        return Homogeneous3D( this->cartesian() - other.cartesian(),
                              this->scale() - other.scale() );
    }
    //! Right-multiplication of homogeneous coordinate by scalar
    inline const Homogeneous3D operator*( const Scalar s ) const {
        return Homogeneous3D( s * this->cartesian(), s * this->scale() );
    }
    //! Left-multiplication of homogeneous coordinate by scalar
    /*!
        Needs to be a friend inside the class to keep the templating
        and avoid passing 'this' as the first argument.
    */
    inline friend const Homogeneous3D<Scalar> operator*(
        const Scalar s, const Homogeneous3D<Scalar>& h ) {
        return h * s;
    }
    //! Unary minus operator for homogeneous coordinates
    inline const Homogeneous3D operator-() const {
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
           << h.homogeneous().format(CommaInitFmt)
           << ")";
        return os;
    }

protected:
    //! Cartesian part
    Cartesian cartesian_;
    //! Homogeneous scale part
    Scalar scale_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_GEOMETRY_HOMOGENEOUS3D_H_
