#ifndef CERES_SLAM_POINT_LIGHT_H_
#define CERES_SLAM_POINT_LIGHT_H_

#include <memory>
#include <Eigen/Core>

#include <ceres_slam/geometry.h>

namespace ceres_slam {

//! A point light source model using Phong lighting
template <typename Scalar>
class PointLight {
public:
    //! Pointer type
    typedef std::shared_ptr<PointLight> Ptr;
    //! Const pointer type
    typedef const std::shared_ptr<PointLight> ConstPtr;
    //! Dimension of the observation (greyscale intensity only for now)
    static const int obs_dim = 1;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Shadable vertex type
    typedef Vertex3D<Scalar> Vertex;
    //! Observation type
    typedef Scalar Colour;
    //! Observation variance type
    typedef Scalar ColourVariance;
    //! Observation covariance matrix type
    typedef Scalar ColourCovariance;
    //! Observation Jacobian type
    typedef Eigen::Matrix<Scalar, obs_dim, Point::dim, Eigen::RowMajor>
        ColourJacobian;

    //! Default constructor
    PointLight( const Point& position ) :
        position_(position), ambient_(1.), diffuse_(1.) { }

    //! Return the light position
    const Point position() const { return position_; }
    //! Return the ambient colour
    const Colour ambient() const { return ambient_; }
    //! Return the diffuse colour
    const Colour diffuse() const { return diffuse_; }
    // TODO: Add specular components

    //! Shade a 3D vertex.
    /*!
        NOTE: The vertex must be expressed in the same
        frame as the light position!
    */
    inline const Colour shade( const Vertex& vertex ) {
        // Vector from point to light
        Vector light_dir = position() - vertex.position;
        light_dir.normalize();

        return shade_ambient(vertex) + shade_diffuse(vertex, light_dir);
    }
    //! Ambient component of Phong lighting
    inline const Colour shade_ambient( const Vertex& vertex ) {
        return ambient() * vertex.ambient;
    }
    //! Diffuse component of Phong lighting
    inline const Colour shade_diffuse( const Vertex& vertex,
                                       const Vector& light_dir ) {
        return fmax(0., light_dir.dot(vertex.normal));
    }
    // TODO: Specular component of Phong lighting

    //! Ostream operator for PointLight
    friend std::ostream& operator<<( std::ostream& os,
                                     const PointLight<Scalar>& p ) {
        os << "Point light source" << std::endl
           << "Position: " << std::endl << p.position() << std::endl
           << "Ambient colour: "  << p.ambient() << std::endl
           << "Diffuse colour: "  << p.diffuse() << std::endl;
        return os;
     }

private:
    //! Position of the light
    Point position_;
    //! Ambient colour. This is just the greyscale intensity for now.
    Colour ambient_;
    //! Diffuse colour. This is just the greyscale intensity for now.
    Colour diffuse_;
    // TODO: Add specular components
};

} // namespace ceres_slam

#endif // CERES_SLAM_POINT_LIGHT_H_
