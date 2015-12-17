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
    typedef std::shared_ptr<const PointLight> ConstPtr;
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
    typedef Eigen::Matrix<Scalar, obs_dim,
        Vertex::dim + Point::dim, Eigen::RowMajor> ColourJacobian;

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

    //! Shade a 3D vertex using Phong lighting (without specularities)
    /*!
        NOTE: The vertex must be expressed in the same
        frame as the light position!

        The Jacobian is 1 x 11 for each colour channel and is organized
        [d(I)/d(pj)  d(I)/d(nj)  d(I)/d(ka)  d(I)/d(kd)  d(I)/d(pL)]
        where pj: map point position
              nj: map point normal vector
              ka: map point ambient reflectance
              kd: map point diffuse reflectance
              pL: light source position
    */
    const Colour shade( const Vertex& vertex,
                        ColourJacobian* jacobian_ptr=nullptr ) const {
        // Vector from point to light
        Vector light_vec = position() - vertex.position;
        // Unit vector in direction of light from point
        Vector light_dir = light_vec;
        light_dir.normalize();
        // Dot product of light direction and surface normal
        Scalar light_dir_dot_normal = light_dir.dot(vertex.normal);

        Colour col = ambient() * vertex.ambient
                   + diffuse() * vertex.diffuse *
                        fmax(0., light_dir_dot_normal);

        if(jacobian_ptr != nullptr) {
            ColourJacobian& jacobian = *jacobian_ptr;
            jacobian = ColourJacobian::Zero();

            Scalar light_norm = light_vec.norm();
            Scalar two_light_norm2 = 2. * light_norm * light_norm;
            Scalar one_over_two_light_norm3 =
                1. / (two_light_norm2 * light_norm);

            // d(I)/d(pj) = d(I)/d(l) d(l)/d(pj)
            if(light_dir_dot_normal >= 0.) {
                jacobian.block(0,0,1,3) =
                    // d(I)/d(l) (1 x 3)
                    diffuse() * vertex.diffuse *
                        vertex.normal.cartesian().transpose()
                    // d(l)/d(pj) (3 x 3)
                    * (-one_over_two_light_norm3)
                        * ( two_light_norm2
                            * Eigen::Matrix<Scalar, 3, 3,
                                            Eigen::RowMajor>::Identity()
                            - light_vec.cartesian() *
                                light_vec.cartesian().transpose() );
            } // else zeros

            // d(I)/d(nj)
            if(light_dir_dot_normal >= 0.) {
                jacobian.block(0,3,1,3) = diffuse() * vertex.diffuse
                                        * light_dir.cartesian().transpose();
            } // else zeros

            // d(I)/d(ka)
            jacobian(0,6) = 1.;

            // d(I)/d(kd)
            jacobian(0,7) = fmax(0., light_dir_dot_normal);

            // d(I)/d(pL)
            jacobian.block(0,8,1,3) = -jacobian.block(0,0,1,3);
        }

        return col;
    }

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
