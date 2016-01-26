#ifndef CERES_SLAM_POINT_LIGHT_H_
#define CERES_SLAM_POINT_LIGHT_H_

#include <memory>
#include <Eigen/Core>

#include <ceres_slam/utils.h>
#include <ceres_slam/geometry.h>
#include <ceres_slam/vertex3d.h>

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
    PointLight() :
        position_(Point() ),
        colour_(static_cast<Scalar>(1) ) { }
    //! Construct from position and colour
    PointLight(Point& position, Colour& colour) :
        position_(position),
        colour_(colour) { }

    //! Return the position of the light(mutable)
    inline Point& position() { return position_; }
    //! Return the position of the light (const)
    inline const Point& position() const { return position_; }

    //! Return the colour of the light (mutable)
    inline Colour& colour() { return colour_; }
    //! Return the colour of the light (const)
    inline const Colour& colour() const { return colour_; }

    //! Shade a 3D vertex using Phong lighting (without specularities)
    /*!
        NOTE: The vertex must be expressed in the same
        frame as the light position!

        The Jacobian is 1 x 11 for each colour channel and is organized
        [ d(I)/d(pj)  d(I)/d(nj)  d(I)/d(ka)  d(I)/d(kd) ...
            d(I)/d(ks) d(I)/d(alpha) d(I)/d(pL) ]
        where pj: map point position
              nj: map point normal vector
              ka: map point ambient reflectance
              kd: map point diffuse reflectance
              ks: map point specular reflectance
              alpha: map point specular exponent (shininess/hardness)
              pL: light source position
    */
    const Colour shade( const Vertex& vertex,
                        const Vector& camera_position,
                        ColourJacobian* jacobian_ptr=nullptr ) const {
        // Ambient component
        // -----------------
        Colour ambient = vertex.material()->ambient();


        // Diffuse component
        // -----------------
        Colour diffuse = Colour(static_cast<Scalar>(0) );

        // Direction to light
        Vector light_vec = this->position() - vertex.position();
        Vector light_dir = light_vec;
        light_dir.normalize();

        // Dot product of light direction and surface normal
        Scalar light_dir_dot_normal = static_cast<Scalar>(0);

        // Check for NaN -- pathological case where light_vec.norm() == 0
        if(light_dir.allFinite() ) {
            light_dir_dot_normal = light_dir.dot(vertex.normal());

            diffuse = fmax(static_cast<Scalar>(0),
                vertex.material()->diffuse() * light_dir_dot_normal);
        }


        // Specular component (Blinn-Phong)
        // --------------------------------
        Colour specular = Colour(static_cast<Scalar>(0) );

        // Direction to camera
        Vector camera_vec = camera_position - vertex.position();
        Vector camera_dir = camera_vec;
        camera_dir.normalize();
        // Halfway direction
        Vector halfway_dir = light_dir + camera_dir;
        halfway_dir.normalize();

        // Dot product of halfway direction and normal vector
        Scalar halfway_dir_dot_normal = static_cast<Scalar>(0);
        Scalar pow_halfway_dir_dot_normal_exponent = static_cast<Scalar>(0);

        // Check for NaN -- pathological case where halfway_dir.norm() == 0
        if(halfway_dir.allFinite() ) {
            halfway_dir_dot_normal = halfway_dir.dot(vertex.normal() );
            pow_halfway_dir_dot_normal_exponent =
                pow(halfway_dir_dot_normal, vertex.material()->exponent() );

            specular = fmax(static_cast<Scalar>(0),
                            vertex.material()->specular()
                                * pow_halfway_dir_dot_normal_exponent);
        }

        // Total intensity
        // std::cout << "ambient: " << ambient << std::endl;
        // std::cout << "diffuse: " << diffuse << std::endl;
        // std::cout << "specular: " << specular << std::endl;
        Colour col = this->colour() * (ambient + diffuse + specular);

        // Enforce intensities in [0,1]
        clamp(col);

        // Compute jacobian if needed
        if(jacobian_ptr != nullptr) {
            ColourJacobian& jacobian = *jacobian_ptr;
            jacobian = ColourJacobian::Zero();

            Scalar light_norm = light_vec.norm();
            Scalar two_light_norm2 = static_cast<Scalar>(2)
                                        * light_norm * light_norm;
            Scalar one_over_two_light_norm3 =
                static_cast<Scalar>(1) / (two_light_norm2 * light_norm);

            // d(I)/d(pj) = d(I)/d(l) d(l)/d(pj)
            if(light_dir_dot_normal >= 0.) {
                jacobian.block(0,0,1,3) =
                    // d(I)/d(l) (1 x 3)
                    vertex.material()->diffuse() * vertex.normal().transpose()
                    // d(l)/d(pj) (3 x 3)
                    * (-one_over_two_light_norm3)
                        * ( two_light_norm2
                            * Eigen::Matrix<Scalar, 3, 3,
                                            Eigen::RowMajor>::Identity()
                            - light_vec * light_vec.transpose() );
            } // else zeros

            // d(I)/d(nj)
            if(light_dir_dot_normal >= static_cast<Scalar>(0)) {
                jacobian.block(0,3,1,3) = vertex.material()->diffuse()
                                            * light_dir.transpose();
            } // else zeros

            // d(I)/d(ka)
            jacobian(0,6) = static_cast<Scalar>(1);

            // d(I)/d(kd)
            jacobian(0,7) = fmax(static_cast<Scalar>(0), light_dir_dot_normal);

            // d(I)/d(ks)
            jacobian(0,8) = fmax(static_cast<Scalar>(0),
                                 pow_halfway_dir_dot_normal_exponent);

            // d(I)/d(alpha) = i_s k_s alpha (h.n)^(alpha-1)
            jacobian(0,9) = fmax(static_cast<Scalar>(0),
                vertex.material()->specular()
                    * vertex.material()->exponent()
                    * pow(halfway_dir_dot_normal,
                          vertex.material()->exponent()
                          - static_cast<Scalar>(1) ) );

            // d(I)/d(pL)
            jacobian.block(0,10,1,3) = -jacobian.block(0,0,1,3);

            // Constant factor for light colour
            jacobian *= this->colour();
        }

        return col;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->position().str() << ","
           << this->colour();
        return ss.str();
    }

    //! Ostream operator for PointLight
    friend std::ostream& operator<<( std::ostream& os,
                                     const PointLight<Scalar>& p ) {
        os << "Point light source" << std::endl
           << "Position: " << p.position() << std::endl
           << "Colour: " << p.colour();
        return os;
     }

private:
    //! Position of the light
    Point position_;
    //! Light colour (intensity)
    Colour colour_;

    //! Clamp intensity values to lie in [0,1]
    void clamp(Colour& col) const {
        col = fmax(Colour(static_cast<Scalar>(0) ), col);
        col = fmin(Colour(static_cast<Scalar>(1) ), col);
    }
};



} // namespace ceres_slam

#endif // CERES_SLAM_POINT_LIGHT_H_
