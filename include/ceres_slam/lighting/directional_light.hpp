#ifndef CERES_SLAM_DIRECTIONAL_LIGHT_H_
#define CERES_SLAM_DIRECTIONAL_LIGHT_H_

#include <Eigen/Core>
#include <memory>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/utils/utils.hpp>

#include "phong.hpp"
#include "vertex3d.hpp"

namespace ceres_slam {

//! A directional light source model using Phong lighting
template <typename Scalar>
class DirectionalLight {
   public:
    //! Pointer type
    typedef std::shared_ptr<DirectionalLight> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const DirectionalLight> ConstPtr;
    //! Dimension of the observation (greyscale intensity only for now)
    static const int obs_dim = 1;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Shadable vertex type
    typedef Vertex3D<Scalar> Vertex;
    //! Lighting model type
    typedef PhongModel<Scalar> LightingModel;
    //! Observation type
    typedef Scalar Colour;
    //! Observation variance type
    typedef Scalar ColourVariance;
    //! Observation covariance matrix type
    typedef Scalar ColourCovariance;
    //! Observation Jacobian type
    typedef Eigen::Matrix<Scalar, obs_dim, Vertex::dim + Point::dim,
                          Eigen::RowMajor>
        ColourJacobian;

    //! Default constructor
    DirectionalLight()
        : direction_(Vector::Constant(static_cast<Scalar>(1))),
          colour_(static_cast<Scalar>(1)) {
        direction_.normalize();
    }
    //! Construct from direction and colour
    DirectionalLight(Vector& direction, Colour& colour)
        : direction_(direction), colour_(colour) {
        direction_.normalize();
    }

    //! Return the direction of the light(mutable)
    inline Vector& direction() { return direction_; }
    //! Return the direction of the light (const)
    inline const Vector& direction() const { return direction_; }

    //! Return the colour of the light (mutable)
    inline Colour& colour() { return colour_; }
    //! Return the colour of the light (const)
    inline const Colour& colour() const { return colour_; }

    //! Shade a 3D vertex using Phong lighting
    /*!
        NOTE: The vertex must be expressed in the same
        frame as the light direction!

        The Jacobian is 1 x 11 for each colour channel and is organized
        [ d(I)/d(pj)  d(I)/d(nj)  d(I)/d(ka)  d(I)/d(kd) ...
            d(I)/d(ks) d(I)/d(alpha) d(I)/d(pL) ]
        where pj: map point direction
              nj: map point normal vector
              ka: map point ambient reflectance
              kd: map point diffuse reflectance
              ks: map point specular reflectance
              alpha: map point specular exponent (shininess/hardness)
              pL: light source direction
    */
    const Colour shade(const Vertex& vertex, const Vector& camera_position,
                       ColourJacobian* jacobian_ptr = nullptr) const {
        // Direction to camera
        Vector camera_vec = camera_position - vertex.position();
        Vector camera_dir = camera_vec;
        camera_dir.normalize();

        return LightingModel::shade(vertex, this->direction(), camera_dir,
                                    this->colour());
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->direction().str() << "," << this->colour();
        return ss.str();
    }

    //! Ostream operator for DirectionalLight
    friend std::ostream& operator<<(std::ostream& os,
                                    const DirectionalLight<Scalar>& l) {
        os << "Directional light source" << std::endl
           << "Direction: " << l.direction() << std::endl
           << "Colour: " << l.colour();
        return os;
    }

   private:
    //! Direction of the light
    Vector direction_;
    //! Light colour (intensity)
    Colour colour_;
};

}  // namespace ceres_slam

#endif  // CERES_SLAM_DIRECTIONAL_LIGHT_H_
