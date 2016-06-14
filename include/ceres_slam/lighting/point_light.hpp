#ifndef CERES_SLAM_POINT_LIGHT_H_
#define CERES_SLAM_POINT_LIGHT_H_

#include <Eigen/Core>
#include <memory>

#include <ceres_slam/geometry/geometry.hpp>
#include <ceres_slam/utils/utils.hpp>

#include "phong.hpp"
#include "vertex3d.hpp"

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
    PointLight() : position_(Point()), colour_(static_cast<Scalar>(1)) {}
    //! Construct from position and colour
    PointLight(Point& position, Colour& colour)
        : position_(position), colour_(colour) {}

    //! Return the position of the light(mutable)
    inline Point& position() { return position_; }
    //! Return the position of the light (const)
    inline const Point& position() const { return position_; }

    //! Return the colour of the light (mutable)
    inline Colour& colour() { return colour_; }
    //! Return the colour of the light (const)
    inline const Colour& colour() const { return colour_; }

    //! Shade a 3D vertex using Phong lighting
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
    const Colour shade(const Vertex& vertex, const Vector& camera_position,
                       ColourJacobian* jacobian_ptr = nullptr) const {
        // Direction to light
        Vector light_vec = this->position() - vertex.position();
        Vector light_dir = light_vec;
        light_dir.normalize();

        // Direction to camera
        Vector camera_vec = camera_position - vertex.position();
        Vector camera_dir = camera_vec;
        camera_dir.normalize();

        return LightingModel::shade(vertex, light_dir, camera_dir,
                                    this->colour());
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->position().str() << "," << this->colour();
        return ss.str();
    }

    //! Ostream operator for PointLight
    friend std::ostream& operator<<(std::ostream& os,
                                    const PointLight<Scalar>& l) {
        os << "Point light source" << std::endl
           << "Position: " << l.position() << std::endl
           << "Colour: " << l.colour();
        return os;
    }

   private:
    //! Position of the light
    Point position_;
    //! Light colour (intensity)
    Colour colour_;
};

}  // namespace ceres_slam

#endif  // CERES_SLAM_POINT_LIGHT_H_
