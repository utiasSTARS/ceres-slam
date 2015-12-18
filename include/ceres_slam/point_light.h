#ifndef CERES_SLAM_POINT_LIGHT_H_
#define CERES_SLAM_POINT_LIGHT_H_

#include <memory>
#include <Eigen/Core>

#include <ceres_slam/geometry.h>
#include <ceres_slam/utils.h>

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
    //! Phong illumination parameters
    typedef Eigen::Matrix<Scalar, obs_dim, 2, Eigen::RowMajor> PhongParams;

    //! Default constructor
    PointLight() : PointLight( Point(), PhongParams::Zero() ) { }
    //! Construct from position and phong parameters
    PointLight(Point position, PhongParams phong_params) :
        position_(position),
        phong_params_(phong_params) { }

    //! Return the position of the light(mutable)
    inline Point& position() { return position_; }
    //! Return the position of the light (const)
    inline const Point& position() const { return position_; }

    //! Return the Phong parameter matrix (mutable)
    inline PhongParams& phong_params() { return phong_params_; }
    //! Return the Phong parameter matrix (const)
    inline const PhongParams& phong_params() const { return phong_params_; }

    //! Return the ambient component of the light (mutable)
    inline Colour& ambient() { return phong_params_(0); }
    //! Return the ambient component of the light (mutable)
    inline const Colour& ambient() const { return phong_params_(0); }

    //! Return the diffuse component of the light (mutable)
    inline Colour& diffuse() { return phong_params_(1); }
    //! Return the diffuse component of the light (const)
    inline const Colour& diffuse() const { return phong_params_(1); }

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
        Vector light_vec = this->position() - vertex.position();
        // Unit vector in direction of light from point
        Vector light_dir = light_vec;
        light_dir.normalize();
        // Dot product of light direction and surface normal
        Scalar light_dir_dot_normal = light_dir.dot(vertex.normal());

        Colour col = this->ambient() * vertex.ambient()
                   + this->diffuse() * vertex.diffuse() *
                        fmax<Scalar>(Scalar(0), light_dir_dot_normal);

        if(jacobian_ptr != nullptr) {
            ColourJacobian& jacobian = *jacobian_ptr;
            jacobian = ColourJacobian::Zero();

            Scalar light_norm = light_vec.norm();
            Scalar two_light_norm2 = Scalar(2) * light_norm * light_norm;
            Scalar one_over_two_light_norm3 =
                Scalar(1) / (two_light_norm2 * light_norm);

            // d(I)/d(pj) = d(I)/d(l) d(l)/d(pj)
            if(light_dir_dot_normal >= 0.) {
                jacobian.block(0,0,1,3) =
                    // d(I)/d(l) (1 x 3)
                    this->diffuse() * vertex.diffuse() *
                        vertex.normal().cartesian().transpose()
                    // d(l)/d(pj) (3 x 3)
                    * (-one_over_two_light_norm3)
                        * ( two_light_norm2
                            * Eigen::Matrix<Scalar, 3, 3,
                                            Eigen::RowMajor>::Identity()
                            - light_vec.cartesian() *
                                light_vec.cartesian().transpose() );
            } // else zeros

            // d(I)/d(nj)
            if(light_dir_dot_normal >= Scalar(0)) {
                jacobian.block(0,3,1,3) = this->diffuse() * vertex.diffuse()
                                        * light_dir.cartesian().transpose();
            } // else zeros

            // d(I)/d(ka)
            jacobian(0,6) = Scalar(1);

            // d(I)/d(kd)
            jacobian(0,7) = fmax<Scalar>(Scalar(0), light_dir_dot_normal);

            // d(I)/d(pL)
            jacobian.block(0,8,1,3) = -jacobian.block(0,0,1,3);
        }

        return col;
    }

    //! Convert to a string
    inline const std::string str() const {
        std::stringstream ss;
        ss << this->position().format(CommaInitFmt) << ","
           << this->phong_params().format(CommaInitFmt);
        return ss.str();
    }

    //! Ostream operator for PointLight
    friend std::ostream& operator<<( std::ostream& os,
                                     const PointLight<Scalar>& p ) {
        os << "Point light source" << std::endl
           << "Position: " << p.position() << std::endl
           << "Phong parameters: " << std::endl
           << p.phong_params() << std::endl;
        return os;
     }

private:
    //! Position of the light
    Point position_;
    //! Phong illumination parameters stored column-wise in a matrix.
    /*!
        col(0) is ambient, col(1) is diffuse,
        col(2) will be specular, col(3) will be shininess.
    */
    PhongParams phong_params_;
};



} // namespace ceres_slam

#endif // CERES_SLAM_POINT_LIGHT_H_
