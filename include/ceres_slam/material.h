#ifndef CERES_SLAM_MATERIAL_H_
#define CERES_SLAM_MATERIAL_H_

#include <memory>
#include <Eigen/Core>

namespace ceres_slam {

//! Material definition with Phong reflectance
template <typename Scalar>
class Material {
public:
    //! Pointer type
    typedef std::shared_ptr<Material> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const Material> ConstPtr;
    //! Number of parameters
    static const int dim = 4;
    //! Colour type
    typedef Scalar Colour;
    //! Phong illumination parameters
    typedef Eigen::Matrix<Scalar, 1, dim, Eigen::RowMajor> PhongParams;

    //! Default constructor
    Material() : Material(PhongParams::Zero()) { }
    //! Construct from Phong parameter matrix
    Material(PhongParams phong_params) : phong_params_(phong_params) { }

    //! Return the Phong parameter matrix (mutable)
    inline PhongParams& phong_params() { return phong_params_; }
    //! Return the Phong parameter matrix (const)
    inline const PhongParams& phong_params() const { return phong_params_; }

    //! Return the ambient component of the vertex reflectance (mutable)
    inline Colour& ambient() { return phong_params_(0); }
    //! Return the ambient component of the vertex reflectance (mutable)
    inline const Colour& ambient() const { return phong_params_(0); }

    //! Return the diffuse component of the vertex reflectance (mutable)
    inline Colour& diffuse() { return phong_params_(1); }
    //! Return the diffuse component of the vertex reflectance (const)
    inline const Colour& diffuse() const { return phong_params_(1); }

    //! Return the specular component of the vertex reflectance (mutable)
    inline Colour& specular() { return phong_params_(2); }
    //! Return the specular component of the vertex reflectance (mutable)
    inline const Colour& specular() const { return phong_params_(2); }

    //! Return the exponent component of the vertex reflectance (mutable)
    inline Colour& exponent() { return phong_params_(3); }
    //! Return the exponent component of the vertex reflectance (const)
    inline const Colour& exponent() const { return phong_params_(3); }

    //! Ostream operator for materials
    friend std::ostream& operator<<( std::ostream& os,
                                     const Material<Scalar>& m ) {
        os << "Material" << std::endl
           << "Phong parameters: " << std::endl
           << m.phong_params();
        return os;
    }

private:
    //! Phong illumination parameters stored column-wise in a matrix.
    /*!
        phong_params_.col(0) is ambient,  col(1) is diffuse,
        col(2) is specular, col(3) is shininess.
    */
    PhongParams phong_params_;
};

} // namespace ceres_slam

#endif // CERES_SLAM_MATERIAL_H_
