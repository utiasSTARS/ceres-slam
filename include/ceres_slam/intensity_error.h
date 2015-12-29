#ifndef CERES_SLAM_INTENSITY_ERROR_H_
#define CERES_SLAM_INTENSITY_ERROR_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry.h>
#include <ceres_slam/point_light.h>
#include <ceres_slam/material.h>

namespace ceres_slam {

//! Intensity error cost function for Ceres with automatic Jacobians
class IntensityErrorAutomatic {
public:
    //! Light source type
    typedef PointLight<double> Light;
    //! SO(3) type
    typedef SO3Group<double> SO3;
    //! SE(3) type
    typedef SE3Group<double> SE3;
    //! Point type
    typedef Point3D<double> Point;
    //! Vector type
    typedef Vector3D<double> Vector;

    //! Constructor with fixed model parameters
    IntensityErrorAutomatic(const Light::Colour& colour,
                            const Light::ColourCovariance& stiffness) :
        colour_(colour),
        stiffness_(stiffness) { }

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T* const xi_c_g_ceres,
                    const T* const pt_g_ceres,
                    const T* const normal_g_ceres,
                    const T* const phong_params_ceres,
                    const T* const lightpos_g_ceres,
                    T* residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef typename SE3T::TangentVector TangentVectorT;
        typedef Point3D<T> PointT;
        typedef Vector3D<T> VectorT;
        typedef Vertex3D<T> VertexT;
        typedef PointLight<T> LightT;
        typedef typename LightT::Colour ColourT;
        typedef Material<T> MaterialT;
        typedef typename MaterialT::PhongParams PhongParamsT;

        // Camera pose in the global frame
        Eigen::Map<const TangentVectorT> xi_c_g(xi_c_g_ceres);
        SE3T T_c_g = SE3T::exp(xi_c_g);

        // Map point
        PointT pt_g(pt_g_ceres);                // In the global frame
        PointT pt_c = T_c_g * pt_g;             // In the camera frame

        // Normal vector at the map point
        VectorT normal_g(normal_g_ceres);       // In the global frame
        VectorT normal_c = T_c_g * normal_g;    // In the camera frame
        normal_c.normalize();

        // Phong reflectance coefficients (ambient and diffuse only for now)
        Eigen::Map<const PhongParamsT> phong_params(phong_params_ceres);
        typename MaterialT::Ptr material = std::make_shared<MaterialT>(phong_params);

        // Vertex in the camera frame for shading
        VertexT vertex_c(pt_c, normal_c, material);

        // Light source position
        PointT lightpos_g(lightpos_g_ceres);    // In the global frame
        PointT lightpos_c = T_c_g * lightpos_g; // In the camera frame

        // Light source model
        LightT light(lightpos_c, PhongParamsT::Constant(T(1)));

        // Compute the predicted intensity at the vertex
        ColourT predicted_colour = light.shade(vertex_c);

        // Compute the residuals
        // (no need to map to an eigen matrix yet since it's only 1D)
        residuals_ceres[0] = T(stiffness_) * (predicted_colour - T(colour_));
        // std::cout << residuals_ceres[0] << std::endl;

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction* Create(
                                    const Light::Colour& colour,
                                    const Light::ColourCovariance& stiffness) {
        return( new ceres::AutoDiffCostFunction
                    <IntensityErrorAutomatic,
                                           1,  // Residual dimension
                                           6,  // Vehicle pose vector
                                           3,  // Map point position
                                           3,  // Map point normal
                                           2,  // Map point Phong parameters
                                           3>  // Light source position
                       (new IntensityErrorAutomatic(colour, stiffness))
              );
    }

private:
    //! Intensity observation
    Light::Colour colour_;
    //! Intensity stiffness matrix (inverse sqrt of covariance matrix)
    Light::ColourCovariance stiffness_;

}; // class IntensityErrorAutomatic

} // namespace ceres_slam

#endif // CERES_SLAM_INTENSITY_ERROR_H_
