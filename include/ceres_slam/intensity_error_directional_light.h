#ifndef CERES_SLAM_INTENSITY_ERROR_DIRECTIONAL_LIGHT_H_
#define CERES_SLAM_INTENSITY_ERROR_DIRECTIONAL_LIGHT_H_

#include <ceres/ceres.h>

#include <ceres_slam/geometry/geometry.h>
#include <ceres_slam/lighting/lighting.h>
#include <ceres_slam/utils/utils.h>

namespace ceres_slam {

//! Intensity error cost function for Ceres with automatic Jacobians
class IntensityErrorDirectionalLightAutomatic {
   public:
    //! Light source type
    typedef DirectionalLight<double> Light;

    //! Constructor with fixed model parameters
    IntensityErrorDirectionalLightAutomatic(
        const Light::Colour &colour, const Light::ColourCovariance &stiffness)
        : colour_(colour), stiffness_(stiffness) {}

    //! Templated evaluator operator for use with ceres::Jet
    template <typename T>
    bool operator()(const T *const T_c_g_ceres, const T *const pt_g_ceres,
                    const T *const normal_g_ceres,
                    const T *const phong_params_ceres,
                    const T *const texture_ceres,
                    const T *const lightdir_g_ceres, T *residuals_ceres) const {
        // Local typedefs for convenience
        typedef SE3Group<T> SE3T;
        typedef Point3D<T> PointT;
        typedef Vector3D<T> VectorT;
        typedef Vertex3D<T> VertexT;
        typedef DirectionalLight<T> LightT;
        typedef typename LightT::Colour ColourT;
        typedef Material<T> MaterialT;
        typedef typename MaterialT::PhongParams PhongParamsT;
        typedef Texture<T> TextureT;

        // Camera pose in the global frame
        Eigen::Map<const SE3T> T_c_g(T_c_g_ceres);
        // std::cout << "T_c_g: " << T_c_g << std::endl;

        // Map point
        Eigen::Map<const PointT> pt_g(pt_g_ceres);  // Global frame
        PointT pt_c = T_c_g * pt_g;                 // Camera frame
        // std::cout << "pt_g: " << pt_g << std::endl;
        // std::cout << "pt_c: " << pt_c << std::endl;

        // Normal vector at the map point
        Eigen::Map<const VectorT> normal_g(normal_g_ceres);  // Global frame
        VectorT normal_c = T_c_g * normal_g;                 // Camera frame

        // std::cout << "normal_g: " << normal_g << std::endl;
        // std::cout << "normal_c: " << normal_c << std::endl;

        // Phong reflectance coefficients and per-pixel diffuse texture
        Eigen::Map<const PhongParamsT> phong_params(phong_params_ceres);
        typename MaterialT::Ptr material =
            std::make_shared<MaterialT>(phong_params);
        // std::cout << "material: " << *material << std::endl;

        // Vertex in the camera frame for shading
        typename TextureT::Ptr texture =
            std::make_shared<TextureT>(*texture_ceres);
        VertexT vertex_c(pt_c, normal_c, material, texture);
        // std::cout << "vertex_c: " << vertex_c << std::endl;

        // Light source direction
        Eigen::Map<const VectorT> lightdir_g(lightdir_g_ceres);  // Global frame
        VectorT lightdir_c = T_c_g * lightdir_g;                 // Camera frame
        // std::cout << "lightdir_g: " << lightdir_g << std::endl;
        // std::cout << "lightdir_c: " << lightdir_c << std::endl;

        // Light source model
        ColourT light_colour = static_cast<ColourT>(1);
        LightT light(lightdir_c, light_colour);
        // std::cout << "light: " << light << std::endl;

        // Compute the predicted intensity at the vertex
        // NOTE: camera position in camera frame is the origin
        PointT campos_c = VectorT::Zero();
        ColourT predicted_colour = light.shade(vertex_c, campos_c);

        // Compute the residuals
        // (no need to map to an eigen matrix yet since it's only 1D)
        residuals_ceres[0] = static_cast<T>(stiffness_) *
                             (predicted_colour - static_cast<ColourT>(colour_));

        // std::cout << "predicted_colour: " << predicted_colour << std::endl;
        // std::cout << "colour_: " << colour_ << std::endl;
        // std::cout << "residuals_ceres[0]" << residuals_ceres[0] << std::endl;

        return true;
    }

    //! Factory to hide the construction of the CostFunction object from
    //! the client code.
    static ceres::CostFunction *Create(
        const Light::Colour &colour, const Light::ColourCovariance &stiffness) {
        return (
            new ceres::AutoDiffCostFunction<
                IntensityErrorDirectionalLightAutomatic,
                1,   // Residual dimension
                12,  // Compact SE(3) vehicle pose (3 trans + 9 rot)
                3,   // Map point position
                3,   // Map point normal
                3,   // Map point Phong parameters (ambient + specular)
                1,   // Map point texture (per-pixel diffuse)
                3>   // Light source direction
            (new IntensityErrorDirectionalLightAutomatic(colour, stiffness)));
    }

   private:
    //! Intensity observation
    Light::Colour colour_;
    //! Intensity stiffness matrix (inverse sqrt of covariance matrix)
    Light::ColourCovariance stiffness_;

};  // class IntensityErrorDirectionalLightAutomatic

}  // namespace ceres_slam

#endif  // CERES_SLAM_INTENSITY_ERROR_DIRECTIONAL_LIGHT_H_
