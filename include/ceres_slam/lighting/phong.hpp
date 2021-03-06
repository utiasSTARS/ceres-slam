#ifndef CERES_SLAM_PHONG_H_
#define CERES_SLAM_PHONG_H_

#include <ceres_slam/geometry/geometry.hpp>

#include "vertex3d.hpp"

namespace ceres_slam {

template <typename Scalar>
class PhongModel {
   public:
    //! Vector type
    typedef Vector3D<Scalar> Vector;
    //! Shadable vertex type
    typedef Vertex3D<Scalar> Vertex;
    //! Observation type
    typedef Scalar Colour;

    //! Shade a 3D vertex using Phong lighting
    /*!
        NOTE: The vertex, light direction, and camera direction
        must all be expressed in the same coordinate frame!
    */
    inline static const Colour shade(
        const Vertex& vertex,
        const Vector& light_dir,   // Must be unit length
        const Vector& camera_dir,  // Must be unit length
        const Colour& light_colour) {
        // Shade each component
        // DEBUG: Don't use this component
        // Colour ambient = shade_ambient(vertex);
        Colour ambient = Colour(0.);

        // Check if normal is pointing away from camera
        // if(camera_dir.dot(vertex.normal() ) <= static_cast<Scalar>(0) ) {
        //     clamp(ambient);
        //     return ambient;
        // }

        Colour diffuse = shade_diffuse(vertex, light_dir);
        Colour specular = shade_specular(vertex, light_dir, camera_dir);
        // Colour specular = shade_specular_blinn(vertex, light_dir,
        // camera_dir);

        // Total intensity
        Colour col = light_colour * (ambient + diffuse + specular);
        clamp(col);

        return col;
    }

    //! Shade the ambient component of the Phong model
    inline static const Colour shade_ambient(const Vertex& vertex) {
        return vertex.material()->ambient();
    }

    //! Shade the diffuse component of the Phong model
    inline static const Colour shade_diffuse(const Vertex& vertex,
                                             const Vector& light_dir) {
        // Check for NaN -- pathological case where light_vec.norm() == 0
        if (!light_dir.allFinite()) {
            return Colour(static_cast<Scalar>(0));
        }

        // Dot product of light direction and surface normal
        Scalar light_dir_dot_normal = light_dir.dot(vertex.normal());

        if (light_dir_dot_normal <= static_cast<Scalar>(0)) {
            return Colour(static_cast<Scalar>(0));
        }

        return vertex.texture()->col() * light_dir_dot_normal;
    }

    //! Shade the specular component of the Phong model
    inline static const Colour shade_specular(const Vertex& vertex,
                                              const Vector& light_dir,
                                              const Vector& camera_dir) {
        // Mirror direction
        Vector mirror_dir = static_cast<Scalar>(2) *
                                vertex.normal().dot(light_dir) *
                                vertex.normal() -
                            light_dir;

        // Check for NaN in case of pathological cases
        // where mirror_dir.norm() == 0 or light_vec.norm() == 0
        if (mirror_dir.squaredNorm() <= static_cast<Scalar>(0)) {
            return Colour(static_cast<Scalar>(0));
        }

        mirror_dir.normalize();

        // Dot product of mirror direction and camera direction
        // NOTE: pow(x,y) returns NaN if x < 0 and y non-integer
        Scalar mirror_dir_dot_camera_dir = mirror_dir.dot(camera_dir);

        if (mirror_dir_dot_camera_dir <= static_cast<Scalar>(0)) {
            return Colour(static_cast<Scalar>(0));
        }

        return vertex.material()->specular() *
               pow(mirror_dir_dot_camera_dir, vertex.material()->exponent());
    }

    //! Shade the specular component of the Phong model
    //! using the Blinn-Phong approximation
    inline static const Colour shade_specular_blinn(const Vertex& vertex,
                                                    const Vector& light_dir,
                                                    const Vector& camera_dir) {
        // Halfway direction
        Vector halfway_dir = light_dir + camera_dir;

        // Check for NaN in case of pathological cases
        // where halfway_dir.norm() == 0 or light_vec.norm() == 0
        if (halfway_dir.squaredNorm() <= static_cast<Scalar>(0)) {
            return Colour(static_cast<Scalar>(0));
        }

        halfway_dir.normalize();

        // Dot product of halfway direction and normal vector
        // NOTE: pow(x,y) returns NaN if x < 0 and y non-integer
        Scalar halfway_dir_dot_normal = halfway_dir.dot(vertex.normal());

        if (halfway_dir_dot_normal <= static_cast<Scalar>(0)) {
            return Colour(static_cast<Scalar>(0));
        }

        return vertex.material()->specular() *
               pow(halfway_dir_dot_normal, vertex.material()->exponent());
    }

   private:
    //! Clamp intensity values to lie in [0,1]
    inline static void clamp(Colour& col) {
        col = fmax(Colour(static_cast<Scalar>(0)), col);
        col = fmin(Colour(static_cast<Scalar>(1)), col);
    }
};

}  // namespace ceres_slam

#endif  // CERES_SLAM_PHONG_H_
