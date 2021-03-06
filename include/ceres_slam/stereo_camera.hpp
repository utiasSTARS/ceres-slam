#ifndef CERES_SLAM_STEREO_CAMERA_H_
#define CERES_SLAM_STEREO_CAMERA_H_

#include <Eigen/Core>
#include <memory>

#include <ceres_slam/geometry/geometry.hpp>

namespace ceres_slam {

//! Stereo camera model
template <typename Scalar>
class StereoCamera {
   public:
    //! Pointer type
    typedef std::shared_ptr<StereoCamera> Ptr;
    //! Const pointer type
    typedef std::shared_ptr<const StereoCamera> ConstPtr;
    //! Dimension of the observation (u, v, d)
    static const int obs_dim = 3;
    //! Point type
    typedef Point3D<Scalar> Point;
    //! Point Jacobian type
    typedef Eigen::Matrix<Scalar, Point::dim, obs_dim, Eigen::RowMajor>
        TriangulationJacobian;
    //! Observation type
    typedef Eigen::Matrix<Scalar, obs_dim, 1> Observation;
    //! Observation variance type
    typedef Eigen::Matrix<Scalar, obs_dim, 1> ObservationVariance;
    //! Observation covariance matrix type
    typedef Eigen::Matrix<Scalar, obs_dim, obs_dim, Eigen::RowMajor>
        ObservationCovariance;
    //! Observation Jacobian type
    typedef Eigen::Matrix<Scalar, obs_dim, Point::dim, Eigen::RowMajor>
        ProjectionJacobian;

    //! Copy constructor
    StereoCamera(const StereoCamera &other)
        : fu_(other.fu()),
          fv_(other.fv()),
          cu_(other.cu()),
          cv_(other.cv()),
          b_(other.b()) {}
    //! Construct from parameters
    StereoCamera(const Scalar fu, const Scalar fv, const Scalar cu,
                 const Scalar cv, const Scalar b)
        : fu_(fu), fv_(fv), cu_(cu), cv_(cv), b_(b) {}

    //! Cast to another scalar type (mutable)
    template <typename OtherScalar>
    StereoCamera<OtherScalar> cast() {
        return StereoCamera<OtherScalar>(OtherScalar(fu()), OtherScalar(fv()),
                                         OtherScalar(cu()), OtherScalar(cv()),
                                         OtherScalar(b()));
    }
    //! Cast to another scalar type (const)
    template <typename OtherScalar>
    const StereoCamera<OtherScalar> cast() const {
        return StereoCamera<OtherScalar>(OtherScalar(fu()), OtherScalar(fv()),
                                         OtherScalar(cu()), OtherScalar(cv()),
                                         OtherScalar(b()));
    }

    //! Return horizontal focal length
    inline Scalar fu() const { return fu_; }
    //! Return vertical focal length
    inline Scalar fv() const { return fv_; }
    //! Return horizontal principal point coordinate
    inline Scalar cu() const { return cu_; }
    //! Return vertical horizontal principal point coordinate
    inline Scalar cv() const { return cv_; }
    //! Return baseline
    inline Scalar b() const { return b_; }

    //! Projects a 3D point in the camera frame into the camera
    //! to get a uvd stereo observation.
    const Observation project(
        const Point &pt_c, ProjectionJacobian *jacobian_ptr = nullptr) const {
        Scalar one_over_z = Scalar(1) / pt_c(2);

        Observation obs;  // [u_l, v_l, d]
        obs(0) = fu() * pt_c(0) * one_over_z + cu();
        obs(1) = fv() * pt_c(1) * one_over_z + cv();
        obs(2) = fu() * b() * one_over_z;

        if (jacobian_ptr != nullptr) {
            ProjectionJacobian &jacobian = *jacobian_ptr;

            Scalar one_over_z2 = one_over_z * one_over_z;

            // d(u_l) / d(pt_c)
            jacobian(0, 0) = fu() * one_over_z;
            jacobian(0, 1) = Scalar(0);
            jacobian(0, 2) = -fu() * pt_c(0) * one_over_z2;

            // d(v_l) / d(pt_c)
            jacobian(1, 0) = Scalar(0);
            jacobian(1, 1) = fv() * one_over_z;
            jacobian(1, 2) = -fv() * pt_c(1) * one_over_z2;

            // d(d) / d(pt_c)
            jacobian(2, 0) = Scalar(0);
            jacobian(2, 1) = Scalar(0);
            jacobian(2, 2) = -fu() * b() * one_over_z2;
        }

        return obs;
    }

    //! Triangulates a 3D point in the camera frame
    //! from a uvd stereo observation.
    const Point triangulate(
        const Observation &obs,
        TriangulationJacobian *jacobian_ptr = nullptr) const {
        Point pt_c;
        Scalar b_over_d = b() / obs(2);
        Scalar fu_over_fv = fu() / fv();
        pt_c(0) = (obs(0) - cu()) * b_over_d;
        pt_c(1) = (obs(1) - cv()) * b_over_d * fu_over_fv;
        pt_c(2) = fu() * b_over_d;

        if (jacobian_ptr != nullptr) {
            TriangulationJacobian &jacobian = *jacobian_ptr;

            Scalar b_over_d2 = b_over_d / obs(2);

            // d(x) / d(obs)
            jacobian(0, 0) = b_over_d;
            jacobian(0, 1) = Scalar(0);
            jacobian(0, 2) = (cu() - obs(0)) * b_over_d2;

            // d(y) / d(obs)
            jacobian(1, 0) = Scalar(0);
            jacobian(1, 1) = b_over_d * fu_over_fv;
            jacobian(1, 2) = (cv() - obs(1)) * b_over_d2 * fu_over_fv;

            // d(z) / d(obs)
            jacobian(2, 0) = Scalar(0);
            jacobian(2, 1) = Scalar(0);
            jacobian(2, 2) = -fu() * b_over_d2;
        }

        return pt_c;
    }

    //! Ostream operator for StereoCamera
    friend std::ostream &operator<<(std::ostream &os,
                                    const StereoCamera<Scalar> &c) {
        os << "StereoCamera("
           << "fu: " << c.fu() << ", "
           << "fv: " << c.fv() << ", "
           << "cu: " << c.cu() << ", "
           << "cv: " << c.cv() << ", "
           << "b: " << c.b() << ")";
        return os;
    }

   private:
    Scalar fu_;  //!< Horizontal focal length [px]
    Scalar fv_;  //!< Vertical focal length [px]
    Scalar cu_;  //!< Horizontal principal point coordinate [px]
    Scalar cv_;  //!< Vertical principal point coordinate [px]
    Scalar b_;   //!< Stereo baseline [m]
};               // class StereoCamera

}  // namespace ceres_slam

#endif  // CERES_SLAM_STEREO_CAMERA_H_
