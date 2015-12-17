namespace ceres_slam {

//! Templated version of fmax for use with ceres
template <typename Scalar>
Scalar fmax(Scalar a, Scalar b) { return ( (a >= b) ? a : b ); }

//! Templated version of fmin for use with ceres
template <typename Scalar>
Scalar fmin(Scalar a, Scalar b) { return ( (a <= b) ? a : b ); }

} // namespace ceres_slam
