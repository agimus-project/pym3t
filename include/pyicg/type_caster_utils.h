#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>



namespace pybind11 {
namespace detail {

namespace py = pybind11;
namespace E = Eigen;

/**
 * Enable automatic casting of arguments and return values between numpy 4x4 arrays to
 * Eigen Affine 3D Transforms of any scalar type. Might be extended to support other Transform Mode 
 * related to other numpy matrix sizes (e.g. Eigen::Transform::AffineCompact requires a 3x4 matrix).
 * 
 * Special care about the memory layout of Eigen and numpy object. In fact, by default:
 * - Eigen: column-major = Fortran style
 * - numpy: row-major = C-Style
 * Solution: define the Eigen::Transform by imposing a RowMajor underlying representation
*/
template <typename Scalar>
class type_caster<E::Transform<Scalar, 3, E::Affine>> {
 public:
  using TransformTplt = E::Transform<Scalar, 3, E::Affine>;

  PYBIND11_TYPE_CASTER(TransformTplt, _("E::Transform<Scalar, 3, E::Affine>"));

  /**
   * Python array->C++ E::Transform)
   */
  bool load(py::handle src, bool convert) {
    if (py::isinstance<py::array>(src)) {
      py::array_t<Scalar> array = src.cast<py::array_t<Scalar>>();
      // takes a 4x4 matrix
      if (array.ndim() == 2 && array.shape(0) == 4 && array.shape(1) == 4) {

        py::buffer_info buff_info = array.request();
        // buf.ptr is a "void *" type -> needs to be casted to "double *" before use
        Scalar *ptr = static_cast<Scalar *>(buff_info.ptr);

        E::Map<const E::Matrix<Scalar,4,4,E::RowMajor>> matmap(ptr);

        value.matrix() = matmap;

        return true;
      }
    }
    return false;
  }

  /**
   * Conversion part 2 (C++ -> Python)
   */
  static py::handle cast(const E::Transform<Scalar, 3, E::Affine>& src,
                         py::return_value_policy /* policy */,
                         py::handle /* parent */) {
    
    // src.matrix() is column major = Fortran style, default py::array_t is c_style (row-major)
    // -> enforce f_style to have the right representation. np.ndarray will also be F-style on python side
    py::array_t<Scalar, py::array::f_style | py::array::forcecast> array({4,4}, src.data());

    return array.release();
  }
};


}  // namespace detail
}  // namespace pybind11