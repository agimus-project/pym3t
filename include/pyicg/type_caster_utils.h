// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>



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

  PYBIND11_TYPE_CASTER(TransformTplt, _("np.ndarray 4x4"));

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

        E::Map<const E::Matrix<Scalar, 4, 4, E::RowMajor>> matmap(ptr);

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





// struct buffer_info {
//     void *ptr;                      /* Pointer to buffer */
//     ssize_t itemsize;               /* Size of one scalar */
//     std::string format;             /* Python struct-style format descriptor */
//     ssize_t ndim;                   /* Number of dimensions */
//     std::vector<ssize_t> shape;     /* Buffer dimensions */
//     std::vector<ssize_t> strides;   /* Strides (in bytes) for each index */
// };

namespace pybind11 { 
namespace detail{
template<>
struct type_caster<cv::Mat>{
public:
    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

    //! 1. cast numpy.ndarray to cv::Mat
    bool load(handle obj, bool){
        py::array b = py::reinterpret_borrow<array>(obj);
        py::buffer_info info = b.request();

        int nh = 1;
        int nw = 1;
        int nc = 1;
        int ndims = info.ndim;
        if(ndims == 2){
            nh = info.shape[0];
            nw = info.shape[1];
        } else if(ndims == 3){
            nh = info.shape[0];
            nw = info.shape[1];
            nc = info.shape[2];
        }else{
            char msg[64];
            std::sprintf(msg, "Unsupported dim %d, only support 2d, or 3-d", ndims);
            throw std::logic_error(msg);
            return false;
        }

        int dtype;
        if(info.format == py::format_descriptor<unsigned char>::format()){
            dtype = CV_8UC(nc);
        }else if (info.format == py::format_descriptor<int>::format()){
            dtype = CV_32SC(nc);
        }else if (info.format == py::format_descriptor<float>::format()){
            dtype = CV_32FC(nc);
        }else{
            throw std::logic_error("Unsupported type, only support uchar, int32, float");
            return false;
        }

        value = cv::Mat(nh, nw, dtype, info.ptr);
        return true;
    }

    //! 2. cast cv::Mat to numpy.ndarray
    static handle cast(const cv::Mat& mat, 
                       py::return_value_policy  /* policy */, 
                       py::handle  /* parent */){

        std::string format = py::format_descriptor<unsigned char>::format();
        size_t elemsize = sizeof(unsigned char);
        int nw = mat.cols;
        int nh = mat.rows;
        int nc = mat.channels();
        int depth = mat.depth();
        int type = mat.type();
        int dim = (depth == type)? 2 : 3;

        if(depth == CV_8U){
            format = py::format_descriptor<unsigned char>::format();
            elemsize = sizeof(unsigned char);
        }else if(depth == CV_32S){
            format = py::format_descriptor<int>::format();
            elemsize = sizeof(int);
        }else if(depth == CV_32F){
            format = py::format_descriptor<float>::format();
            elemsize = sizeof(float);
        }else{
            throw std::logic_error("Unsupport type, only support uchar, int32, float");
        }

        std::vector<size_t> bufferdim;
        std::vector<size_t> strides;
        if (dim == 2) {
            bufferdim = {(size_t) nh, (size_t) nw};
            strides = {elemsize * (size_t) nw, elemsize};
        } else if (dim == 3) {
            bufferdim = {(size_t) nh, (size_t) nw, (size_t) nc};
            strides = {(size_t) elemsize * nw * nc, (size_t) elemsize * nc, (size_t) elemsize};
        }
        return py::array(py::buffer_info( mat.data,  elemsize,  format, dim, bufferdim, strides )).release();
    }
};
}}//! end namespace pybind11::detail