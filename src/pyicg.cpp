#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "icg/tracker.h"


namespace py = pybind11;
// to be able to use "arg"_a shorthand
using namespace pybind11::literals;


#define PYBIND11_DETAILED_ERROR_MESSAGES


PYBIND11_MODULE(_pyicg_mod, m) {
    // Constructor and setup method
    py::class_<icg::Tracker>(m, "Tracker")
        .def(py::init<const std::string, int, int, bool, 
                      const std::chrono::milliseconds&, int, int>(), 
                      "name"_a="tracker", "n_corr_iterations"_a=5, "n_update_iterations"_a=2, "synchronize_cameras"_a=true, 
                      "cycle_duration"_a=std::chrono::milliseconds{33}, "visualization_time"_a=0, "viewer_time"_a=1)
        .def("SetUp", &icg::Tracker::SetUp);

        //////////////////////////////////////
        //////////////////////////////////////
        // TODO
        //////////////////////////////////////
        //////////////////////////////////////

        // RendererGeometry
        // RealSenseColorCamera
        // RealSenseDepthCamera
        // NormalDepthViewer
        // NormalColorViewer
        // FocusedBasicDepthRenderer
        // Body
        // StaticDetector
        // RegionModel
        // DepthModel
        // RegionModality
        // DepthModality
        // Optimizer
}   


