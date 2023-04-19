#include <string>

#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"

using sdk_sagittarius_arm::SagittariusArmKinematics;
namespace py = pybind11;

PYBIND11_MODULE(pysagittarius, m) {
  m.doc() = "Python binding for Sagittarius Robotic Arm (K1)";

  py::class_<SagittariusArmKinematics>(m, "SagittariusArmKinematics")
      .def(py::init<float, float, float>(),
           py::arg("x") = 0.0,
           py::arg("y") = 0.0,
           py::arg("z") = 0.0)
      .def("solve_pos_quat",
           [](SagittariusArmKinematics &obj,
              py::array_t<float> position,
              py::array_t<float> quaternion) -> py::array_t<float> {
             auto p = position.unchecked<1>();
             auto q = quaternion.unchecked<1>();

             py::array_t<float> joints     = py::array_t<float>(7);
             py::buffer_info joints_buffer = joints.request();
             auto *joints_ptr              = static_cast<float *>(joints_buffer.ptr);

             obj.getIKinThetaQuaternion(
                 p(0), p(1), p(2), q(1), q(2), q(3), q(0), joints_ptr);
             return joints;
           });
}
