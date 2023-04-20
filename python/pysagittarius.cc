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
      .def(
          "solve_pos_quat",
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
          },
          R"pbdoc(
              Given EE position (w.r.t. baselink) and orientation in quaternion,
              compute joint positions
          )pbdoc",
          py::arg("position"),
          py::arg("quaternion"))
      .def(
          "solve_pos_rpy",
          [](SagittariusArmKinematics &obj,
             py::array_t<float> position,
             py::array_t<float> rpy) -> py::array_t<float> {
            auto p     = position.unchecked<1>();
            auto euler = rpy.unchecked<1>();

            py::array_t<float> joints     = py::array_t<float>(7);
            py::buffer_info joints_buffer = joints.request();
            auto *joints_ptr              = static_cast<float *>(joints_buffer.ptr);

            obj.getIKinThetaEuler(
                p(0), p(1), p(2), euler(0), euler(1), euler(2), joints_ptr);
            return joints;
          },
          R"pbdoc(
              Given EE position (w.r.t. baselink) and orientation in RPY,
              compute joint positions
          )pbdoc",
          py::arg("position"),
          py::arg("quaternion"))
      .def(
          "compute_pos_quat",
          [](SagittariusArmKinematics &obj, py::array_t<float> joints) -> py::tuple {
            py::buffer_info joints_buffer = joints.request();
            auto *j                       = static_cast<float *>(joints_buffer.ptr);

            py::array_t<float> pos     = py::array_t<float>(3);
            py::buffer_info pos_buffer = pos.request();
            auto *pos_ptr              = static_cast<float *>(pos_buffer.ptr);

            py::array_t<float> quat     = py::array_t<float>(4);
            py::buffer_info quat_buffer = quat.request();
            auto *quat_ptr              = static_cast<float *>(quat_buffer.ptr);

            obj.getFKinQuaternion(j, pos_ptr, quat_ptr);
            return py::make_tuple(pos, quat);
          },
          R"pbdoc(
              Given joint positions, compute the forward kinematics to deduce
              the end effector's position and orientation (in quaternion).
              Returns a pair of numpy arrays as position and quaternion.
          )pbdoc",
          py::arg("joints"));
}
