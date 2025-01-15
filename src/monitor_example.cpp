#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

/// @brief Example extension module that monitors the odometry estimation frames.
class MonitorExample : public ExtensionModule {
public:
  MonitorExample() : logger(create_module_logger("monitor")) {
    logger->info("MonitorExample created");

    // Register a callback to odometry estimation frame created event.
    OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& frame) {
      // Print out the frame id and the world velocity in the IMU frame.
      const int id = frame->id;
      const Eigen::Vector3d v_world_imu = frame->v_world_imu;
      logger->info("id={} v_world_imu={} {} {}", id, v_world_imu.x(), v_world_imu.y(), v_world_imu.z());
    });
  }

private:
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

// Define the entry point for the extension module.
extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::MonitorExample();
}