#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

/// @brief Example extension module that monitors the odometry estimation frames.
class ZeroAltitudeExample : public ExtensionModule {
public:
  ZeroAltitudeExample() : logger(create_module_logger("zeroalt")) {
    logger->info("ZeroAltitudeExample created");

    // Register callback for new submap insertion events
    GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) {
      logger->info("submap id: {}", submap->id);

      using gtsam::symbol_shorthand::X;

      auto noise_model = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(0.0, 0.0, 1e6));
      auto factor = gtsam::make_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(X(submap->id), gtsam::Vector3(0.0, 0.0, 0.0), noise_model);
      this->new_factors.add(factor);
    });

    // Register callback for smoother update events
    GlobalMappingCallbacks::on_smoother_update.add([this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
      if (this->new_factors.empty()) {
        return;
      }
      logger->info("Adding {} new factors", this->new_factors.size());

      new_factors.add(this->new_factors);
      this->new_factors.resize(0);
    });
  }

private:
  gtsam::NonlinearFactorGraph new_factors;
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

// Define the entry point for the extension module.
extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::ZeroAltitudeExample();
}