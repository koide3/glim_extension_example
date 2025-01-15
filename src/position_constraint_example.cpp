#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>

#include <glim/util/logging.hpp>
#include <glim/util/interpolation_helper.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/odometry/callbacks.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace glim {

class PositionConstraintExample : public ExtensionModuleROS2 {
public:
  /// @brief Constructor
  PositionConstraintExample() : logger(create_module_logger("position")) {
    logger->info("PositionConstraintExample created");

    // Odom-world transformation initialization-related variables
    is_initialized = false;
    T_odom_world = Eigen::Isometry3d::Identity();

    // Setup odometry estimation callbacks
    OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& frame) { new_frame_callback(frame); });
    OdometryEstimationCallbacks::on_smoother_update.add(
      [this](auto& smoother, auto& new_factors, auto& new_values, auto& new_stamps) { smoother_update_callback(smoother, new_factors, new_values, new_stamps); });
  }

  /// @brief Callback for ROS2 topic subscriptions
  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override {
    auto pose_sub = std::make_shared<TopicSubscription<geometry_msgs::msg::PoseStamped>>("/pose", [this](const auto& msg) { pose_callback(msg); });
    return {pose_sub};
  }

  /// @brief Pose message callback
  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    // Insert the position observation into the input queue
    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    const Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    position_observation_queue.add(stamp, pos);
  }

  /// @brief New odometry estimation frame creation callback
  void new_frame_callback(const EstimationFrame::ConstPtr& new_frame) {
    // Add the new frame to the queue and process the frames in the queue.
    frame_queue.emplace_back(new_frame);
    process_frames();
  }

  /// @brief Process the frames in the queue
  void process_frames() {
    if (frame_queue.empty()) {
      return;
    }

    const auto frame = frame_queue.front();

    // Find the position observations that cover the frame's timestamp. (pos_left.first <= frame->stamp <= pos_right.first).
    int remove_loc;
    std::pair<double, Eigen::Vector3d> pos_left, pos_right;
    const InterpolationHelperResult result = position_observation_queue.find(frame->stamp, &pos_left, &pos_right, &remove_loc);

    if (result == InterpolationHelperResult::FAILURE) {
      // If the frame's timestamp is older than the oldest position observation, remove the frame.
      frame_queue.pop_front();
      return process_frames();
    }

    if (result == InterpolationHelperResult::WAITING) {
      // If the frame's timestamp is newer than the newest position observation, wait for the next position observation.
      return;
    }

    // Here, we successfully found the position observations that cover the frame's timestamp.
    // Remove the used position observations and the frame from the queues.
    position_observation_queue.erase(remove_loc);
    frame_queue.pop_front();

    // Linear interpolate the position observations.
    const double p = (frame->stamp - pos_left.first) / (pos_right.first - pos_left.first);  // Interpolation ratio in [0, 1]
    const Eigen::Vector3d pos_world = (1.0 - p) * pos_left.second + p * pos_right.second;   // Interpolated position

    // If odom-world transformation is not initialized, try to initialize it.
    if (!is_initialized) {
      // Collect the odom-world position correspondences.
      const Eigen::Vector3d pos_odom = frame->T_world_imu.translation();
      odom_pos_correspondences.emplace_back(pos_odom, pos_world);

      // Calc the distance between the first and last world positions to check if there are enough correspondences to estimate odom-world transformation.
      const Eigen::Vector3d first_world_pos = odom_pos_correspondences.front().second;
      const Eigen::Vector3d last_world_pos = odom_pos_correspondences.back().second;
      const double distance = (first_world_pos - last_world_pos).norm();

      if (odom_pos_correspondences.size() < 10 || distance < 10.0) {
        logger->info("Collecting odom-world correspondences: size={} dist={}", odom_pos_correspondences.size(), distance);
        return;
      }

      boost::optional<gtsam::Pose3> result = gtsam::Pose3::Align(odom_pos_correspondences);
      if (!result) {
        logger->error("Failed to align odom and world frames!!");
        return;
      }

      T_odom_world = Eigen::Isometry3d(result->matrix());
      is_initialized = true;
    }

    // Transform the world position to the odom frame and create a factor to constrain the odometry estimation position.
    const Eigen::Vector3d pos_odom = T_odom_world * pos_world;

    using gtsam::symbol_shorthand::X;

    // Assume 0.001m standard deviation
    auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);

    // Create a factor to constrain the IMU pose : X(frame->id) to the transformed position.
    auto factor = gtsam::make_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(X(frame->id), pos_odom, noise_model);
    factor_output_queue.emplace_back(factor);
  }

  /// @brief Odometry estimation optimization callback.
  void smoother_update_callback(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    gtsam::FixedLagSmootherKeyTimestampMap& new_stamps) {
    if (factor_output_queue.empty()) {
      return;
    }

    // Add the created factors to new_factors that will be inserted into the odometry estimation graph.
    logger->info("Insert {} position factors", factor_output_queue.size());
    new_factors.add(factor_output_queue);
    factor_output_queue.clear();
  }

private:
  // Logging
  std::shared_ptr<spdlog::logger> logger;

  // Input queues
  InterpolationHelper<Eigen::Vector3d> position_observation_queue;  // Queue for stamped position observations
  std::deque<EstimationFrame::ConstPtr> frame_queue;                // Queue of unprocessed odometry estimation frames

  // Odom-world transformation initialization
  bool is_initialized;                                                                // If T_odom_world is initialized
  Eigen::Isometry3d T_odom_world;                                                     // Transformation from world to odom frame
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> odom_pos_correspondences;  // pairs of (pos_odom, pos_world)

  // Output queue
  std::vector<gtsam::NonlinearFactor::shared_ptr> factor_output_queue;  // Created factors that will be inserted into the odometry estimation graph
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::PositionConstraintExample();
}