#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <chrono>
#include <random>

class StateEstimateCorruptorNode : public rclcpp::Node {
 public:
	StateEstimateCorruptorNode() : Node("state_estimate_corruptor_node") {
		// Declare and get parameters
		this->declare_parameter<double>("pos_white_sig", 0.0);
		this->declare_parameter<double>("drift_rw_factor", 0.0);
		
		pos_white_sigma_ = this->get_parameter("pos_white_sig").as_double();
		drift_rw_factor_ = this->get_parameter("drift_rw_factor").as_double();

		// Subscribers
		pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/true_pose", 1,
			std::bind(&StateEstimateCorruptorNode::OnPose, this, std::placeholders::_1));
		
		velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
			"/true_twist", 1,
			std::bind(&StateEstimateCorruptorNode::OnVelocity, this, std::placeholders::_1));

		// Publishers
		corrupted_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_est", 1);
		corrupted_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/twist_est", 1);
		corrupted_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/current_state_est", 1);

		// TF
		tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		srand(0);  // initialize the random seed

		RCLCPP_INFO(this->get_logger(), "Finished constructing the state estimate corruptor node");
	}

 private:

	geometry_msgs::msg::PoseStamped pose_corrupted_;

	void OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
		++pose_count_;

		// Apply drift / jumps
		if (virgin_) {
			prev_jump_pose_ = pose->pose;
			drift_.position.x = 0.0;
			drift_.position.y = 0.0;
			drift_.position.z = 0.0;
		} else if (jump_seconds_ > 0.0) {
			double pose_dt = (rclcpp::Time(pose->header.stamp) - prev_pose_time_).seconds();

			// Calculate jump probability as a bernoulli parameter
			// With parameter p drawn based on the CDF of an exponential distribution
			const double bernoulli_p = 1.0 - std::exp(-pose_dt / jump_seconds_);
			const double uniform_random = (rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1);

			// If jump
			if (uniform_random < bernoulli_p) {
				// Time for a jump
				// Drift integrates as random walk over distance
				// Compute displacement since last update
				const double dx = pose->pose.position.x - prev_jump_pose_.position.x;
				const double dy = pose->pose.position.y - prev_jump_pose_.position.y;
				const double dz = pose->pose.position.z - prev_jump_pose_.position.z;

				// Sample jump
				geometry_msgs::msg::Pose jump;
				jump.position.x = whiteNoise(drift_rw_factor_ * std::abs(dx));
				jump.position.y = whiteNoise(drift_rw_factor_ * std::abs(dy));
				jump.position.z = whiteNoise(drift_rw_factor_ * std::abs(dz));

				// Apply to current drift
				drift_.position.x += jump.position.x;
				drift_.position.y += jump.position.y;
				drift_.position.z += jump.position.z;

				prev_jump_pose_ = pose->pose;
			}
		}

		// Corrupt pose with instantaneous white noise and current drift
		geometry_msgs::msg::PoseStamped pose_corrupted(*pose);
		pose_corrupted.pose.position.x += drift_.position.x + whiteNoise(pos_white_sigma_);
		pose_corrupted.pose.position.y += drift_.position.y + whiteNoise(pos_white_sigma_);
		pose_corrupted.pose.position.z += drift_.position.z + whiteNoise(pos_white_sigma_);

		PublishCorruptedPose(pose_corrupted);

		// Update latest timestamps
		virgin_ = false;
		prev_pose_time_ = pose->header.stamp;
	}

	double whiteNoise(double sigma) {
		static std::random_device rd;
		static std::mt19937 gen(rd());

		std::normal_distribution<> d(0.0, sigma);
		return d(gen);
	}

	void OnVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr twist) {
		actual_velocity_global_x_ = twist->twist.linear.x;
		actual_velocity_global_y_ = twist->twist.linear.y;
		actual_velocity_global_z_ = twist->twist.linear.z;

		PublishCorruptedTwist(*twist);
		PublishCorruptedState(*twist);
	}

	void PublishCorruptedPose(const geometry_msgs::msg::PoseStamped& corrupt_pose) {
		corrupted_pose_pub_->publish(corrupt_pose);
		PublishBodyTransform(corrupt_pose);
		pose_corrupted_ = corrupt_pose;
	}

	void PublishCorruptedState(const geometry_msgs::msg::TwistStamped& corrupt_twist) {
		nav_msgs::msg::Odometry corrupted_state;

		corrupted_state.header.stamp = corrupt_twist.header.stamp;
		corrupted_state.header.frame_id = "world";
		corrupted_state.child_frame_id = "body";

		corrupted_state.twist.twist = corrupt_twist.twist;
		corrupted_state.pose.pose = pose_corrupted_.pose;

		corrupted_state_pub_->publish(corrupted_state);
	}

	void PublishCorruptedTwist(const geometry_msgs::msg::TwistStamped& twist) {
		geometry_msgs::msg::TwistStamped corrupted_twist;
		corrupted_twist = twist;
		corrupted_twist.twist.linear.x = twist.twist.linear.x * (1 + whiteNoise(drift_rw_factor_));
		corrupted_twist.twist.linear.y = twist.twist.linear.y * (1 + whiteNoise(drift_rw_factor_));
		corrupted_velocity_pub_->publish(corrupted_twist);
	}

	void PublishBodyTransform(const geometry_msgs::msg::PoseStamped& pose) {
		geometry_msgs::msg::TransformStamped transformStamped;

		transformStamped.header.stamp = pose.header.stamp;
		transformStamped.header.frame_id = "world";
		transformStamped.child_frame_id = "body";
		transformStamped.transform.translation.x = pose.pose.position.x;
		transformStamped.transform.translation.y = pose.pose.position.y;
		transformStamped.transform.translation.z = pose.pose.position.z;
		transformStamped.transform.rotation.x = pose.pose.orientation.x;
		transformStamped.transform.rotation.y = pose.pose.orientation.y;
		transformStamped.transform.rotation.z = pose.pose.orientation.z;
		transformStamped.transform.rotation.w = pose.pose.orientation.w;

		tf_broadcaster_->sendTransform(transformStamped);
	}

	bool virgin_ = true;
	size_t pose_count_ = 0;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrupted_pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr corrupted_velocity_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrupted_state_pub_;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	rclcpp::Time prev_pose_time_;
	geometry_msgs::msg::Pose prev_jump_pose_;
	geometry_msgs::msg::Pose drift_;

	double actual_velocity_global_x_ = 0;
	double actual_velocity_global_y_ = 0;
	double actual_velocity_global_z_ = 0;

	double drift_rw_factor_ = 0.0;
	double pos_white_sigma_ = 0.0;
	double jump_seconds_ = 0.0;
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing state_estimate_corruptor node" << std::endl;

	rclcpp::init(argc, argv);

	auto node = std::make_shared<StateEstimateCorruptorNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
