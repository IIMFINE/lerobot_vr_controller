#ifndef LEROBOT_VR_CONTROLLER_SRC_VR_TF_FILTER_H_
#define LEROBOT_VR_CONTROLLER_SRC_VR_TF_FILTER_H_

#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace lerobot_vr_controller {

class VrTfFilter {
public:
	explicit VrTfFilter(double alpha_position = 0.5,
											double alpha_orientation = 0.5);

	geometry_msgs::msg::TransformStamped
	Filter(const geometry_msgs::msg::TransformStamped &ts);

	void Reset(const geometry_msgs::msg::TransformStamped &ts);

	void SetAlphaPosition(double alpha);
	void SetAlphaOrientation(double alpha);

	double GetAlphaPosition() const;
	double GetAlphaOrientation() const;

	bool IsInitialized() const;

private:
	static tf2::Quaternion Slerp(const tf2::Quaternion &qa,
															 const tf2::Quaternion &qb, double t);

	double alpha_position_;
	double alpha_orientation_;
	bool first_run_;
	geometry_msgs::msg::TransformStamped filtered_;
};

}  // namespace lerobot_vr_controller

#endif // LEROBOT_VR_CONTROLLER_SRC_VR_TF_FILTER_H_

