#include "vr_tf_filter.h"

#include <cmath>

#include "log.h"

namespace lerobot_vr_controller {

namespace {
constexpr double kEpsilon = 1e-12;
}

VrTfFilter::VrTfFilter(double alpha_position, double alpha_orientation)
		: alpha_position_(alpha_position),
			alpha_orientation_(alpha_orientation),
			first_run_(true) {
	if (alpha_position_ <= 0.0 || alpha_position_ > 1.0) {
		throw std::out_of_range("alpha_position must be in range (0, 1]");
	}
	if (alpha_orientation_ <= 0.0 || alpha_orientation_ > 1.0) {
		throw std::out_of_range("alpha_orientation must be in range (0, 1]");
	}
}

geometry_msgs::msg::TransformStamped
VrTfFilter::Filter(const geometry_msgs::msg::TransformStamped &ts) {
	if (first_run_) {
		filtered_ = ts;
		first_run_ = false;
		LE_LOG_INFO << "VrTfFilter initialized" << std::endl;
		return filtered_;
	}

	geometry_msgs::msg::TransformStamped out = filtered_;

	out.header = ts.header;
	out.child_frame_id = ts.child_frame_id;

	out.transform.translation.x =
			alpha_position_ * ts.transform.translation.x +
			(1.0 - alpha_position_) * filtered_.transform.translation.x;
	out.transform.translation.y =
			alpha_position_ * ts.transform.translation.y +
			(1.0 - alpha_position_) * filtered_.transform.translation.y;
	out.transform.translation.z =
			alpha_position_ * ts.transform.translation.z +
			(1.0 - alpha_position_) * filtered_.transform.translation.z;

		tf2::Quaternion qa(filtered_.transform.rotation.x,
											 filtered_.transform.rotation.y,
											 filtered_.transform.rotation.z,
											 filtered_.transform.rotation.w);
		tf2::Quaternion qb(ts.transform.rotation.x, ts.transform.rotation.y,
											 ts.transform.rotation.z, ts.transform.rotation.w);

	double na = qa.length2();
	double nb = qb.length2();
	if (na < kEpsilon || nb < kEpsilon) {
		out.transform.rotation = ts.transform.rotation;
	}

	if (na >= kEpsilon && nb >= kEpsilon) {
		qa.normalize();
		qb.normalize();
		tf2::Quaternion q = Slerp(qa, qb, alpha_orientation_);
		q.normalize();
		out.transform.rotation.x = q.x();
		out.transform.rotation.y = q.y();
		out.transform.rotation.z = q.z();
		out.transform.rotation.w = q.w();
	}

	filtered_ = out;
	return filtered_;
}

void VrTfFilter::Reset(const geometry_msgs::msg::TransformStamped &ts) {
	filtered_ = ts;
	first_run_ = true;
}

void VrTfFilter::SetAlphaPosition(double alpha) {
	if (alpha <= 0.0 || alpha > 1.0) {
		throw std::out_of_range("alpha_position must be in range (0, 1]");
	}
	alpha_position_ = alpha;
}

void VrTfFilter::SetAlphaOrientation(double alpha) {
	if (alpha <= 0.0 || alpha > 1.0) {
		throw std::out_of_range("alpha_orientation must be in range (0, 1]");
	}
	alpha_orientation_ = alpha;
}

double VrTfFilter::GetAlphaPosition() const { return alpha_position_; }

double VrTfFilter::GetAlphaOrientation() const { return alpha_orientation_; }

bool VrTfFilter::IsInitialized() const { return !first_run_; }

tf2::Quaternion VrTfFilter::Slerp(const tf2::Quaternion &qa,
								  const tf2::Quaternion &qb, double t) {
	tf2::Quaternion q1 = qa;
	tf2::Quaternion q2 = qb;

	double dot = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() +
							 q1.w() * q2.w();
	if (dot < 0.0) {
		q2 = tf2::Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
		dot = -dot;
	}

	if (dot > 0.9995) {
		tf2::Quaternion result(
				q1.x() + t * (q2.x() - q1.x()), q1.y() + t * (q2.y() - q1.y()),
				q1.z() + t * (q2.z() - q1.z()), q1.w() + t * (q2.w() - q1.w()));
		result.normalize();
		return result;
	}

	double theta_0 = std::acos(dot);
	double theta = theta_0 * t;

	double sin_theta_0 = std::sin(theta_0);
	if (std::abs(sin_theta_0) < kEpsilon) {
		return q1;
	}

	double sin_theta = std::sin(theta);
	double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
	double s1 = sin_theta / sin_theta_0;

	tf2::Quaternion out(q1.x() * s0 + q2.x() * s1, q1.y() * s0 + q2.y() * s1,
											q1.z() * s0 + q2.z() * s1,
											q1.w() * s0 + q2.w() * s1);
	out.normalize();
	return out;
}

}  // namespace lerobot_vr_controller

