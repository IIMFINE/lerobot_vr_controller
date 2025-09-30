#ifndef TOPIC_H_
#define TOPIC_H_

namespace lerobot_vr_controller {

// Topic name constants
static constexpr const char *kVrControllerJointCmdTopic =
    "/vr_controller/joint_cmd";
static constexpr const char *kVrControllerJoyTopic = "/vr/controller_right/joy";
static constexpr const char *kJointStatesTopic = "/joint_states";
static constexpr const char *kRobotControlMotorCmdTopic =
    "/robot_control/motor_cmd";
static constexpr const char *kRobotControlMotorStateTopic =
    "/robot_control/motor_state";
} // namespace lerobot_vr_controller

#endif // TOPIC_H_
