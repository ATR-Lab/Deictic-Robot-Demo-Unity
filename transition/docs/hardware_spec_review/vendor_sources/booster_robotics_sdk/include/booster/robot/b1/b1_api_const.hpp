#ifndef BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP
#define BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP

#include <string>

namespace booster {
namespace robot {
namespace b1 {

/**
 * @file b1_api_const.hpp
 * @brief Topic names, joint layouts, and small command enums for B1 robots.
 *
 * The low-level joint arrays are model-specific and must be paired with the
 * matching joint-count constant. Sending a command with the wrong layout can
 * address the wrong actuator or cause the robot to reject the message.
 *
 * @par Joint layouts
 * | Robot model | Joint enum | Joint count |
 * | --- | --- | --- |
 * | K1 | JointIndexK1 | kJointCntK1 (22) |
 * | T1 | JointIndex | kJointCnt (23) |
 * | T1_7DofArm | JointIndexWith7DofArm | kJointCnt7DofArm (29) |
 */

/** @brief Low-level joint-command topic. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicJointCtrl = "rt/joint_ctrl";

/** @brief Low-level motor and sensor state topic. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicLowState = "rt/low_state";

/** @brief Fall and recovery-state topic. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicFallDown = "rt/fall_down";

/** @brief Planar odometry topic used by MoveController. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicOdometerState = "rt/odometer_state";

/**
 * @brief Installed hand actuator-data topic.
 * @note Supported model: K1 | T1 | T2
 * @note Requires a compatible hand.
 */
static const std::string kTopicBoosterHandData = "rt/booster_hand_data";

/**
 * @brief Touch-sensor data topic for compatible hands.
 * @note Availability depends on installed hand hardware, not the robot model.
 */
static const std::string kTopicHandTouchData = "rt/booster_hand_touch_data";

/** @brief Robot transform-tree topic. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicTF = "rt/tf";

/** @brief Current mode, body-control, and action-state topic. @note Supported model: K1 | T1 | T2 */
static const std::string kTopicRobotStates = "rt/robot_states";

/**
 * @brief Prone-body and visual-kick status topic.
 * @note Supported model: K1 | T1 | T2
 * @note Current non-inactive prone-body states are produced by T1-specific body
 * controllers.
 */
static const std::string kTopicProneBodyControlStatus = "rt/prone_body_control_status";

/**
 * @brief Identifier of the recorded upper-body trajectory being replayed.
 * @note Supported model: K1 | T1 | T2
 * @note Trajectory recording and replay supported model: K1 | T1
 */
static const std::string kTopicRobotReplayTrajID = "rt/robot_replay_traj_id";

/**
 * @brief Built-in or custom trained-trajectory execution status topic.
 * @note Supported model: K1 | T1 | T2
 * @note Trained-trajectory execution supported model: K1 | T2
 * @note T1 normally reports an idle state.
 */
static const std::string kTopicTrainedTrajStatus = "rt/trained_traj_status";

/**
 * @brief Soccer behavior-state topic.
 * @note Supported model: K1 | T1
 * @note Soccer mode is disabled on T2.
 */
static const std::string kTopicRobocupBehaviorStatus = "rt/robocup_behavior_status";

/**
 * @brief Visual/soccer kick reference-command topic.
 * @note Supported model: K1 | T1
 * @note T2 has no visual-kick controller.
 */
static const std::string kTopicKickReference = "rt/kick_ball";

/**
 * @brief ROS-compatible odometry topic.
 * @note Supported model: K1 | T1 | T2
 * @note Requires the ROS bridge to be enabled.
 */
static const std::string kTopicRosOdometer = "rt/odom";

/**
 * @brief ROS-compatible IMU topic.
 * @note Supported model: K1 | T1 | T2
 * @note Requires the ROS bridge to be enabled.
 */
static const std::string kTopicRosImu = "rt/imu/data";

/**
 * @brief ROS-compatible joint-state topic.
 * @note Supported model: K1 | T1 | T2
 * @note Requires the ROS bridge to be enabled.
 */
static const std::string kTopicRosJointStates = "rt/joint_states";

/**
 * @brief Joint indices for the 23-joint T1 layout.
 * @note Supported model: T1
 * @note Use JointIndexK1 for K1 and JointIndexWith7DofArm for T1_7DofArm.
 */
enum class JointIndex {
    kHeadYaw = 0,            ///< Head yaw joint.
    kHeadPitch = 1,          ///< Head pitch joint.
    kLeftShoulderPitch = 2,  ///< Left shoulder pitch joint.
    kLeftShoulderRoll = 3,   ///< Left shoulder roll joint.
    kLeftElbowPitch = 4,     ///< Left elbow pitch joint.
    kLeftElbowYaw = 5,       ///< Left elbow yaw joint.
    kRightShoulderPitch = 6, ///< Right shoulder pitch joint.
    kRightShoulderRoll = 7,  ///< Right shoulder roll joint.
    kRightElbowPitch = 8,    ///< Right elbow pitch joint.
    kRightElbowYaw = 9,      ///< Right elbow yaw joint.
    kWaist = 10,             ///< Waist yaw joint.
    kLeftHipPitch = 11,      ///< Left hip pitch joint.
    kLeftHipRoll = 12,       ///< Left hip roll joint.
    kLeftHipYaw = 13,        ///< Left hip yaw joint.
    kLeftKneePitch = 14,     ///< Left knee pitch joint.
    kCrankUpLeft = 15,       ///< Left upper crank joint.
    kCrankDownLeft = 16,     ///< Left lower crank joint.
    kRightHipPitch = 17,     ///< Right hip pitch joint.
    kRightHipRoll = 18,      ///< Right hip roll joint.
    kRightHipYaw = 19,       ///< Right hip yaw joint.
    kRightKneePitch = 20,    ///< Right knee pitch joint.
    kCrankUpRight = 21,      ///< Right upper crank joint.
    kCrankDownRight = 22,    ///< Right lower crank joint.
};

/**
 * @brief Joint indices for the 22-joint K1 layout.
 * @note Supported model: K1
 * @note K1 has no waist joint in this layout.
 */
enum class JointIndexK1 {
    kHeadYaw = 0,            ///< Head yaw joint.
    kHeadPitch = 1,          ///< Head pitch joint.
    kLeftShoulderPitch = 2,  ///< Left shoulder pitch joint.
    kLeftShoulderRoll = 3,   ///< Left shoulder roll joint.
    kLeftElbowPitch = 4,     ///< Left elbow pitch joint.
    kLeftElbowYaw = 5,       ///< Left elbow yaw joint.
    kRightShoulderPitch = 6, ///< Right shoulder pitch joint.
    kRightShoulderRoll = 7,  ///< Right shoulder roll joint.
    kRightElbowPitch = 8,    ///< Right elbow pitch joint.
    kRightElbowYaw = 9,      ///< Right elbow yaw joint.
    kLeftHipPitch = 10,      ///< Left hip pitch joint.
    kLeftHipRoll = 11,       ///< Left hip roll joint.
    kLeftHipYaw = 12,        ///< Left hip yaw joint.
    kLeftKneePitch = 13,     ///< Left knee pitch joint.
    kCrankUpLeft = 14,       ///< Left upper crank joint.
    kCrankDownLeft = 15,     ///< Left lower crank joint.
    kRightHipPitch = 16,     ///< Right hip pitch joint.
    kRightHipRoll = 17,      ///< Right hip roll joint.
    kRightHipYaw = 18,       ///< Right hip yaw joint.
    kRightKneePitch = 19,    ///< Right knee pitch joint.
    kCrankUpRight = 20,      ///< Right upper crank joint.
    kCrankDownRight = 21,    ///< Right lower crank joint.
};

/**
 * @brief Joint indices for the 29-joint T1_7DofArm layout with two 7-DOF arms.
 * @note Supported model: T1_7DofArm
 */
enum class JointIndexWith7DofArm {
    kHeadYaw = 0,            ///< Head yaw joint.
    kHeadPitch = 1,          ///< Head pitch joint.
    kLeftShoulderPitch = 2,  ///< Left shoulder pitch joint.
    kLeftShoulderRoll = 3,   ///< Left shoulder roll joint.
    kLeftElbowPitch = 4,     ///< Left elbow pitch joint.
    kLeftElbowYaw = 5,       ///< Left elbow yaw joint.
    kLeftWristPitch = 6,     ///< Left wrist pitch joint.
    kLeftWristYaw = 7,       ///< Left wrist yaw joint.
    kLeftHandRoll = 8,       ///< Left hand roll joint.
    kRightShoulderPitch = 9, ///< Right shoulder pitch joint.
    kRightShoulderRoll = 10, ///< Right shoulder roll joint.
    kRightElbowPitch = 11,   ///< Right elbow pitch joint.
    kRightElbowYaw = 12,     ///< Right elbow yaw joint.
    kRightWristPitch = 13,   ///< Right wrist pitch joint.
    kRightWristYaw = 14,     ///< Right wrist yaw joint.
    kRightHandRoll = 15,     ///< Right hand roll joint.
    kWaist = 16,             ///< Waist yaw joint.
    kLeftHipPitch = 17,      ///< Left hip pitch joint.
    kLeftHipRoll = 18,       ///< Left hip roll joint.
    kLeftHipYaw = 19,        ///< Left hip yaw joint.
    kLeftKneePitch = 20,     ///< Left knee pitch joint.
    kCrankUpLeft = 21,       ///< Left upper crank joint.
    kCrankDownLeft = 22,     ///< Left lower crank joint.
    kRightHipPitch = 23,     ///< Right hip pitch joint.
    kRightHipRoll = 24,      ///< Right hip roll joint.
    kRightHipYaw = 25,       ///< Right hip yaw joint.
    kRightKneePitch = 26,    ///< Right knee pitch joint.
    kCrankUpRight = 27,      ///< Right upper crank joint.
    kCrankDownRight = 28     ///< Right lower crank joint.
};

static const size_t kJointCnt = 23;        ///< Number of joints in the T1 layout.
static const size_t kJointCnt7DofArm = 29; ///< Number of joints in the T1_7DofArm layout.
static const size_t kJointCntK1 = 22;      ///< Number of joints in the K1 layout.

/** @brief Selects the left or right hand. @note Supported model: K1 | T1 | T2 */
enum HandIndex {
    kLeftHand = 0,  ///< Left hand or gripper.
    kRightHand = 1, ///< Right hand or gripper.
};

/**
 * @brief Hand hardware identifiers used by dexterous-hand commands.
 * @note Support depends on the hand hardware returned by
 * B1LocoClient::GetHands(), not solely on the robot model.
 */
enum class BoosterHandType {
    kInspireHand = 0,      ///< Inspire dexterous hand without touch reporting.
    kInspireTouchHand = 2, ///< Inspire dexterous hand with touch sensing.
    kRevoHand = 3,         ///< Revo dexterous hand.
    kUnknown = -1,         ///< Unknown or unavailable hand type.
};

/**
 * @brief Start/stop command convention used by hand actions.
 *
 * For high-level gestures, kHandOpen conventionally starts the action and
 * kHandClose stops it. Individual APIs may impose additional restrictions;
 * for example, T2 hand waving does not support manual stop.
 */
enum HandAction {
    kHandOpen = 0,  ///< Open the hand or start the selected hand action.
    kHandClose = 1, ///< Close the hand or stop the selected hand action.
};

/** @brief Linux input event types emitted by a remote controller device. */
enum RemoteControllerEvent {
    NONE = 0,            ///< No event.
    AXIS = 0x600,        ///< Analog-axis motion.
    HAT = 0x602,         ///< Directional-hat position change.
    BUTTON_DOWN = 0x603, ///< Button pressed.
    BUTTON_UP = 0x604,   ///< Button released.
    REMOVE = 0x606       ///< Controller device removed.
};

}
}
} // namespace booster::robot::b1

#endif // BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP
