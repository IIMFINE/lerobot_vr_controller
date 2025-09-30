#include "kinematics.h"
#include "xlerobot_kinematics.h"
#include "log.h"

#include <algorithm>
#include <cctype>

namespace lerobot_vr_controller {

std::unique_ptr<KinematicsInterface> KinematicsFactory::CreateKinematics(KinematicsType type) {
  switch (type) {
    case KinematicsType::XLEROBOT:
      return std::make_unique<XLeRobotKinematics>();
    
    default:
      LE_LOG_ERROR << "Unknown kinematics type: " << static_cast<int>(type) << std::endl;
      return nullptr;
  }
}

std::unique_ptr<KinematicsInterface> KinematicsFactory::CreateKinematics(const std::string &type_str) {
  // 转换为小写进行比较
  std::string lower_type = type_str;
  std::transform(lower_type.begin(), lower_type.end(), lower_type.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  
  if (lower_type == "xlerobot") {
    return CreateKinematics(KinematicsType::XLEROBOT);
  }
  
  LE_LOG_ERROR << "Unknown kinematics type string: " << type_str << std::endl;
  return nullptr;
}

std::vector<std::string> KinematicsFactory::GetAvailableTypes() {
  return {"xlerobot"};
}

void KinematicsInterface::SetEndEffectorFrame(const std::string &end_effector_frame) {
  end_effector_frame_ = end_effector_frame;
}

} // namespace lerobot_vr_controller