#ifndef POSE_MESSAGES__MSG__CYLINDER_POSE_HPP_
#define POSE_MESSAGES__MSG__CYLINDER_POSE_HPP_

#include <array>
#include <vector>
#include <string>

namespace pose_messages
{
namespace msg
{

struct CylinderPose
{
    std::string frame_id;
    std::string cylinder_type;
    std::vector<double> positions; // Assuming float64[] in your message
};

}  // namespace msg
}  // namespace pose_messages

#endif  // POSE_MESSAGES__MSG__CYLINDER_POSE_HPP_
