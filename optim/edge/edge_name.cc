#include"optim/edge/edge_reprojection.h"
#include"optim/edge/edge_direct_method.h"

namespace Optim{

const std::string EdgeMonoInvDepReprojection::type_name_ =
    "EdgeMonoInvDepReprojection";

const std::string EdgeMonoReprojectionXYZ::type_name_ =
    "EdgeMonoReprojectionXYZ";

const std::string EdgeMonoReprojectionPoseOnly::type_name_ =
    "EdgeMonoReprojectionPoseOnly";

const std::string EdgeMonoDirectMethod::type_name_ =
    "EdgeMonoDirectMethod";
}//