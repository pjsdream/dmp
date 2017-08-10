#include <dmp/rendering/request/request_frame.h>

namespace dmp
{
RequestFrame::RequestFrame()
    : action(Action::Nothing), transform(Eigen::Affine3d::Identity())
{
}

RequestFrame::~RequestFrame() = default;
}
