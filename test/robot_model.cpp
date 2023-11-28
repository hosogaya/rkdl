#include <rkdl/rkdl.h>

int main()
{
    rkdl::Vector3 v, pp,ps;
    std::shared_ptr<rkdl::JointBase> j1 = std::make_shared<rkdl::RevoluteJoint>("j1", pp, ps, rkdl::RevoluteAxis::X);
    rkdl::RobotModel model;
    model.addJoint(j1);
    model.joints_[0]->print();

    return 1;
}