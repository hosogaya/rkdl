#pragma once

#include <rkdl/frame.h>
#include <rkdl/joint/joint_base.h>

#include <vector>
#include <memory>

namespace rkdl
{
class RobotModel
{
public:
    RobotModel();
    ~RobotModel();

    void addJoint(std::shared_ptr<JointBase>& j);
    void addFrame(std::shared_ptr<Frame>& f);
    bool initialize();
    bool updateState();
    std::shared_ptr<Frame>& findFrame(const Name& name);
    std::shared_ptr<JointBase>& findJoint(const Name& name);

    std::vector<std::shared_ptr<JointBase>> joints_;
    std::vector<std::shared_ptr<Frame>> frames_;
    Name root_frame_;
    int root_frame_index_;
    std::vector<Name> leaf_frames_;
    std::vector<int> leaf_frame_indexes_;
    std::vector<int> root_joint_indexes_;

    int jn_ = 0; // the number of joints
    int fn_ = 0; // the number of frames
    // int dn_ = 0; // depth of kinematic tree

private:
    bool checkDuplication();
    bool checkConnectivity();
};

}