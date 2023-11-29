#pragma once

#include <rkdl/frame.h>
#include <rkdl/joint/joint_base.h>

#include <vector>
#include <memory>
#include <unordered_map>

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
    std::shared_ptr<Frame> findFrame(const Name& name);
    std::shared_ptr<JointBase> findJoint(const Name& name);

    std::vector<std::shared_ptr<JointBase>> joints_;
    std::vector<std::shared_ptr<Frame>> frames_;
    Name root_frame_;
    int root_frame_index_;
    // std::vector<Name> leaf_frames_;
    // std::vector<int> leaf_frame_indexes_;
    // std::vector<int> root_joint_indexes_;

    int jn_ = 0; // the number of joints
    int fn_ = 0; // the number of frames
    // int dn_ = 0; // depth of kinematic tree

    std::shared_ptr<Frame> getFrame(const Name& name){return frames_[frame_indexes_.at(name)];}
    std::shared_ptr<Frame> getFrame(const int index) {return frames_[index];}
    int getFrameIndex(const Name& name) {return frame_indexes_.at(name);}
    std::shared_ptr<JointBase> getJoint(const Name& name) {return joints_[joint_indexes_.at(name)];}
    std::shared_ptr<JointBase> getJoint(const int& index) {return joints_[index];}
    int getJointIndex(const Name& name) {return joint_indexes_.at(name);}

private:
    bool checkDuplication();

    bool setRootBody();
    void setTreeTypeOfBody();

    void coordinateFrameOrder();
    void setIndexes();

    bool setConnectivity();
    bool setParentOfFrame();
    void setChildOfJoint();
    void setParentOfJoint();

    std::unordered_map<Name, int> joint_indexes_;
    std::unordered_map<Name, int> frame_indexes_;
};

}