#include <rkdl/rkdl.h>

#include <iostream>

void printFrameName(const rkdl::RobotModel& model);
void printJointName(const rkdl::RobotModel& model);
void printRootFrame(const rkdl::RobotModel& model);
void printTreeTypeOfFrame(const rkdl::RobotModel& model);
void printParentFrameOfFrame(const rkdl::RobotModel& model);
void printParentJointOfFrame(const rkdl::RobotModel& model);
void printParentJointOfJoint(const rkdl::RobotModel& model);
void printChildJointOfJoint(const rkdl::RobotModel& model);

int main()
{
    std::vector<std::string> frame_name{
        "body", 
        "lo1", "frame1-1", "frame1-2", "frame1-3"
    };

    std::vector<std::string> revolute_joint_name{
        "rot1-1", "rot1-2", "rot1-3"
    };
    std::vector<std::string> fixed_joint_name{
        "fix1"
    };
    std::vector<std::string> parent_frame_name{
        "none",
        "body", "lo1", "frame1-1", "frame1-2"
    };
    std::vector<std::string> parent_joint_name{
        "none",
        "fix1", "rot1-1", "rot1-2", "rot1-3"
    };
    std::vector<rkdl::Scalar> mass{
        1.0, 
        1.0, 1.0, 1.0, 1.0
    };
    
    std::vector<rkdl::Vector3> revolute_joint_fixed_position{
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0} 
    };
    std::vector<rkdl::RevoluteAxis> revolute_joint_axis{
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Z
    };

    std::vector<rkdl::Vector3> fixed_joint_fixed_position{
        {1.0, 1.0, 0.0}
    };

    std::vector<rkdl::Scalar> fixed_joint_rotation_rad{
        M_PI_4
    };
    std::vector<rkdl::Vector3> fixed_joint_rotation_axis{
        {0.0, 0.0, 1.0}
    };
    std::vector<rkdl::Matrix3> fixed_joint_rotaiton_mat(fixed_joint_name.size());
    for (int i=0; i<fixed_joint_name.size(); ++i) fixed_joint_rotaiton_mat[i] = Eigen::AngleAxis<rkdl::Scalar>(fixed_joint_rotation_rad[i], fixed_joint_rotation_axis[i]).toRotationMatrix();

    std::vector<rkdl::Vector3> cog{
        {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0} 
    };


    rkdl::RobotModel model;
    for (int i=frame_name.size()-1; i>=0; --i)
    {
        std::shared_ptr<rkdl::Frame> f = std::make_shared<rkdl::Frame>(frame_name[i], parent_frame_name[i], parent_joint_name[i], mass[i], cog[i]);
        model.addFrame(f);
    }
    for (int i=0; i<fixed_joint_name.size(); ++i)
    {
        std::shared_ptr<rkdl::JointBase> j = std::make_shared<rkdl::FixedJoint>(fixed_joint_name[i], fixed_joint_fixed_position[i], fixed_joint_rotaiton_mat[i]);
        model.addJoint(j);
    }
    for (int i=0; i<revolute_joint_name.size(); ++i)
    {
        std::shared_ptr<rkdl::JointBase> j = std::make_shared<rkdl::RevoluteJoint>(revolute_joint_name[i], revolute_joint_fixed_position[i], revolute_joint_axis[i]);
        model.addJoint(j);
    }

    if (!model.initialize()) return 1;

    printFrameName(model);
    printJointName(model);
    printRootFrame(model);
    printTreeTypeOfFrame(model);
    printParentFrameOfFrame(model);
    printParentJointOfFrame(model);
    printParentJointOfJoint(model);
    printChildJointOfJoint(model);
    
    return 0;
}

void printFrameName(const rkdl::RobotModel& model)
{
    std::cout << "frame name: ";
    for (const auto& f: model.frames_)
    {
        std::cout << f->name_ << " ";
    }
    std::cout << std::endl;
}
void printJointName(const rkdl::RobotModel& model)
{
    std::cout << "joint name: ";
    for (const auto& j: model.joints_)
    {
        std::cout << j->name_ << " ";
    }
    std::cout << std::endl;
}
void printRootFrame(const rkdl::RobotModel& model)
{
    std::cout << "root frame: " << model.root_frame_ << "index: " << model.root_frame_index_ << std::endl;
}

void printTreeTypeOfFrame(const rkdl::RobotModel& model)
{
    std::cout << "tree type of each frame: ";
    for (const auto& f: model.frames_)
    {
        std::cout << f->type_ << " ";
    }
    std::cout << std::endl;
}

void printParentFrameOfFrame(const rkdl::RobotModel& model)
{
    std::cout << "parent frame of each frame: " << std::endl;
    for (const auto& f: model.frames_)
    {
        std::cout << f->parent_frame_ << " ";
    }
    std::cout << std::endl;

    std::cout << "parent frame index of each frame: " << std::endl;
    for (const auto& f: model.frames_)
    {
        std::cout << f->parent_frame_index_ << " ";
    }
    std::cout << std::endl;
}

void printParentJointOfFrame(const rkdl::RobotModel& model)
{
    std::cout << "parent joint of each frame: " << std::endl;
    for (const auto& f: model.frames_)
    {
        std::cout << f->parent_joint_ << " ";
    }
    std::cout << std::endl;

    std::cout << "parent joint index of each frame: " << std::endl;
    for (const auto& f: model.frames_)
    {
        std::cout << f->parent_joint_index_ << " ";
    }
    std::cout << std::endl;
}

void printParentJointOfJoint(const rkdl::RobotModel& model)
{
    std::cout << "parent joint of each joint: " << std::endl;
    for (const auto& j: model.joints_)
    {
        if (j->tree_type_ != rkdl::Tree::Root) std::cout << model.getJoint(j->parent_joint_index_)->name_ << " ";
        else std::cout << "none ";
    }
    std::cout << std::endl;

    std::cout << "parent joint index of each joint: " << std::endl;
    for (const auto& j: model.joints_)
    {
        std::cout << j->parent_joint_index_ << " ";
    }
    std::cout << std::endl;
}
void printChildJointOfJoint(const rkdl::RobotModel& model)
{
    std::cout << "child joint of each joint: " << std::endl;
    for (const auto& j: model.joints_)
    {
        if (j->tree_type_ != rkdl::Tree::Leaf) std::cout << model.getJoint(j->child_joint_index_)->name_ << " ";
        else std::cout << "none ";
    }
    std::cout << std::endl;

    std::cout << "parent joint index of each joint: " << std::endl;
    for (const auto& j: model.joints_)
    {
        std::cout << j->child_joint_index_ << " ";
    }
    std::cout << std::endl;
}