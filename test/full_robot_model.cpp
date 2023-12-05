#include <rkdl/rkdl.h>

#include <iostream>
#include <chrono>

bool buildModel(rkdl::RobotModel& model, rkdl::InputMap& input);
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
    rkdl::RobotModel model;
    rkdl::InputMap input;
    if (!buildModel(model, input)) return 1;
    printFrameName(model);
    printJointName(model);
    printRootFrame(model);
    printTreeTypeOfFrame(model);
    printParentFrameOfFrame(model);
    printParentJointOfFrame(model);
    printParentJointOfJoint(model);
    printChildJointOfJoint(model);
    
    auto start = std::chrono::system_clock::now();
    model.updatePos(input);
    rkdl::Kinematics::updateKinematics(model);
    for (const auto& f: model.frames_) {f->transform_matirx_.rotation_.eval(); f->transform_matirx_.translation_.eval();}
    auto end = std::chrono::system_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    return 0;
}


bool buildModel(rkdl::RobotModel& model, rkdl::InputMap& input)
{
    std::vector<std::string> frame_name{
        "body", 
        "lo1", "frame1-1", "frame1-2", "frame1-3",
        "lo2", "frame2-1", "frame2-2", "frame2-3",
        "lo3", "frame3-1", "frame3-2", "frame3-3",
        "lo4", "frame4-1", "frame4-2", "frame4-3",
        "lo5", "frame5-1", "frame5-2", "frame5-3",
        "lo6", "frame6-1", "frame6-2", "frame6-3"
    };

    std::vector<std::string> revolute_joint_name{
        "rot1-1", "rot1-2", "rot1-3",
        "rot2-1", "rot2-2", "rot2-3",
        "rot3-1", "rot3-2", "rot3-3",
        "rot4-1", "rot4-2", "rot4-3",
        "rot5-1", "rot5-2", "rot5-3",
        "rot6-1", "rot6-2", "rot6-3"
    };
    std::vector<std::string> fixed_joint_name{
        "fix1-1", 
        "fix2-1", 
        "fix3-1", 
        "fix4-1", 
        "fix5-1", 
        "fix6-1"
    };
    std::vector<std::string> parent_frame_name{
        "none",
        "body", "lo1", "frame1-1", "frame1-2",
        "body", "lo2", "frame2-1", "frame2-2",
        "body", "lo3", "frame3-1", "frame3-2",
        "body", "lo4", "frame4-1", "frame4-2",
        "body", "lo5", "frame5-1", "frame5-2",
        "body", "lo6", "frame6-1", "frame6-2",
    };
    std::vector<std::string> parent_joint_name{
        "none",
        "fix1-1", "rot1-1", "rot1-2", "rot1-3",
        "fix2-1", "rot2-1", "rot2-2", "rot2-3",
        "fix3-1", "rot3-1", "rot3-2", "rot3-3",
        "fix4-1", "rot4-1", "rot4-2", "rot4-3",
        "fix5-1", "rot5-1", "rot5-2", "rot5-3",
        "fix6-1", "rot6-1", "rot6-2", "rot6-3",
    };
    std::vector<rkdl::Scalar> mass{
        1.0, 
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0
    };
    
    std::vector<rkdl::Vector3> revolute_joint_fixed_position{
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}
    };
    std::vector<rkdl::RevoluteAxis> revolute_joint_axis{
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y
    };

    // std::vector<rkdl::Vector3> revolute_joint_axis{
    //     {0.0, 0.0, 1.0}, {0.0, 1.0, 0.1}, {0.0, 1.0, 0.0}
    // };

    std::vector<rkdl::Vector3> fixed_joint_fixed_position{
        {1.0, 1.0, 0.0}, 
        {1.0, 1.0, 0.0}, 
        {1.0, 1.0, 0.0}, 
        {1.0, 1.0, 0.0}, 
        {1.0, 1.0, 0.0}, 
        {1.0, 1.0, 0.0}
    };

    std::vector<rkdl::Scalar> fixed_joint_rotation_rad{
        -M_PI_4,
        -M_PI_2,
        -3*M_PI_4,
        3*M_PI_4, 
        M_PI_2,
        M_PI_4
    };
    std::vector<rkdl::Vector3> fixed_joint_rotation_axis{
        {0.0, 0.0, 1.0}, 
        {0.0, 0.0, 1.0}, 
        {0.0, 0.0, 1.0}, 
        {0.0, 0.0, 1.0}, 
        {0.0, 0.0, 1.0}, 
        {0.0, 0.0, 1.0}
    };
    std::vector<rkdl::Matrix3> fixed_joint_rotaiton_mat(fixed_joint_name.size());
    for (int i=0; i<fixed_joint_name.size(); ++i) fixed_joint_rotaiton_mat[i] = Eigen::AngleAxis<rkdl::Scalar>(fixed_joint_rotation_rad[i], fixed_joint_rotation_axis[i]).toRotationMatrix();

    std::vector<rkdl::Vector3> cog{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}
    };


    // rkdl::RobotModel model;
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

    for (const auto& s: revolute_joint_name) input.emplace(s, 0.0); 

    return model.initialize();
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