#include <rkdl/rkdl.h>

#include <iostream>
#include <chrono>

bool buildModel(rkdl::RobotModel& model, rkdl::ActuatedJointMap& input, rkdl::FootMap& feet);
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
    rkdl::ActuatedJointMap input;
    rkdl::FootMap feet;
    if (!buildModel(model, input, feet)) return 1;

    // printFrameName(model);
    // printJointName(model);
    // printRootFrame(model);
    // printTreeTypeOfFrame(model);
    // printParentFrameOfFrame(model);
    // printParentJointOfFrame(model);
    printParentJointOfJoint(model);
    printChildJointOfJoint(model);
    input.at("rot1-2") = M_PI_4;
    input.at("rot2-2") = M_PI_4;
    input.at("rot3-2") = M_PI_4;
    input.at("rot4-2") = M_PI_4;
    input.at("rot5-2") = M_PI_4;
    input.at("rot6-2") = M_PI_4;

    input.at("rot1-3") = M_PI_2;
    input.at("rot2-3") = M_PI_2;
    input.at("rot3-3") = M_PI_2;
    input.at("rot4-3") = M_PI_2;
    input.at("rot5-3") = M_PI_2;
    input.at("rot6-3") = M_PI_2;

    auto start = std::chrono::system_clock::now();
    model.updatePos(input);
    rkdl::Kinematics::updateKinematics(model);
    for (const auto& f: feet) 
    {
        rkdl::Vector3 fk = model.getFrame(f.first)->transform_matirx_*f.second;
        // fk.eval();
        rkdl::Jacobian jac = rkdl::Kinematics::jacobian(model, f.first, f.second);
        // jac.eval();
        std::cout << f.first << ": " << std::endl << fk.transpose() << std::endl << jac << std::endl;
    }
    auto end = std::chrono::system_clock::now();
    rkdl::Vector q(3);
    q << 0.0, M_PI_4, M_PI_2;
    rkdl::Vector3 v;
    v << 0.0, 0.0, 180.0;
    auto tm = rkdl::Kinematics::transformMatrix(model, "frame2-3", q);
    std::cout << tm.rotation_ << std::endl << tm.translation_.transpose() << std::endl;
    std::cout << (tm*v).transpose() << std::endl;
    std::cout << rkdl::Kinematics::posFK(model, "frame1-3", q, v).transpose() << std::endl;
    std::cout << std::endl << (rkdl::Kinematics::differentialTransformMatrix(model, "frame1-3", "rot1-1", q)*v).transpose() << std::endl;
    std::cout << std::endl << rkdl::Kinematics::jacobian(model, "frame1-3", q, v) << std::endl;;

    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    return 0;
}


bool buildModel(rkdl::RobotModel& model, rkdl::ActuatedJointMap& input, rkdl::FootMap& feet)
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

    std::vector<std::string> foot_names
    {
        "frame1-3",
        "frame2-3",
        "frame3-3",
        "frame4-3",
        "frame5-3",
        "frame6-3"
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
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0},
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0},
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0},
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0},
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0},
        {0.0, 0.0, 0.0}, {60.504, 0.0, 60.566}, {0.0, 0.0, 220.0}
    };
    std::vector<rkdl::RevoluteAxis> revolute_joint_axis{
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y,
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y,
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y,
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y,
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y,
        rkdl::RevoluteAxis::Z, rkdl::RevoluteAxis::Y, rkdl::RevoluteAxis::Y
    };

    // std::vector<rkdl::Vector3> revolute_joint_axis{
    //     {0.0, 0.0, 1.0}, {0.0, 1.0, 0.1}, {0.0, 1.0, 0.0}
    // };

    std::vector<rkdl::Vector3> fixed_joint_fixed_position{
        { 160.0, -59.27, -42.694}, 
        {   0.0, -59.27, -42.694}, 
        {-160.0, -59.27, -42.694}, 
        {-160.0,  59.27, -42.694}, 
        {   0.0,  59.27, -42.694}, 
        { 160.0,  59.27, -42.694}
    };

    std::vector<rkdl::Scalar> fixed_joint_rotation_rad1
    {
        -M_PI_4, -M_PI_2, -3*M_PI_4,
        3*M_PI_4, M_PI_2, M_PI_4
    };
    std::vector<rkdl::Scalar> fixed_joint_rotation_rad2
    {
        M_PI_4, M_PI_4, M_PI_4, 
        M_PI_4, M_PI_4, M_PI_4
    };
    std::vector<rkdl::Matrix3> fixed_joint_rotation1(fixed_joint_rotation_rad1.size());
    for (int i=0; i<fixed_joint_rotation_rad1.size(); ++i)
        fixed_joint_rotation1[i] = Eigen::AngleAxis<rkdl::Scalar>(fixed_joint_rotation_rad1[i], rkdl::Vector3::UnitZ()).toRotationMatrix();

    std::vector<rkdl::Matrix3> fixed_joint_rotation2(fixed_joint_rotation_rad2.size());
    for (int i=0; i<fixed_joint_rotation_rad2.size(); ++i)
        fixed_joint_rotation2[i] = Eigen::AngleAxis<rkdl::Scalar>(fixed_joint_rotation_rad2[i], rkdl::Vector3::UnitY()).toRotationMatrix();

    std::vector<rkdl::Matrix3> fixed_joint_rotaiton_mat(fixed_joint_name.size());
    for (int i=0; i<fixed_joint_name.size(); ++i) fixed_joint_rotaiton_mat[i] = fixed_joint_rotation1[i]*fixed_joint_rotation2[i];

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
    rkdl::Vector3 to_foot{0.0, 0.0, 180.0}; 
    for (const auto& s: foot_names) feet.emplace(s, to_foot);

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

    std::cout << "pajn: ";
    for (const auto& j: model.joints_)
    {
        std::cout << j->pajn_ << " ";
    }
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

    std::cout << "cajn: ";
    for (const auto& j: model.joints_)
    {
        std::cout << j->cajn_ << " ";
    }
}