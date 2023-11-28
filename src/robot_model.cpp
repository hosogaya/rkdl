#include <rkdl/robot_model.h>

namespace rkdl
{
RobotModel::RobotModel() {}

RobotModel::~RobotModel() {}

void RobotModel::addFrame(std::shared_ptr<Frame>& f) 
{
    frames_.emplace_back(f);
}

void RobotModel::addJoint(std::shared_ptr<JointBase>& j)
{
    joints_.emplace_back(j);
}

bool RobotModel::initialize() 
{
    if (!checkDuplication()) return false;
    if (!checkConnectivity()) return false;

    jn_ = joints_.size();
    fn_ = frames_.size();
    return true;
}

bool RobotModel::updateState()
{
    return true;
}

std::shared_ptr<Frame>& RobotModel::findFrame(const Name& name)
{
    auto iter = std::find_if(frames_.begin(), frames_.end(), [name](const std::shared_ptr<Frame>& f){return f->name_ == name;});
    
    assert(iter!=frames_.end());
    return *iter;
}

std::shared_ptr<JointBase>& RobotModel::findJoint(const Name& name)
{
    auto iter = std::find_if(joints_.begin(), joints_.end(), [name](const std::shared_ptr<JointBase>& j){return j->name_ == name;});
    
    assert(iter!=joints_.end());
    return *iter;
}

bool RobotModel::checkDuplication()
{
    for (auto& f: frames_)
    {
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& ff){return f->name_ == ff->name_;});

        if (iter != frames_.end()) return false;
    }
    
    for (auto& j: joints_)
    {
        auto iter = std::find_if(joints_.begin(), joints_.end(), [j](const std::shared_ptr<JointBase>& jj){return j->name_ == jj->name_;});

        if (iter != joints_.end()) return false;
    }
    return true;
}

bool RobotModel::checkConnectivity()
{
    // check whether parent frame exists and uniqueness of the root frame
    int root_num = 0;
    for (auto& f: frames_)
    {
        // search parent frame
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& pf){return f->parent_frame_ == pf->name_;});

        // does not have parent frame
        if (iter == frames_.end())
        {
            ++root_num;
            if (root_num > 1) return false;
            root_frame_ = f->name_;
            root_frame_index_ = std::distance(frames_.begin(), std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& ff){return f->name_ == ff->name_;}));
            f->type_ = Tree::Root;
        }
        else
        {
            f->type_ = Tree::Internal;
            f->parent_frame_index_ = std::distance(frames_.begin(), iter);
        }
    }

    // check leaf frames
    for (auto& f: frames_)
    {
        // search child frame
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& cf){return f->name_ == cf->parent_frame_;});

        // does not have children
        if (iter == frames_.end())
        {
            f->type_ = Tree::Leaf;
        }
    }

    // check wheter parent joint exists
    for (auto& f: frames_)
    {
        if (f->isRoot()) continue;
        // search parent joint
        auto iter = std::find_if(joints_.begin(), joints_.end(), [f](const std::shared_ptr<JointBase>& j){return f->parent_joint_ == j->name_;});

        if (iter == joints_.end()) return false;
        else f->parent_joint_index_ = std::distance(joints_.begin(), iter);
    }

    // set child joint of each joint
    for (auto& j: joints_)
    {
        // search child frame
        std::shared_ptr<Frame>& c_frame = *std::find_if(frames_.begin(), frames_.end(), [j](const std::shared_ptr<Frame>& f){return j->name_ == f->parent_joint_;});
        
        if (c_frame->isLeaf())
        {
            j->child_joint_index_ = -1;
            continue;
        }

        // search grandchild frame
        std::shared_ptr<Frame>& gc_frame = *std::find_if(frames_.begin(), frames_.end(), [c_frame](const std::shared_ptr<Frame>& gcf){return c_frame->name_ == gcf->parent_frame_;});

        // search parent joint of grandchild frame
        auto iter = std::find_if(joints_.begin(), joints_.end(), [gc_frame](const std::shared_ptr<JointBase>& cj){return gc_frame->parent_joint_ == cj->name_;});
        
        // set child joint
        j->child_joint_index_ = std::distance(joints_.begin(), iter);
    }

    // set parent joint of each joint
    for (auto& j: joints_)
    {
        // search child frame
        std::shared_ptr<Frame>& c_frame = *std::find_if(frames_.begin(), frames_.end(), [j](const std::shared_ptr<Frame>& f){return j->name_ == f->parent_joint_;});
        
        // search parent frame
        std::shared_ptr<Frame>& p_frame = *std::find_if(frames_.begin(), frames_.end(), [c_frame](const std::shared_ptr<Frame>& pf){return c_frame->parent_frame_ == pf->name_;});

        if (p_frame->isRoot()) 
        {
            j->parent_joint_index_ = -1;
            continue;
        }

        // search parent joint of parent frame
        auto iter = std::find_if(joints_.begin(), joints_.end(), [p_frame](const std::shared_ptr<JointBase>& pj){return p_frame->parent_joint_ == pj->name_;});
        
        // set parent joint
        j->parent_joint_index_ = std::distance(joints_.begin(), iter);
    }

    return true;
}

}