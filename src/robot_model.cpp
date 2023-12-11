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

ActuatedJointMap RobotModel::createActuatedJointMap() const
{
    ActuatedJointMap map;
    for (const auto& j: joints_)
    {
        if (j->joint_type_ != JointType::Fixed)
        {
            map.emplace(j->name_, 0.0);
        }
    }

    return map;
}


bool RobotModel::initialize() 
{
    if (!checkDuplication()) return false;
    if (!setRootBody()) return false;
    if (!checkUniquenessOfParent()) return false;
    setTreeTypeOfBody();
    coordinateFrameOrder();
    setIndexes();
    if (!setConnectivity()) return false;

    jn_ = joints_.size();
    fn_ = frames_.size();
    return true;
}

void RobotModel::updatePos(const ActuatedJointMap& pos)
{
    for (const auto& p: pos)
    {
        getJoint(p.first)->setPosition(p.second);
    }
}

void RobotModel::updateVel(const ActuatedJointMap& pos)
{
    for (const auto& p: pos)
    {
        getJoint(p.first)->setVelocity(p.second);
    }
}

void RobotModel::updateTor(const ActuatedJointMap& pos)
{
    for (const auto& p: pos)
    {
        getJoint(p.first)->setTorque(p.second);
    }
}

std::shared_ptr<Frame> RobotModel::findFrame(const Name& name)
{
    auto iter = std::find_if(frames_.begin(), frames_.end(), [name](const std::shared_ptr<Frame>& f){return f->name_ == name;});
    
    assert(iter!=frames_.end());
    return *iter;
}

std::shared_ptr<JointBase> RobotModel::findJoint(const Name& name)
{
    auto iter = std::find_if(joints_.begin(), joints_.end(), [name](const std::shared_ptr<JointBase>& j){return j->name_ == name;});
    
    assert(iter!=joints_.end());
    return *iter;
}

bool RobotModel::checkDuplication()
{
    // check the uniqueness of frames
    for (const auto& f: frames_)
    {
        int fn = std::count_if(frames_.begin(), frames_.end(), [f](const auto& af){return f->name_ == af->name_;});
        if (fn != 1) return false;
    }
    // check the uinquenss of joints
    for (const auto& j: joints_)
    {
        int jn = std::count_if(joints_.begin(), joints_.end(), [j](const auto& aj){return j->name_ == aj->name_;});
        if (jn != 1) return false;
    }
    return true;
}

// search and set root_body name (root_frame)
bool RobotModel::setRootBody()
{
    // detect the root body
    int root_num = 0;
    for (auto& f: frames_)
    {
        // search parent frame
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& pf){return f->parent_frame_ == pf->name_;});
        if (iter == frames_.end())
        {
            if (++root_num > 1) return false;
            root_frame_ = f->name_;
        }
    }
    return true;
}

bool RobotModel::checkUniquenessOfParent()
{
    for (const auto& f: frames_)
    {
        if (f->name_ == root_frame_ || f->parent_frame_ == root_frame_) continue;
        int num = std::count_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& pf){return f->parent_frame_ == pf->name_;});
        if (num > 1) return false;
    }
    for (const auto& f: frames_)
    {
        int num = std::count_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& af){return f->parent_joint_ == af->parent_joint_;});
        if (num != 1) return false;
    }

    return true;
}


void RobotModel::coordinateFrameOrder()
{
    // coordinate frame order
    std::vector<std::shared_ptr<Frame>> fv;
    auto iter = std::find_if(frames_.begin(), frames_.end(), [this](const std::shared_ptr<Frame>& f){return f->name_ == this->root_frame_;});
    fv.push_back(*iter);
    std::vector<Name> fn{root_frame_};
    while (fv.size() < frames_.size())
    {
        std::vector<Name> nfn; // next frame name
        for (auto& f: frames_)
        {
            // search frames whose parent frame is included in fn
            if (std::count_if(fn.begin(), fn.end(), [f](const Name& pfn){return f->parent_frame_ == pfn;}))
            {
                fv.push_back(f);
                nfn.push_back(f->name_);
            }
        }
        fn = nfn;
    }
    frames_ = fv;
}

// set parent frame/joint of frames
bool RobotModel::setParentOfFrame()
{
    // check whether parent frame and joint of the frames exists
    int root_num = 0;
    for (auto& f: frames_)
    {
        // search parent frame
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& pf){return f->parent_frame_ == pf->name_;});

        // root frame
        if (iter == frames_.end())
        {
            if (root_frame_ != f->name_) return false;
            root_frame_index_ = std::distance(frames_.begin(), std::find(frames_.begin(), frames_.end(), f));
            f->parent_frame_index_ = -1;
            f->parent_joint_index_ = -1;
        }
        else
        {
            // check the parent joint exists
            auto jiter = std::find_if(joints_.begin(), joints_.end(), [f](const std::shared_ptr<JointBase>& j){return f->parent_joint_ == j->name_;});
            if (jiter == joints_.end()) return false;

            // set parent frame/joint index
            f->parent_frame_index_ = std::distance(frames_.begin(), iter);
            f->parent_joint_index_ = std::distance(joints_.begin(), jiter);
        }
    }

    return true;
}

// tree type of frame
void RobotModel::setTreeTypeOfBody()
{
    for (auto& f: frames_)
    {
        // search child frame
        auto iter = std::find_if(frames_.begin(), frames_.end(), [f](const std::shared_ptr<Frame>& cf){return f->name_ == cf->parent_frame_;});

        // does not have children
        if (iter == frames_.end())
        {
            f->type_ = Tree::Leaf;
        }
        else if (f->name_ != root_frame_)
        {
            f->type_ = Tree::Internal;
        }
        else 
        {
            f->type_ = Tree::Root;
        }
    }
}

// frame/joint_indexes_
void RobotModel::setIndexes()
{
    for (int i=0; i<frames_.size(); ++i)
    {
        frame_indexes_.emplace(frames_[i]->name_, i);
    }

    for (int i=0; i<joints_.size(); ++i)
    {
        joint_indexes_.emplace(joints_[i]->name_, i);
    }
}

// set child joint index of joints and the number of child actuated joint
void RobotModel::setChildOfJoint()
{
    // set child joint of each joint
    for (auto& j: joints_)
    {
        // search child frame
        std::shared_ptr<Frame>& c_frame = *std::find_if(frames_.begin(), frames_.end(), [j](const std::shared_ptr<Frame>& f){return j->name_ == f->parent_joint_;});

        if (c_frame->isLeaf())
        {
            j->tree_type_ = Tree::Leaf;
            j->child_joint_index_ = -1; // leaf
            continue;
        }

        // search grandchild frame
        std::shared_ptr<Frame>& gc_frame = *std::find_if(frames_.begin(), frames_.end(), [c_frame](const std::shared_ptr<Frame>& gcf){return c_frame->name_ == gcf->parent_frame_;});

        // search parent joint of grandchild frame
        auto iter = std::find_if(joints_.begin(), joints_.end(), [gc_frame](const std::shared_ptr<JointBase>& cj){return gc_frame->parent_joint_ == cj->name_;});
        
        // set child joint
        j->child_joint_index_ = std::distance(joints_.begin(), iter);
    }

    // set the number of child actuated joint
    for (auto j: joints_)
    {
        int num = 0;
        if (j->joint_type_ == JointType::Fixed) num = 0;
        else num = 1;

        if (j->isLeaf()) continue;
        std::shared_ptr<JointBase> tj = joints_[j->child_joint_index_];
        while (!tj->isLeaf())
        {
            if (tj->joint_type_ != JointType::Fixed) ++num;
            tj = joints_[tj->child_joint_index_];
        }
        if (tj->joint_type_ == JointType::Fixed) j->cajn_ = num;
        else j->cajn_ = num+1;
    }
}

// set parent joint index of joints and the number of parent actuated joint
void RobotModel::setParentOfJoint()
{
    for (auto& j: joints_)
    {
        // search child frame
        std::shared_ptr<Frame>& c_frame = *std::find_if(frames_.begin(), frames_.end(), [j](const std::shared_ptr<Frame>& f){return j->name_ == f->parent_joint_;});
        
        // search parent frame
        std::shared_ptr<Frame>& p_frame = *std::find_if(frames_.begin(), frames_.end(), [c_frame](const std::shared_ptr<Frame>& pf){return c_frame->parent_frame_ == pf->name_;});

        if (p_frame->isRoot()) 
        {
            j->tree_type_ = Tree::Root;
            j->parent_joint_index_ = -1; // root 
            continue;
        }

        // search parent joint of parent frame
        auto iter = std::find_if(joints_.begin(), joints_.end(), [p_frame](const std::shared_ptr<JointBase>& pj){return p_frame->parent_joint_ == pj->name_;});
        
        // set parent joint
        j->parent_joint_index_ = std::distance(joints_.begin(), iter);
    }

    // set the number of parent actuated joint
    for (auto j: joints_)
    {
        int num = 0;
        if (j->joint_type_ == JointType::Fixed) num = 0;
        else num = 1;

        if (j->isRoot()) continue;
        std::shared_ptr<JointBase> tj = joints_[j->parent_joint_index_];
        while (!tj->isRoot())
        {
            if (tj->joint_type_ != JointType::Fixed) ++num;
            tj = joints_[tj->parent_joint_index_];
        }
        if (tj->joint_type_ == JointType::Fixed) j->pajn_ = num;
        else j->pajn_ = num+1;
    }
}

bool RobotModel::setConnectivity()
{
    if (!setParentOfFrame()) return false;
    setChildOfJoint();
    setParentOfJoint();

    return true;
}

}