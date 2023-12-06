#include <rkdl/kinematics/solver.h>

namespace rkdl
{
extern bool Kinematics::error_;

TransformMatrix Kinematics::transformMatrix(RobotModel& model, const Name& frame_name, const  Vector& q)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot()) return TransformMatrix();
    std::shared_ptr<JointBase> j = model.joints_[f->parent_joint_index_];

    int q_ind =j->pajn_;
    // the number of states is not enough
    if (q.size() < j->pajn_) {
        error_ = true;
        return TransformMatrix();
    }
    
    TransformMatrix result;
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->transformMatrix(q[q_ind]);
    }
    else result = j->transformMatrix();
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            result = j->transformMatrix(q[q_ind])*result;
        }
        else result = j->transformMatrix()*result;
    }

    return result;
}

Vector3 Kinematics::posFK(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot() || q.size()<1) return Vector3(p);
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = j->pajn_;
    // the number of states is not enough
    if (q.size() < j->pajn_) {
        error_ = true;
        return Vector3(p);
    }

    Vector3 result;
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        result = j->transformMatrix(q[q_ind])*p;
    }
    result = j->transformMatrix()*p;
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            result = j->transformMatrix(q[q_ind])*result;
        }
        else result = j->transformMatrix()*result;
    }

    return result;
}


Matrix3 Kinematics::rotFK(RobotModel& model, const Name& frame_name, const Vector& q, const Matrix3& r)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot() || q.size()<1) return Matrix3(r);
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = j->pajn_;
    // the number of states is not enough
    if (q.size() < j->pajn_) {
        error_ = true;
        return Matrix3(r);
    }

    Matrix3 result;
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        result = j->transformMatrix(q[q_ind])*r;
    }
    else result = j->transformMatrix()*r;
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            result = j->transformMatrix(q[q_ind])*result;
        }
        else result = j->transformMatrix()*result;
    }

    return result;   
}

TransformMatrix Kinematics::differentialTransformMatrix(RobotModel& model, const Name& frame_name, const Name& joint_name, const Vector& q)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot() || q.size()<1) return TransformMatrix();
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = j->pajn_;
    // the number of states is not enough
    if (q.size() < j->pajn_) {
        error_ = true;
        return TransformMatrix();
    }

    TransformMatrix result;
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        if (j->name_ == joint_name)
            result = j->differentialTransformMatrix(q[q_ind]);
        else result = j->transformMatrix(q[q_ind]);
    }
    else
    {
        if (j->name_ == joint_name) result = j->differentialTransformMatrix();
        else result = j->transformMatrix();
    }
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (j->name_ == joint_name) result = j->differentialTransformMatrix(q[q_ind])*result;
            else result = j->transformMatrix(q[q_ind])*result;
        }
        if (j->name_ == joint_name) result = j->differentialTransformMatrix()*result;
        else result = j->transformMatrix()*result;
    }

    return result;
}


Jacobian Kinematics::jacobian(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot()) return Jacobian(3, q.size());
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int valid_size = j->pajn_;
    int q_ind = j->pajn_;
    // the number of states is not enough
    if (q.size() < valid_size) {
        error_ = true;
        return Jacobian(3, q.size());;
    }

    Jacobian result(3, q.size());
    for (int i=0; valid_size; ++i) result.col(i)=p;
    if (j->joint_type_ != JointType::Fixed)
    {
        --q_ind;
        Vector3 t(result.col(q_ind));
        result.col(q_ind) = j->differentialOperator(q[q_ind])*t;
        result = j->transformMatrix(q[q_ind])*result;
    }
    else result = j->transformMatrix()*result;
    // result = j->transformMatrix()*result.(valid_size);

    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            Vector3 t(result.col(q_ind));
            result.col(q_ind) = j->differentialOperator(q[q_ind])*t;
            result = j->transformMatrix(q[q_ind])*result;
        }
        else result = j->transformMatrix()*result;
    }

    return result;
}

void Kinematics::updateKinematics(RobotModel& model)
{
    model.getFrame(model.root_frame_)->transform_matirx_.setIdentity();
    for (int i=1; i<model.frames_.size(); ++i)
    {
        std::shared_ptr<Frame> f = model.frames_[i];
        std::shared_ptr<JointBase> pj = model.getJoint(f->parent_joint_index_);
        std::shared_ptr<Frame> pf = model.getFrame(f->parent_frame_index_);
        f->transform_matirx_ = pf->transform_matirx_*pj->transformMatrix();
    }
}

Jacobian Kinematics::jacobian(const RobotModel& model, const Name& frame_name, const Vector3& p)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    std::shared_ptr<JointBase> j = model.getJoint(f->parent_joint_index_);
    Jacobian result(3, j->pajn_);

    int index = j->pajn_;
    while (true)
    {
        if (j->joint_type_ != JointType::Fixed)
        {
            --index;
            const TransformMatrix& pt = model.getFrame(f->parent_frame_index_)->transform_matirx_;
            result.col(index) = pt.inverse()*f->transform_matirx_*j->differentialOperator()*pt*p;
        }
        if (index == 0) break;
        if (j->isRoot()) {error_=true; break;}
        f = model.getFrame(f->parent_frame_index_);
        j = model.getJoint(f->parent_joint_index_);
    }
    return result;
}
}