#include <rkdl/kinematics/solver.h>

namespace rkdl
{
TransformMatrix Kinematics::transformMatrix(RobotModel& model, const Name& frame_name, const  Vector& q)
{
    std::shared_ptr<Frame> f = model.findFrame(frame_name);
    if (f->isRoot() || q.size()<1) return TransformMatrix();
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = q.size();
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->setPosition(q[q_ind]);
    }
    TransformMatrix result(j->transformMatrix());
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            j->setPosition(q[q_ind]);
        }
        TransformMatrix t(j->transformMatrix()*result);
        result = t;
    }

    if (q_ind != 0) error_ = true;
    return result;
}

Vector3 Kinematics::posFK(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p)
{
    std::shared_ptr<Frame> f = model.findFrame(frame_name);
    if (f->isRoot() || q.size()<1) return Vector3(p);
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = q.size();
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->setPosition(q[q_ind]);
    }
    Vector3 result(j->transformMatrix()*p);
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            j->setPosition(q[q_ind]);
        }
        Vector3 t(j->transformMatrix()*result);
        result = t;
    }

    if (q_ind != 0) error_ = true;
    return result;
}


Matrix3 Kinematics::rotFK(RobotModel& model, const Name& frame_name, const Vector& q, const Matrix3& r)
{
    std::shared_ptr<Frame> f = model.findFrame(frame_name);
    if (f->isRoot() || q.size()<1) return Matrix3(r);
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = q.size();
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->setPosition(q[q_ind]);
    }
    Matrix3 result(j->transformMatrix()*r);
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            j->setPosition(q[q_ind]);
        }
        Matrix3 t(j->transformMatrix()*result);
        result = t;
    }

    if (q_ind != 0) error_ = true;
    return result;   
}

TransformMatrix Kinematics::differentialTransformMatrix(RobotModel& model, const Name& frame_name, const Name& joint_name, const Vector& q)
{
    std::shared_ptr<Frame> f = model.findFrame(frame_name);
    if (f->isRoot() || q.size()<1) return TransformMatrix();
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = q.size();
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->setPosition(q[q_ind]);
    }

    TransformMatrix result(j->transformMatrix());
    if (j->name_ == joint_name) 
    {
        TransformMatrix temp(result*j->differentialOperator());
        result = temp;
    }
    
    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            j->setPosition(q[q_ind]);
        }
        if (j->name_ == joint_name) {
            TransformMatrix t(j->transformMatrix()*result);
            result = t;
        }
        else {
            TransformMatrix t(j->transformMatrix()*j->differentialOperator()*result);
            result = t;
        }
    }

    if (q_ind != 0) error_ = true;
    return result;
}


Jacobian Kinematics::jacobian(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p)
{
    std::shared_ptr<Frame> f = model.findFrame(frame_name);
    if (f->isRoot()) return Jacobian(3, q.size());
    std::shared_ptr<JointBase> j = model.joints_[f->parent_frame_index_];
    
    int q_ind = q.size();
    Jacobian result(3, q.size());
    for (int i=0; result.cols(); ++i) result.col(i)=p;
    if (j->joint_type_!=JointType::Fixed)
    {
        --q_ind;
        j->setPosition(q[q_ind]);
        Vector3 t(result.col(q_ind));
        result.col(q_ind) = j->differentialOperator()*t;
    }
    Jacobian temp(j->transformMatrix()*result);
    result = temp;

    while(!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_!=JointType::Fixed)
        {
            --q_ind;
            if (q_ind < 0) break;
            j->setPosition(q[q_ind]);
            Vector3 t(result.col(q_ind));
            result.col(q_ind) = j->differentialOperator()*t;
        }
        Jacobian tj(j->transformMatrix()*result);
        result = tj;
    }

    if (q_ind != 0) error_ = true;
    return result;
}

}