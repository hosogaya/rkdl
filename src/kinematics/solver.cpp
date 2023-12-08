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
        result = j->transformMatrix(q[q_ind]);
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
        else 
        {
            result = j->transformMatrix()*result;
        }
    }

    return result;
}

Vector3 Kinematics::posFK(RobotModel& model, const Name& frame_name, const Vector& q, const Vector3& p)
{
    TransformMatrix tm = transformMatrix(model, frame_name, q);

    return tm*p;
}


Matrix3 Kinematics::rotFK(RobotModel& model, const Name& frame_name, const Vector& q, const Matrix3& r)
{
    return transformMatrix(model, frame_name, q).rotation_*r;
}

// dT/dq
TransformMatrix Kinematics::differentialTransformMatrix(RobotModel& model, const Name& frame_name, const Name& joint_name, const Vector& q)
{
    std::shared_ptr<Frame> f = model.getFrame(frame_name);
    if (f->isRoot() || q.size()<1) return TransformMatrix();
    std::shared_ptr<JointBase> j = model.joints_[f->parent_joint_index_];
    
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
    std::shared_ptr<JointBase> j = model.joints_[f->parent_joint_index_];
    
    const int valid_size = j->pajn_;
    int q_ind = j->pajn_;
    // the number of states is not enough
    if (q.size() < valid_size) {
        error_ = true;
        return Jacobian(3, q.size());;
    }

    // cal. trasnform matrixes of each frame to root frame. 
    std::vector<TransformMatrix> tfv(0); // transform matrix of all joint
    std::vector<int> aj_index(valid_size); // actuated joint index of tfv
    std::vector<TransformMatrix> dov(valid_size); // differential operator
    if (j->joint_type_ != JointType::Fixed)
    {
        --q_ind;
        aj_index[q_ind] = tfv.size();
        dov[q_ind] = j->differentialOperator(q[q_ind]);
        tfv.emplace_back(j->transformMatrix(q[q_ind]));
    }
    else tfv.emplace_back(j->transformMatrix());

    while (!j->isRoot())
    {
        j = model.joints_[j->parent_joint_index_];
        if (j->joint_type_ != JointType::Fixed)
        {
            --q_ind;
            aj_index[q_ind] = tfv.size();
            dov[q_ind] = j->differentialOperator(q[q_ind]);
            tfv.emplace_back(j->transformMatrix(q[q_ind]));
        }
        else tfv.emplace_back(j->transformMatrix());
    }
    // update transform matrixes (0Tj)
    for (size_t i=tfv.size()-1; i>0; --i)
    {
        tfv[i-1] = tfv[i]*tfv[i-1];
    }

    // cal. jacobian
    Jacobian result(3, valid_size);
    TransformMatrix tr(tfv[0]);
    tr.translation_ = tr.rotation_*p + tr.translation_;
    for (size_t i=0; i<valid_size; ++i)
    {
        result.col(i) = tfv[aj_index[i]].rotation_*(dov[i]*(tfv[aj_index[i]].inverse()*tr).translation_);
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
    const TransformMatrix& tf = f->transform_matirx_;
    std::shared_ptr<JointBase> j = model.getJoint(f->parent_joint_index_);
    Jacobian result(3, j->pajn_);

    int index = j->pajn_;
    TransformMatrix tt(f->transform_matirx_);
    tt.translation_ = tt.rotation_*p + tt.translation_;
    while (true)
    {
        if (j->joint_type_ != JointType::Fixed)
        {
            --index;
            const TransformMatrix& ct = f->transform_matirx_;
            result.col(index) = ct.rotation_*(j->differentialOperator()*(ct.inverse()*tt).translation_);
        }
        if (index == 0) break;
        if (j->isRoot()) {error_=true; break;}
        f = model.getFrame(f->parent_frame_index_);
        j = model.getJoint(f->parent_joint_index_);
    }
    return result;
}
}