#include <rkdl/frame.h>

namespace rkdl
{
Frame::Frame(Name id, Name parent_frame, Name parent_joint, Scalar m) 
: name_(id), parent_frame_(parent_frame), parent_joint_(parent_joint), mass_(m)
{}

Frame::~Frame() {}
}