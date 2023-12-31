# Robot Kinematics and Dynamics Library
This library calculate Kinematics (Forward Kiematics, Jacobian, etc.) and Dynamics (Forward/Inverse Dynamics, etc.) based on Eigen. 
<span style="color: red; ">Dynamics did not implemented.</span>

# Tree structure
This library can handle robot tree structure which has the following specifications.
* Root Node (Body) is only one. 
* Only Root Node has multiple children nodes.
* A Joint define only one edge (if $N\ (> 2)$ bodies connects a joint, please define $N-1$ joints).  

# Support Joint
* Fixed Joint: fix two bodies relative pose.
* revolute Joint: Successor body rotates around a constant axis. 

# Geometry
* Origin of Frame: the connection position with parent joint
* Frame::cog_: the position of center of body relative to its origin
* JointBase::fixed_position: the coordinate of connection position with parent frame in parent frame origin

# Support functions
These functions use and calculate position/orientation relative to root body coordinate. 

* calculate transform matrixes of each body (${}^{root\ body}T_{body}$)
* forward kinematics
* inverse kinematics (Levenberg-Marquardt method)
* Jacobian of forward kinematics 

# Install
```cmd
mkdir build
cd build
cmake ..
make
sudo make install
```

# ToDo
* Use auto: 
  * If the matrix type was declared explicitly, the benefits of lazy evaluation wolud be decreased. However, it is preferable to declare temporary objects that are used repeatedly with an explicit matrix type.