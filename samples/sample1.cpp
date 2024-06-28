#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "cscalar.h"

//void forward_kinematics(builder::dyn_var<Eigen::> joint_angles) {
//
//}

void blah() {
  Eigen::MatrixXcsd a(2,2);
  a << 
      SpatialAlgebra::cscalard(1), SpatialAlgebra::cscalard(0),
      SpatialAlgebra::cscalard(0), SpatialAlgebra::cscalard(1);

  Eigen::MatrixXcsd b(2,2);
  b << 
      SpatialAlgebra::cscalard(), SpatialAlgebra::cscalard(6),
      SpatialAlgebra::cscalard(7), SpatialAlgebra::cscalard(8);

  Eigen::MatrixXcsd c(2,2);

  c = a * b;
}

int main(int argc, char* argv[]) {
    // parse URDF file into link and joint tree format
    // do this at runtime because compile-time XML parsing
    // not required.
    
	SpatialAlgebra::generate_spatial_algebra_program(blah, "blah", std::cout);
	return 0;
}
