#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "cscalar.h"
#include "cscalar_impl.h"

//void forward_kinematics(builder::dyn_var<Eigen::> joint_angles) {
//
//}

//void blah() {
//  Eigen::MatrixXcsd a(2,2);
//  a << 
//      SpatialAlgebra::csed(1), SpatialAlgebra::csed(0),
//      SpatialAlgebra::csed(0), SpatialAlgebra::csed(1);
//
//  Eigen::MatrixXcsd b(2,2);
//  b << 
//      SpatialAlgebra::csed(0), SpatialAlgebra::csed(6),
//      SpatialAlgebra::csed(7), SpatialAlgebra::csed(8);
//
//  Eigen::MatrixXcsd c(2,2);
//
//  c = a * b;
//}

void blah() {
  //Eigen::MatrixXcsd a(2,2);
  //a <<
  //    SpatialAlgebra::cscalard(1), SpatialAlgebra::cscalard(2),
  //    SpatialAlgebra::cscalard(3), SpatialAlgebra::cscalard(4);

  //Eigen::MatrixXcsd b(2,2);
  //b <<
  //    SpatialAlgebra::cscalard(5), SpatialAlgebra::cscalard(6),
  //    SpatialAlgebra::cscalard(7), SpatialAlgebra::cscalard(8);

  //Eigen::MatrixXcsd c(2,2);

  //c = a + b;

  SpatialAlgebra::cscalard a(1);
  SpatialAlgebra::cscalard b(3);

  SpatialAlgebra::cscalard x,y,z;

  x = a + b;
  //x = (a + b);// * b;
  //x = (a + b) * b;
  //x *= b;
}

int main(int argc, char* argv[]) {
    // parse URDF file into link and joint tree format
    // do this at runtime because compile-time XML parsing
    // not required.
    
	SpatialAlgebra::generate_spatial_algebra_program(blah, "blah", std::cout);
	return 0;
}
