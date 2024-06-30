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
  typedef SpatialAlgebra::cscalar_expr_leaf<double> csec;
  //a <<
  //    csec(1), csec(2),
  //    csec(3), csec(4);

  ////a << 
  ////    SpatialAlgebra::csed(1), SpatialAlgebra::csed(0),
  ////    SpatialAlgebra::csed(0), SpatialAlgebra::csed(1);

  //Eigen::MatrixXcsd b(2,2);
  //b <<
  //    csec(5), csec(6),
  //    csec(7), csec(8);
  ////b << 
  ////    SpatialAlgebra::csed(0), SpatialAlgebra::csed(6),
  ////    SpatialAlgebra::csed(7), SpatialAlgebra::csed(8);

  //Eigen::MatrixXcsd c(2,2);
  //c <<
  //    csec(0), csec(0),
  //    csec(0), csec(0);

  SpatialAlgebra::cscalard a(1);
  SpatialAlgebra::cscalard b(3);

  SpatialAlgebra::cscalard x,y,z;

  x = (a + b) * b;
  x *= b;
 
  //x = c(0,0);
  //y = c(0,1);
  //z = c(1,0);
}

int main(int argc, char* argv[]) {
    // parse URDF file into link and joint tree format
    // do this at runtime because compile-time XML parsing
    // not required.
    
	SpatialAlgebra::generate_spatial_algebra_program(blah, "blah", std::cout);
	return 0;
}
