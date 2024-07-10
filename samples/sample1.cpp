#include "builder/dyn_var.h"
#include "builder/forward_declarations.h"
#include "Eigen/Dense"
#include "builder/static_var.h"
#include "cscalar.h"
#include "cscalar_impl.h"

using builder::dyn_var;
using builder::static_var;

//void forward_kinematics(builder::dyn_var<Eigen::> joint_angles) {
//
//}

void eep() {
  Eigen::MatrixXcsd a(2,2);
  a << 
      1, 0,
      0, 1;

  Eigen::MatrixXcsd b(2,2);
  b << 
      0, 6,
      7, 8;

  Eigen::MatrixXcsd c(2,2);

  //c = (a + b).eval();
  c = (a + b).eval();
  //c(0,0) = a(0,0) + b(0,0);
  //c(1,0) = a(1,0) + b(1,0);
  //c(0,1) = a(0,1) + b(0,1);
  //c(1,1) = a(1,1) + b(1,1);
}

void meh() {
  //SpatialAlgebra::cscalard a[4], b[4], c[4];
  //SpatialAlgebra::cscalard sum = 0;

  //for (static_var<int> i = 0; i < 2; i = i+1) {
  //  for (static_var<int> j = 0; j < 2; j = j+1) {
  //    sum = 0;
  //    for (static_var<int> k = 0; k < 2; k = k+1) {
  //      sum += a[i*2+k] * b[k*2+j];
  //    }
  //    c[i*2+j] = sum;
  //  }
  //}
  SpatialAlgebra::cscalard a, b;
  a = 2;
  b = 3;
  SpatialAlgebra::cscalard sum = 0;
  sum += a; 
  sum += b; 
}

void ree() {
  std::vector<dyn_var<double>*> a, b, c;

  for (static_var<int> i = 0; i < 4; i = i+1) {
    dyn_var<double>& l = *new dyn_var<double>(i);
    dyn_var<double>& p = *new dyn_var<double>(i+2);
    dyn_var<double>& q = *new dyn_var<double>;
    a.push_back(l.addr());
    b.push_back(p.addr());
    c.push_back(q.addr());
  }

  for (static_var<int> i = 0; i < 4; i = i+1) {
    *c[i] = *a[i] + *b[i];
  }
}

void blah() {
  SpatialAlgebra::cscalard a(1);
  SpatialAlgebra::cscalard b(3);
  SpatialAlgebra::cscalard x;

  x = a + b;
}

void oof() {
  dyn_var<double> a(1);
  dyn_var<double> b(3);
  dyn_var<double> x;

  x = a + b;
}

int main(int argc, char* argv[]) {
    // parse URDF file into link and joint tree format
    // do this at runtime because compile-time XML parsing
    // not required.
    
	SpatialAlgebra::generate_spatial_algebra_program(eep, "eep", std::cout);
	//SpatialAlgebra::generate_spatial_algebra_program(meh, "meh", std::cout);
	//SpatialAlgebra::generate_spatial_algebra_program(ree, "ree", std::cout);
	//SpatialAlgebra::generate_spatial_algebra_program(blah, "blah", std::cout);
	//SpatialAlgebra::generate_spatial_algebra_program(oof, "oof", std::cout);
	return 0;
}
