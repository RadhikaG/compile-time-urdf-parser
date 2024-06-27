#ifndef ROBOTTYPES_H
#define ROBOTTYPES_H
#include "builder/dyn_var.h"
#include "builder/static_var.h"

using builder::dyn_var;
using builder::static_var;
using builder::as_member;

namespace URDFCompiler {
struct Link {
public:
  std::string name;
  static_var<int> lid;
  static_var<int> urdf_lid;
  static_var<int> bfs_lid;
  static_var<int> parent_id;
};

struct Joint {
public:
  std::string name;
  static_var<int> jid;
  static_var<int> urdf_jid;
  static_var<int> bfs_jid;
  static_var<int> parent_id;
  std::string jtype;
  // free joint variable that is materialized in later stages
  dyn_var<double> theta;
};

class Robot {
  public:

};

}

#endif
