#include "Eigen/Dense"

#include <iostream>

namespace ctup_gen {

int Sphere_Environment_Collision (blaze::StaticVector<double, 16> arg1, blaze::StaticVector<double, 16> arg2, blaze::StaticVector<double, 16> arg3, blaze::StaticVector<double, 16> arg4, std::vector<blaze::StaticVector<double, 16>> arg5, std::vector<blaze::StaticVector<double, 16>> arg6, std::vector<blaze::StaticVector<double, 16>> arg7, std::vector<blaze::StaticVector<double, 16>> arg8) {
  int var174;
  std::vector<blaze::StaticVector<double, 16>> var0 = arg8;
  std::vector<blaze::StaticVector<double, 16>> var1 = arg7;
  std::vector<blaze::StaticVector<double, 16>> var2 = arg6;
  std::vector<blaze::StaticVector<double, 16>> var3 = arg5;
  blaze::StaticVector<double, 16> var4 = arg4;
  blaze::StaticVector<double, 16> var5 = arg3;
  blaze::StaticVector<double, 16> var6 = arg2;
  blaze::StaticVector<double, 16> var7 = arg1;
  int var8 = 0;
  blaze::StaticVector<double, 16> var9 = var7 - 1.33815338075485;
  blaze::StaticVector<double, 16> var10 = var6 - -0.331943679234558;
  blaze::StaticVector<double, 16> var11 = var5 - 0.38;
  blaze::StaticVector<double, 16> var12 = var11 * 0.0;
  blaze::StaticVector<double, 16> var13 = min(max(var12 * 0.03, 1.0), 0.0);
  var11 = var5 - (0.38 + (0.0 * var13));
  blaze::StaticVector<double, 16> var14 = ((var9 * var9) + (var10 * var10)) + (var11 * var11);
  blaze::StaticVector<double, 16> var15 = var4 + 0.14;
  var14 = var14 - (var15 * var15);
  int var16 = 0;
  while ((var16 < 16) && (var8 == 0)) {
    if (var14[var16] < 0.0) {
      var8 = 1;
    } 
    var16 = var16 + 1;
  }
  if (var8) {
    for (int var17 = 0; var17 < var3.size(); var17 = var17 + 1) {
      var9 = var3[var17] - 1.33815338075485;
      var10 = var2[var17] - -0.331943679234558;
      var11 = var1[var17] - 0.38;
      var13 = min(max(var12 * 0.03, 1.0), 0.0);
      var11 = var5 - (0.38 + (0.0 * var13));
      var14 = ((var9 * var9) + (var10 * var10)) + (var11 * var11);
      var15 = var4 + 0.14;
      var14 = var14 - (var15 * var15);
      for (int var18 = 0; var18 < 16; var18 = var18 + 1) {
        if (var14[var18] < 0.0) {
          int var19 = 1;
          return var19;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var20 = var7 - 1.13815338075485;
  blaze::StaticVector<double, 16> var21 = var6 - -0.126633131394491;
  blaze::StaticVector<double, 16> var22 = var5 - 0.38;
  blaze::StaticVector<double, 16> var23 = var22 * 0.0;
  blaze::StaticVector<double, 16> var24 = min(max(var23 * 0.03, 1.0), 0.0);
  var22 = var5 - (0.38 + (0.0 * var24));
  blaze::StaticVector<double, 16> var25 = ((var20 * var20) + (var21 * var21)) + (var22 * var22);
  blaze::StaticVector<double, 16> var26 = var4 + 0.14;
  var25 = var25 - (var26 * var26);
  int var27 = 0;
  while ((var27 < 16) && (var8 == 0)) {
    if (var25[var27] < 0.0) {
      var8 = 1;
    } 
    var27 = var27 + 1;
  }
  if (var8) {
    for (int var28 = 0; var28 < var3.size(); var28 = var28 + 1) {
      var20 = var3[var28] - 1.13815338075485;
      var21 = var2[var28] - -0.126633131394491;
      var22 = var1[var28] - 0.38;
      var24 = min(max(var23 * 0.03, 1.0), 0.0);
      var22 = var5 - (0.38 + (0.0 * var24));
      var25 = ((var20 * var20) + (var21 * var21)) + (var22 * var22);
      var26 = var4 + 0.14;
      var25 = var25 - (var26 * var26);
      for (int var29 = 0; var29 < 16; var29 = var29 + 1) {
        if (var25[var29] < 0.0) {
          int var30 = 1;
          return var30;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var31 = var7 - 0.938153380754854;
  blaze::StaticVector<double, 16> var32 = var6 - -0.113736409788935;
  blaze::StaticVector<double, 16> var33 = var5 - 0.38;
  blaze::StaticVector<double, 16> var34 = var33 * 0.0;
  blaze::StaticVector<double, 16> var35 = min(max(var34 * 0.03, 1.0), 0.0);
  var33 = var5 - (0.38 + (0.0 * var35));
  blaze::StaticVector<double, 16> var36 = ((var31 * var31) + (var32 * var32)) + (var33 * var33);
  blaze::StaticVector<double, 16> var37 = var4 + 0.14;
  var36 = var36 - (var37 * var37);
  int var38 = 0;
  while ((var38 < 16) && (var8 == 0)) {
    if (var36[var38] < 0.0) {
      var8 = 1;
    } 
    var38 = var38 + 1;
  }
  if (var8) {
    for (int var39 = 0; var39 < var3.size(); var39 = var39 + 1) {
      var31 = var3[var39] - 0.938153380754854;
      var32 = var2[var39] - -0.113736409788935;
      var33 = var1[var39] - 0.38;
      var35 = min(max(var34 * 0.03, 1.0), 0.0);
      var33 = var5 - (0.38 + (0.0 * var35));
      var36 = ((var31 * var31) + (var32 * var32)) + (var33 * var33);
      var37 = var4 + 0.14;
      var36 = var36 - (var37 * var37);
      for (int var40 = 0; var40 < 16; var40 = var40 + 1) {
        if (var36[var40] < 0.0) {
          int var41 = 1;
          return var41;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var42 = var7 - 1.33815338075485;
  blaze::StaticVector<double, 16> var43 = var6 - -0.00506795943493665;
  blaze::StaticVector<double, 16> var44 = var5 - -0.22;
  blaze::StaticVector<double, 16> var45 = var44 * 0.0;
  blaze::StaticVector<double, 16> var46 = min(max(var45 * 0.03, 1.0), 0.0);
  var44 = var5 - (-0.22 + (0.0 * var46));
  blaze::StaticVector<double, 16> var47 = ((var42 * var42) + (var43 * var43)) + (var44 * var44);
  blaze::StaticVector<double, 16> var48 = var4 + 0.14;
  var47 = var47 - (var48 * var48);
  int var49 = 0;
  while ((var49 < 16) && (var8 == 0)) {
    if (var47[var49] < 0.0) {
      var8 = 1;
    } 
    var49 = var49 + 1;
  }
  if (var8) {
    for (int var50 = 0; var50 < var3.size(); var50 = var50 + 1) {
      var42 = var3[var50] - 1.33815338075485;
      var43 = var2[var50] - -0.00506795943493665;
      var44 = var1[var50] - -0.22;
      var46 = min(max(var45 * 0.03, 1.0), 0.0);
      var44 = var5 - (-0.22 + (0.0 * var46));
      var47 = ((var42 * var42) + (var43 * var43)) + (var44 * var44);
      var48 = var4 + 0.14;
      var47 = var47 - (var48 * var48);
      for (int var51 = 0; var51 < 16; var51 = var51 + 1) {
        if (var47[var51] < 0.0) {
          int var52 = 1;
          return var52;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var53 = var7 - 1.13815338075485;
  blaze::StaticVector<double, 16> var54 = var6 - 0.226670350914742;
  blaze::StaticVector<double, 16> var55 = var5 - -0.22;
  blaze::StaticVector<double, 16> var56 = var55 * 0.0;
  blaze::StaticVector<double, 16> var57 = min(max(var56 * 0.03, 1.0), 0.0);
  var55 = var5 - (-0.22 + (0.0 * var57));
  blaze::StaticVector<double, 16> var58 = ((var53 * var53) + (var54 * var54)) + (var55 * var55);
  blaze::StaticVector<double, 16> var59 = var4 + 0.14;
  var58 = var58 - (var59 * var59);
  int var60 = 0;
  while ((var60 < 16) && (var8 == 0)) {
    if (var58[var60] < 0.0) {
      var8 = 1;
    } 
    var60 = var60 + 1;
  }
  if (var8) {
    for (int var61 = 0; var61 < var3.size(); var61 = var61 + 1) {
      var53 = var3[var61] - 1.13815338075485;
      var54 = var2[var61] - 0.226670350914742;
      var55 = var1[var61] - -0.22;
      var57 = min(max(var56 * 0.03, 1.0), 0.0);
      var55 = var5 - (-0.22 + (0.0 * var57));
      var58 = ((var53 * var53) + (var54 * var54)) + (var55 * var55);
      var59 = var4 + 0.14;
      var58 = var58 - (var59 * var59);
      for (int var62 = 0; var62 < 16; var62 = var62 + 1) {
        if (var58[var62] < 0.0) {
          int var63 = 1;
          return var63;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var64 = var7 - 0.938153380754854;
  blaze::StaticVector<double, 16> var65 = var6 - 0.272131034053783;
  blaze::StaticVector<double, 16> var66 = var5 - -0.22;
  blaze::StaticVector<double, 16> var67 = var66 * 0.0;
  blaze::StaticVector<double, 16> var68 = min(max(var67 * 0.03, 1.0), 0.0);
  var66 = var5 - (-0.22 + (0.0 * var68));
  blaze::StaticVector<double, 16> var69 = ((var64 * var64) + (var65 * var65)) + (var66 * var66);
  blaze::StaticVector<double, 16> var70 = var4 + 0.14;
  var69 = var69 - (var70 * var70);
  int var71 = 0;
  while ((var71 < 16) && (var8 == 0)) {
    if (var69[var71] < 0.0) {
      var8 = 1;
    } 
    var71 = var71 + 1;
  }
  if (var8) {
    for (int var72 = 0; var72 < var3.size(); var72 = var72 + 1) {
      var64 = var3[var72] - 0.938153380754854;
      var65 = var2[var72] - 0.272131034053783;
      var66 = var1[var72] - -0.22;
      var68 = min(max(var67 * 0.03, 1.0), 0.0);
      var66 = var5 - (-0.22 + (0.0 * var68));
      var69 = ((var64 * var64) + (var65 * var65)) + (var66 * var66);
      var70 = var4 + 0.14;
      var69 = var69 - (var70 * var70);
      for (int var73 = 0; var73 < 16; var73 = var73 + 1) {
        if (var69[var73] < 0.0) {
          int var74 = 1;
          return var74;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var75 = var7 - 1.33815338075485;
  blaze::StaticVector<double, 16> var76 = var6 - 0.293848515958824;
  blaze::StaticVector<double, 16> var77 = var5 - 0.0800000000000001;
  blaze::StaticVector<double, 16> var78 = var77 * 0.0;
  blaze::StaticVector<double, 16> var79 = min(max(var78 * 0.03, 1.0), 0.0);
  var77 = var5 - (0.0800000000000001 + (0.0 * var79));
  blaze::StaticVector<double, 16> var80 = ((var75 * var75) + (var76 * var76)) + (var77 * var77);
  blaze::StaticVector<double, 16> var81 = var4 + 0.14;
  var80 = var80 - (var81 * var81);
  int var82 = 0;
  while ((var82 < 16) && (var8 == 0)) {
    if (var80[var82] < 0.0) {
      var8 = 1;
    } 
    var82 = var82 + 1;
  }
  if (var8) {
    for (int var83 = 0; var83 < var3.size(); var83 = var83 + 1) {
      var75 = var3[var83] - 1.33815338075485;
      var76 = var2[var83] - 0.293848515958824;
      var77 = var1[var83] - 0.0800000000000001;
      var79 = min(max(var78 * 0.03, 1.0), 0.0);
      var77 = var5 - (0.0800000000000001 + (0.0 * var79));
      var80 = ((var75 * var75) + (var76 * var76)) + (var77 * var77);
      var81 = var4 + 0.14;
      var80 = var80 - (var81 * var81);
      for (int var84 = 0; var84 < 16; var84 = var84 + 1) {
        if (var80[var84] < 0.0) {
          int var85 = 1;
          return var85;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var86 = var7 - 1.13815338075485;
  blaze::StaticVector<double, 16> var87 = var6 - -0.116911587286137;
  blaze::StaticVector<double, 16> var88 = var5 - 0.0800000000000001;
  blaze::StaticVector<double, 16> var89 = var88 * 0.0;
  blaze::StaticVector<double, 16> var90 = min(max(var89 * 0.03, 1.0), 0.0);
  var88 = var5 - (0.0800000000000001 + (0.0 * var90));
  blaze::StaticVector<double, 16> var91 = ((var86 * var86) + (var87 * var87)) + (var88 * var88);
  blaze::StaticVector<double, 16> var92 = var4 + 0.14;
  var91 = var91 - (var92 * var92);
  int var93 = 0;
  while ((var93 < 16) && (var8 == 0)) {
    if (var91[var93] < 0.0) {
      var8 = 1;
    } 
    var93 = var93 + 1;
  }
  if (var8) {
    for (int var94 = 0; var94 < var3.size(); var94 = var94 + 1) {
      var86 = var3[var94] - 1.13815338075485;
      var87 = var2[var94] - -0.116911587286137;
      var88 = var1[var94] - 0.0800000000000001;
      var90 = min(max(var89 * 0.03, 1.0), 0.0);
      var88 = var5 - (0.0800000000000001 + (0.0 * var90));
      var91 = ((var86 * var86) + (var87 * var87)) + (var88 * var88);
      var92 = var4 + 0.14;
      var91 = var91 - (var92 * var92);
      for (int var95 = 0; var95 < 16; var95 = var95 + 1) {
        if (var91[var95] < 0.0) {
          int var96 = 1;
          return var96;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var97 = var7 - 0.938153380754854;
  blaze::StaticVector<double, 16> var98 = var6 - -0.221514909893898;
  blaze::StaticVector<double, 16> var99 = var5 - 0.0800000000000001;
  blaze::StaticVector<double, 16> var100 = var99 * 0.0;
  blaze::StaticVector<double, 16> var101 = min(max(var100 * 0.03, 1.0), 0.0);
  var99 = var5 - (0.0800000000000001 + (0.0 * var101));
  blaze::StaticVector<double, 16> var102 = ((var97 * var97) + (var98 * var98)) + (var99 * var99);
  blaze::StaticVector<double, 16> var103 = var4 + 0.14;
  var102 = var102 - (var103 * var103);
  int var104 = 0;
  while ((var104 < 16) && (var8 == 0)) {
    if (var102[var104] < 0.0) {
      var8 = 1;
    } 
    var104 = var104 + 1;
  }
  if (var8) {
    for (int var105 = 0; var105 < var3.size(); var105 = var105 + 1) {
      var97 = var3[var105] - 0.938153380754854;
      var98 = var2[var105] - -0.221514909893898;
      var99 = var1[var105] - 0.0800000000000001;
      var101 = min(max(var100 * 0.03, 1.0), 0.0);
      var99 = var5 - (0.0800000000000001 + (0.0 * var101));
      var102 = ((var97 * var97) + (var98 * var98)) + (var99 * var99);
      var103 = var4 + 0.14;
      var102 = var102 - (var103 * var103);
      for (int var106 = 0; var106 < 16; var106 = var106 + 1) {
        if (var102[var106] < 0.0) {
          int var107 = 1;
          return var107;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var108 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var109 = var6 - -0.0131415190484088;
  blaze::StaticVector<double, 16> var110 = var5 - -0.3;
  blaze::StaticVector<double, 16> var111 = max(abs(var108) - 1.2, 0.0);
  blaze::StaticVector<double, 16> var112 = max(abs(var109) - 1.0, 0.0);
  blaze::StaticVector<double, 16> var113 = max(abs(var110) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var114 = ((var111 * var111) + (var112 * var112)) + (var113 * var113);
  var114 = var114 - (var4 * var4);
  int var115 = 0;
  while ((var115 < 16) && (var8 == 0)) {
    if (var114[var115] < 0.0) {
      var8 = 1;
    } 
    var115 = var115 + 1;
  }
  if (var8) {
    for (int var116 = 0; var116 < var3.size(); var116 = var116 + 1) {
      var108 = var3[var116] - 1.43815338075485;
      var109 = var2[var116] - -0.0131415190484088;
      var110 = var1[var116] - -0.3;
      var111 = max(abs(var108) - 1.2, 0.0);
      var112 = max(abs(var109) - 1.0, 0.0);
      var113 = max(abs(var110) - 0.04, 0.0);
      var114 = ((var111 * var111) + (var112 * var112)) + (var113 * var113);
      var114 = var114 - (var4 * var4);
      for (int var117 = 0; var117 < 16; var117 = var117 + 1) {
        if (var114[var117] < 0.0) {
          int var118 = 1;
          return var118;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var119 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var120 = var6 - -0.0131415190484088;
  blaze::StaticVector<double, 16> var121 = var5 - 0.0;
  blaze::StaticVector<double, 16> var122 = max(abs(var119) - 1.2, 0.0);
  blaze::StaticVector<double, 16> var123 = max(abs(var120) - 1.0, 0.0);
  blaze::StaticVector<double, 16> var124 = max(abs(var121) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var125 = ((var122 * var122) + (var123 * var123)) + (var124 * var124);
  var125 = var125 - (var4 * var4);
  int var126 = 0;
  while ((var126 < 16) && (var8 == 0)) {
    if (var125[var126] < 0.0) {
      var8 = 1;
    } 
    var126 = var126 + 1;
  }
  if (var8) {
    for (int var127 = 0; var127 < var3.size(); var127 = var127 + 1) {
      var119 = var3[var127] - 1.43815338075485;
      var120 = var2[var127] - -0.0131415190484088;
      var121 = var1[var127] - 0.0;
      var122 = max(abs(var119) - 1.2, 0.0);
      var123 = max(abs(var120) - 1.0, 0.0);
      var124 = max(abs(var121) - 0.04, 0.0);
      var125 = ((var122 * var122) + (var123 * var123)) + (var124 * var124);
      var125 = var125 - (var4 * var4);
      for (int var128 = 0; var128 < 16; var128 = var128 + 1) {
        if (var125[var128] < 0.0) {
          int var129 = 1;
          return var129;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var130 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var131 = var6 - -0.0131415190484088;
  blaze::StaticVector<double, 16> var132 = var5 - 0.3;
  blaze::StaticVector<double, 16> var133 = max(abs(var130) - 1.2, 0.0);
  blaze::StaticVector<double, 16> var134 = max(abs(var131) - 1.0, 0.0);
  blaze::StaticVector<double, 16> var135 = max(abs(var132) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var136 = ((var133 * var133) + (var134 * var134)) + (var135 * var135);
  var136 = var136 - (var4 * var4);
  int var137 = 0;
  while ((var137 < 16) && (var8 == 0)) {
    if (var136[var137] < 0.0) {
      var8 = 1;
    } 
    var137 = var137 + 1;
  }
  if (var8) {
    for (int var138 = 0; var138 < var3.size(); var138 = var138 + 1) {
      var130 = var3[var138] - 1.43815338075485;
      var131 = var2[var138] - -0.0131415190484088;
      var132 = var1[var138] - 0.3;
      var133 = max(abs(var130) - 1.2, 0.0);
      var134 = max(abs(var131) - 1.0, 0.0);
      var135 = max(abs(var132) - 0.04, 0.0);
      var136 = ((var133 * var133) + (var134 * var134)) + (var135 * var135);
      var136 = var136 - (var4 * var4);
      for (int var139 = 0; var139 < 16; var139 = var139 + 1) {
        if (var136[var139] < 0.0) {
          int var140 = 1;
          return var140;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var141 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var142 = var6 - -0.0131415190484088;
  blaze::StaticVector<double, 16> var143 = var5 - 0.6;
  blaze::StaticVector<double, 16> var144 = max(abs(var141) - 1.2, 0.0);
  blaze::StaticVector<double, 16> var145 = max(abs(var142) - 1.0, 0.0);
  blaze::StaticVector<double, 16> var146 = max(abs(var143) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var147 = ((var144 * var144) + (var145 * var145)) + (var146 * var146);
  var147 = var147 - (var4 * var4);
  int var148 = 0;
  while ((var148 < 16) && (var8 == 0)) {
    if (var147[var148] < 0.0) {
      var8 = 1;
    } 
    var148 = var148 + 1;
  }
  if (var8) {
    for (int var149 = 0; var149 < var3.size(); var149 = var149 + 1) {
      var141 = var3[var149] - 1.43815338075485;
      var142 = var2[var149] - -0.0131415190484088;
      var143 = var1[var149] - 0.6;
      var144 = max(abs(var141) - 1.2, 0.0);
      var145 = max(abs(var142) - 1.0, 0.0);
      var146 = max(abs(var143) - 0.04, 0.0);
      var147 = ((var144 * var144) + (var145 * var145)) + (var146 * var146);
      var147 = var147 - (var4 * var4);
      for (int var150 = 0; var150 < 16; var150 = var150 + 1) {
        if (var147[var150] < 0.0) {
          int var151 = 1;
          return var151;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var152 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var153 = var6 - -0.513141519048409;
  blaze::StaticVector<double, 16> var154 = var5 - -0.18;
  blaze::StaticVector<double, 16> var155 = max(abs(var152) - 1.3, 0.0);
  blaze::StaticVector<double, 16> var156 = max(abs(var153) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var157 = max(abs(var154) - 1.64, 0.0);
  blaze::StaticVector<double, 16> var158 = ((var155 * var155) + (var156 * var156)) + (var157 * var157);
  var158 = var158 - (var4 * var4);
  int var159 = 0;
  while ((var159 < 16) && (var8 == 0)) {
    if (var158[var159] < 0.0) {
      var8 = 1;
    } 
    var159 = var159 + 1;
  }
  if (var8) {
    for (int var160 = 0; var160 < var3.size(); var160 = var160 + 1) {
      var152 = var3[var160] - 1.43815338075485;
      var153 = var2[var160] - -0.513141519048409;
      var154 = var1[var160] - -0.18;
      var155 = max(abs(var152) - 1.3, 0.0);
      var156 = max(abs(var153) - 0.04, 0.0);
      var157 = max(abs(var154) - 1.64, 0.0);
      var158 = ((var155 * var155) + (var156 * var156)) + (var157 * var157);
      var158 = var158 - (var4 * var4);
      for (int var161 = 0; var161 < 16; var161 = var161 + 1) {
        if (var158[var161] < 0.0) {
          int var162 = 1;
          return var162;
        } 
      }
    }
    var8 = 0;
  } 
  blaze::StaticVector<double, 16> var163 = var7 - 1.43815338075485;
  blaze::StaticVector<double, 16> var164 = var6 - 0.486858480951591;
  blaze::StaticVector<double, 16> var165 = var5 - -0.18;
  blaze::StaticVector<double, 16> var166 = max(abs(var163) - 1.3, 0.0);
  blaze::StaticVector<double, 16> var167 = max(abs(var164) - 0.04, 0.0);
  blaze::StaticVector<double, 16> var168 = max(abs(var165) - 1.64, 0.0);
  blaze::StaticVector<double, 16> var169 = ((var166 * var166) + (var167 * var167)) + (var168 * var168);
  var169 = var169 - (var4 * var4);
  int var170 = 0;
  while ((var170 < 16) && (var8 == 0)) {
    if (var169[var170] < 0.0) {
      var8 = 1;
    } 
    var170 = var170 + 1;
  }
  if (var8) {
    for (int var171 = 0; var171 < var3.size(); var171 = var171 + 1) {
      var163 = var3[var171] - 1.43815338075485;
      var164 = var2[var171] - 0.486858480951591;
      var165 = var1[var171] - -0.18;
      var166 = max(abs(var163) - 1.3, 0.0);
      var167 = max(abs(var164) - 0.04, 0.0);
      var168 = max(abs(var165) - 1.64, 0.0);
      var169 = ((var166 * var166) + (var167 * var167)) + (var168 * var168);
      var169 = var169 - (var4 * var4);
      for (int var172 = 0; var172 < 16; var172 = var172 + 1) {
        if (var169[var172] < 0.0) {
          int var173 = 1;
          return var173;
        } 
      }
    }
    var8 = 0;
    var174 = 0;
    return var174;
  } 
  var174 = 0;
  return var174;
}

}
