### Build notes for xform DSL

1. Get pinocchio built in deps/
2. There are three code generators of interest:
    * sample8: Generates `DENSE` unrolled matrix multiplies in FK order. (eigen matmul statements)
    * sample9: Generates `SPARSE` unrolled constant-propagated scalar add-muls for matmuls in FK order.
    * sample10: Generates `SPARSE` unrolled scalar add-muls for a small testcase to ensure correctness (todo: decouple this from pinocchio).
3. Running sample[8|9|10] generates an fk\_gen.h file with the generated code.
4. This fk\_gen.h needs to be hooked into another driver script that actually runs the fk function sitting inside fk\_gen.h.
5. The driver script for sample[8|9] is ctup-experiments/scripts/gen\_fk.cpp
6. The driver script for sample10 is apps/sample10/driver.cpp

**sample[8|9]**:

1. Build code generator: `$ make`
2. sample8: Run code generator to generate fk\_gen.h in driver dir:

    `$ ./build/sample8 ./deps/pinocchio/models/baxter_simple.urdf ../ctup-experiments/gen/fk_gen_dense.h`

3. sample9: Run code generator to generate fk\_gen.h in driver dir: 

    `$ ./build/sample9 ./deps/pinocchio/models/baxter_simple.urdf ../ctup-experiments/gen/fk_gen_unrolled.h`

4. Go to ctup-experiments: `$ cd ../ctup-experiments`
5. In scripts/gen\_fk.cpp: make sure the right header file is included: `#include "fk_gen_dense.h` (for sample8) or `#include "fk_gen_unrolled.h` (sample9)
6. Build driver code with fk\_gen.h: `$ cd build && make`
7. Run driver to compare outputs and evaluate: `$ ./gen_fk ../deps/pinocchio/models/baxter_simple.urdf`

**sample10**:

1. Build code generator: `$ make`
2. Run code generator to generate fk\_gen.h in driver dir: `$ ./build/sample10 deps/pinocchio/models/baxter_simple.urdf apps/sample10/fk_gen.h`
3. Build driver code with fk\_gen.h: `$ make`
4. Run driver to compare outputs and evaluate: `$ ./build/driver deps/pinocchio/models/baxter_simple.urdf`


# BuildIt DSL Template 

This is a template repository for creating a DSL with BuildIt. The repository has BuildIt as a submodule and the appropriate Makefiles to compile the DSL. 

Follow the following steps to create a new DSL repo - 

1. Click "Use this template" to create a new repo with the template files
2. Clone the created repo with the recursive flag -

   `git clone --recursive <remote url for your new repo>`

3. Update BuildIt version if not up to date by navigating to `deps/buildit` and running `git pull`
4. Edit Makefile to set the LIBRARY_NAME variable. The library for your DSL will be created with the name lib<name>.a
5. Edit the files under include/ and src/ to implement your DSL.
6. Working samples can be added under samples/ which will be compiled and linked appropriately.
7. The build system takes a single make variable DEBUG=0 or DEBUG=1 and recompiles BuildIt appropriately if needed. This variable can either be added at the command line as `make DEBUG=1` or can be added to a file named Makefile.inc in the top level directory.


This template does not contain a LICENSE file. BuildIt is available under the MIT License and is our recommendation for your DSL. 
