#ifndef SPATIALALGEBRA_H
#define SPATIALALGEBRA_H

#include "cscalar_impl.h"
#include "cmatrix_impl.h"
#include "cscalar_operators.h"
#include "cmatrix_operators.h"
#include "blocks/extract_cuda.h"
#include "blocks/c_code_generator.h"
#include "blocks/rce.h"

namespace SpatialAlgebra {

template <typename F, typename...OtherArgs>
void generate_spatial_algebra_program(F func_input, std::string fname, std::ostream &oss, OtherArgs...args) {
	builder::builder_context ctx;
#ifdef ENABLE_D2X
	ctx.enable_d2x = true;
#endif
	auto ast = ctx.extract_function_ast(func_input, fname, args...);
	block::eliminate_redundant_vars(ast);
    ast->dump(std::cout, 0);
	auto new_decls = block::extract_cuda_from(block::to<block::func_decl>(ast)->body);
	oss << "#include<stdlib.h>\n";
#ifdef ENABLE_D2X
	oss << "#include \"d2x_runtime/d2x_runtime.h\"\n";
#endif
	for (auto a : new_decls)
		block::c_code_generator::generate_code(a, oss, 0);
#ifdef ENABLE_D2X
	block::c_code_generator::generate_code_d2x(ast, oss, 0);
#else
	block::c_code_generator::generate_code(ast, oss, 0);
#endif
}

}

#endif
