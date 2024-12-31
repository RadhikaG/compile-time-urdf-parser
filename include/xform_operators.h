#ifndef XFORM_OPERATORS_H
#define XFORM_OPERATORS_H

#include "xform_impl.h"
#include <type_traits>

namespace ctup {

/*** Xform expressions **/

template <typename T>
struct is_Matrix : std::false_type {};

template <typename T,int _Rows, int _Col>
struct is_Matrix<ctup::EigenMatrix<T, _Rows, _Col>> : std::true_type {};

template <typename T>
inline constexpr bool is_Matrix_m = is_Matrix<T>::value;

template <typename T>
struct inner_type { 
  using type = T;
};

template <typename T,int _Rows, int _Col>
struct inner_type<ctup::EigenMatrix<T, _Rows, _Col>> {
    using type = T;
};

template <typename T>
using inner_type_t = typename inner_type<T>::type;

// type trait for checking if expr types are compatible with each other
template <typename Scalar>
struct is_acceptable_rhs_xform {
  static const bool value = false;
  static const bool matrix = false;
};

template <typename Scalar> //CHANGE
struct is_acceptable_rhs_xform<Xform<Scalar>> {
  static const bool value = true;
  static const bool matrix = is_Matrix_m<Scalar>;
  using scalar_type =  inner_type_t<Scalar>;
  using ins_type =  Scalar;
  static const Xform_expr<Scalar> &cast(const Xform<Scalar> &v) {
    return *new Xform_expr_leaf<Scalar>(v);
  }
};

template <typename Scalar> //CHANGE
struct is_acceptable_rhs_xform<Xform_expr<Scalar>> {
  static const bool value = true;
  static const bool matrix = is_Matrix_m<Scalar>;
  using scalar_type =  inner_type_t<Scalar>;
  using ins_type =  Scalar;
  static const Xform_expr<Scalar> &cast(const Xform_expr<Scalar> &v) {
    return v;
  }
};
// OPERATOR * FOR XFORM

template <typename E1, typename E2>
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_xform<E2>::value && not(is_acceptable_rhs_xform<E1>::matrix|| is_acceptable_rhs_xform<E2>::matrix) ,
                        const Xform_expr<typename is_acceptable_rhs_xform<E1>::ins_type> &>::type
// ...for xform * xform
operator*(const E1 &v1, const E2 &v2) {
    return *new Xform_expr_mul<typename is_acceptable_rhs_xform<E1>::ins_type>(
        is_acceptable_rhs_xform<E1>::cast(v1), is_acceptable_rhs_xform<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_xform<E2>::value && is_acceptable_rhs_xform<E1>::matrix && is_acceptable_rhs_xform<E2>::matrix,
                        const Xform_expr<typename is_acceptable_rhs_xform<E1>::ins_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Xform_expr_mul<typename is_acceptable_rhs_xform<E1>::ins_type>(
      is_acceptable_rhs_xform<E1>::cast(v1), is_acceptable_rhs_xform<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_xform<E2>::value && is_acceptable_rhs_xform<E1>::matrix && not(is_acceptable_rhs_xform<E2>::matrix), //&& std::is_same<is_acceptable_rhs_xform<E1>::scalar_type , is_acceptable_rhs_xform<E2>::ins_type>::value,
                        const Xform_expr<typename is_acceptable_rhs_xform<E1>::ins_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Xform_expr_mul_sec<typename is_acceptable_rhs_xform<E1>::ins_type,typename is_acceptable_rhs_xform<E2>::ins_type>(
      is_acceptable_rhs_xform<E1>::cast(v1), is_acceptable_rhs_xform<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_xform<E1>::value && is_acceptable_rhs_xform<E2>::matrix && not(is_acceptable_rhs_xform<E1>::matrix),// && is_acceptable_rhs_xform<E2>::ins_type == is_acceptable_rhs_xform<E1>::scalar_type,
                        const Xform_expr<typename is_acceptable_rhs_xform<E2>::ins_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Xform_expr_mul_sec<typename is_acceptable_rhs_xform<E2>::ins_type,typename is_acceptable_rhs_xform<E1>::ins_type>(
      is_acceptable_rhs_xform<E1>::cast(v1), is_acceptable_rhs_xform<E2>::cast(v2));
}

/*** Translation expressions **/

template <typename Scalar> //CHANGE
struct is_acceptable_rhs_translation {
  static const bool value = false;
  static const bool matrix = false;
};

template <typename Scalar> //CHANGE
struct is_acceptable_rhs_translation<Translation<Scalar>> {
  static const bool value = true;

  static const bool matrix = is_Matrix_m<Scalar>;
  using scalar_type =  inner_type_t<Scalar>;
  using ins_type =  Scalar;

  static const Translation_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar> //CHANGE
struct is_acceptable_rhs_translation<Translation_expr<Scalar>> {
  static const bool value = true;

  static const bool matrix = is_Matrix_m<Scalar>;
  using scalar_type =  inner_type_t<Scalar>;
  using ins_type =  Scalar;

  static const Translation_expr<Scalar> &cast(const Translation_expr<Scalar> &v) {
    return v;
  }
};

template <typename E1, typename E2>
// sets return type to Translation_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_translation<E2>::value && not(is_acceptable_rhs_translation<E1>::matrix || is_acceptable_rhs_translation<E2>::matrix),
    const Translation_expr<typename is_acceptable_rhs_translation<E1>::scalar_type> &>::type
// ...for translation + translation
operator+(const E1 &v1, const E2 &v2) {
  return *new Translation_expr_add<typename is_acceptable_rhs_translation<E1>::scalar_type>(
      is_acceptable_rhs_translation<E1>::cast(v1), is_acceptable_rhs_translation<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
typename std::enable_if<
    is_acceptable_rhs_translation<E2>::value && is_acceptable_rhs_translation<E1>::matrix && is_acceptable_rhs_translation<E2>::matrix,
    const Translation_expr<typename is_acceptable_rhs_translation<E1>::ins_type> &>::type
// ...for translation + translation
operator+(const E1 &v1, const E2 &v2) {
  return *new Translation_expr_add<typename is_acceptable_rhs_translation<E1>::ins_type>(
      is_acceptable_rhs_translation<E1>::cast(v1), is_acceptable_rhs_translation<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_translation<E2>::value && is_acceptable_rhs_translation<E1>::matrix && not(is_acceptable_rhs_translation<E2>::matrix), //&& std::is_same<is_acceptable_rhs_translation<E1>::scalar_type , is_acceptable_rhs_translation<E2>::ins_type>::value,
                        const Xform_expr<typename is_acceptable_rhs_translation<E1>::ins_type> &>::type
operator+(const E1 &v1, const E2 &v2) {
  return *new Translation_expr_add_sec<typename is_acceptable_rhs_translation<E1>::ins_type,typename is_acceptable_rhs_translation<E2>::ins_type>(
      is_acceptable_rhs_translation<E1>::cast(v1), is_acceptable_rhs_translation<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_translation<E1>::value && is_acceptable_rhs_translation<E2>::matrix && not(is_acceptable_rhs_translation<E1>::matrix),// && is_acceptable_rhs_translation<E2>::ins_type == is_acceptable_rhs_translation<E1>::scalar_type,
                        const Xform_expr<typename is_acceptable_rhs_translation<E2>::ins_type> &>::type
operator+(const E1 &v1, const E2 &v2) {
  return *new Translation_expr_add_sec<typename is_acceptable_rhs_translation<E2>::ins_type,typename is_acceptable_rhs_translation<E1>::ins_type>(
      is_acceptable_rhs_translation<E1>::cast(v1), is_acceptable_rhs_translation<E2>::cast(v2));
}
/*** Rotation expressions **/

//template <typename Scalar>
//struct is_acceptable_rhs_rotation {
//  static const bool value = false;
//};
//
//template <typename Scalar>
//struct is_acceptable_rhs_rotation<Rotation<Scalar>> {
//  static const bool value = true;
//  using scalar_type = Scalar;
//  static const Rotation_expr<Scalar>& cast(const Rotation<Scalar>& v) {
//    return *new Rotation_expr_leaf<Scalar>(v);
//  }
//};
//template <typename Scalar>
//struct is_acceptable_rhs_rotation<Rotation_expr<Scalar>> {
//  static const bool value = true;
//  using scalar_type = Scalar;
//  static const Rotation_expr<Scalar>& cast(const Rotation_expr<Scalar>& v) {
//    return v;
//  }
//};
//
//template<typename E1, typename E2>
//// sets return type to Rotation_expr<Scalar>...
//typename std::enable_if<is_acceptable_rhs_rotation<E2>::value,
//         const Rotation_expr<typename is_acceptable_rhs_rotation<E1>::scalar_type>&>::type
//// ...for rotation * rotation
//operator * (const E1& v1, const E2& v2) {
//  return *new Rotation_expr_mul<typename is_acceptable_rhs_rotation<E1>::scalar_type>(
//          is_acceptable_rhs_rotation<E1>::cast(v1),
//          is_acceptable_rhs_rotation<E2>::cast(v2));
//}

/** Matrix expressions **/

// todo / warning
// this entire section is hacked together
// need to figure out good interface to matrix class

template <typename T, typename = void>
struct is_acceptable_rhs_matrix {
  static const bool value = false;
  static const bool matrix = false;
};

template <template <typename> class T, typename Scalar>
struct is_acceptable_rhs_matrix<T<Scalar>> {
  static const bool value = std::is_base_of<Matrix<Scalar>, T<Scalar>>::value ||
                            std::is_base_of<Matrix_expr<Scalar>, T<Scalar>>::value;

  static const bool matrix = is_Matrix_m<Scalar>;

  using scalar_type = inner_type_t<Scalar>;
  using in_type =  Scalar;

  static const Matrix_expr<Scalar> &cast(const Matrix<Scalar> &v) {
    return *new Matrix_expr_leaf<Scalar>(v);
  }
  static const Matrix_expr<Scalar> &cast(const Matrix_expr<Scalar> &v) {
    return v;
  }
};

// todo: dealing with translation separately because translation needs to be migrated to
// storage, but translation expr is a matrix expr.
template <typename Scalar>
struct is_acceptable_rhs_matrix<Translation<Scalar>> {
  static const bool value = true;

  static const bool matrix = is_Matrix_m<Scalar>;

  using scalar_type = inner_type_t<Scalar>;
  using in_type =  Scalar;

  static const Matrix_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};

// OPERATOR * FOR MATRIX

template <typename E1, typename E2> //CHANGE
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E2>::value && not(is_acceptable_rhs_matrix<E1>::matrix|| is_acceptable_rhs_matrix<E2>::matrix),
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator*(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_mul<typename is_acceptable_rhs_matrix<E1>::type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// 
typename std::enable_if<
  is_acceptable_rhs_matrix<E2>::value && is_acceptable_rhs_matrix<E1>::matrix && is_acceptable_rhs_matrix<E2>::matrix,
  const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::in_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_mul<typename is_acceptable_rhs_matrix<E1>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix<E2>::value && is_acceptable_rhs_matrix<E1>::matrix && not(is_acceptable_rhs_matrix<E2>::matrix), //&& is_acceptable_rhs_matrix<E1>::scalar_type == is_acceptable_rhs_matrix<E2>::scalar_type,
                        const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::in_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_mul_sec<typename is_acceptable_rhs_matrix<E1>::in_type,typename is_acceptable_rhs_matrix<E2>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_matrix<E1>::value && is_acceptable_rhs_matrix<E2>::matrix && not(is_acceptable_rhs_matrix<E1>::matrix),// && is_acceptable_rhs_matrix<E2>::scalar_type == is_acceptable_rhs_matrix<E1>::scalar_type,
                        const Matrix_expr<typename is_acceptable_rhs_matrix<E2>::in_type> &>::type
operator*(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_mul_sec<typename is_acceptable_rhs_matrix<E2>::in_type,typename is_acceptable_rhs_matrix<E1>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

// OPERATOR - FOR MATRIX

template <typename E1> //CHANGE
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E1>::value,
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::in_type> &>::type
// ...for - matrix
operator-(const E1 &v1) {
  return *new Matrix_expr_unary_minus<typename is_acceptable_rhs_matrix<E1>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1));
}

// OPERATOR + FOR MATRIX

template <typename E1, typename E2>
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E2>::value && not(is_acceptable_rhs_matrix<E1>::matrix|| is_acceptable_rhs_matrix<E2>::matrix),
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator+(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_add<typename is_acceptable_rhs_matrix<E1>::scalar_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
//
typename std::enable_if<
  is_acceptable_rhs_matrix<E2>::value && is_acceptable_rhs_matrix<E1>::matrix && is_acceptable_rhs_matrix<E2>::matrix,
  const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::in_type> &>::type
operator+(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_add<typename is_acceptable_rhs_matrix<E1>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
typename std::enable_if<is_acceptable_rhs_matrix<E2>::value && is_acceptable_rhs_matrix<E1>::matrix && not(is_acceptable_rhs_matrix<E2>::matrix), //&& is_acceptable_rhs_matrix<E1>::scalar_type == is_acceptable_rhs_matrix<E2>::scalar_type,
                        const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::in_type> &>::type
operator+(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_add_sec<typename is_acceptable_rhs_matrix<E1>::in_type,typename is_acceptable_rhs_matrix<E2>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1, typename E2> //NEW
typename std::enable_if<is_acceptable_rhs_matrix<E1>::value && is_acceptable_rhs_matrix<E2>::matrix && not(is_acceptable_rhs_matrix<E1>::matrix),// && is_acceptable_rhs_matrix<E2>::scalar_type == is_acceptable_rhs_matrix<E1>::scalar_type,
                        const Matrix_expr<typename is_acceptable_rhs_matrix<E2>::in_type> &>::type
operator+(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_add_sec<typename is_acceptable_rhs_matrix<E2>::in_type,typename is_acceptable_rhs_matrix<E1>::in_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

} // namespace ctup

#endif
