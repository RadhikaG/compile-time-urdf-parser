#ifndef XFORM_OPERATORS_H
#define XFORM_OPERATORS_H

#include "xform_impl.h"
#include <type_traits>

namespace ctup {

/*** Xform expressions **/

// type trait for checking if expr types are compatible with each other
template <typename Scalar>
struct is_acceptable_rhs_xform {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_xform<Xform<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Xform_expr<Scalar>& cast(const Xform<Scalar>& v) {
    return *new Xform_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_xform<Xform_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Xform_expr<Scalar>& cast(const Xform_expr<Scalar>& v) {
    return v;
  }
};

template<typename E1, typename E2>
// sets return type to Xform_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_xform<E2>::value, 
         const Xform_expr<typename is_acceptable_rhs_xform<E1>::scalar_type>&>::type
// ...for xform * xform
operator * (const E1& v1, const E2& v2) {
  return *new Xform_expr_mul<typename is_acceptable_rhs_xform<E1>::scalar_type>(
          is_acceptable_rhs_xform<E1>::cast(v1),
          is_acceptable_rhs_xform<E1>::cast(v2));
}

/*** Translation expressions **/

template <typename Scalar>
struct is_acceptable_rhs_translation {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_translation<Translation<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Translation_expr<Scalar>& cast(const Translation<Scalar>& v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_translation<Translation_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Translation_expr<Scalar>& cast(const Translation_expr<Scalar>& v) {
    return v;
  }
};

template<typename E1, typename E2>
// sets return type to Translation_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_translation<E2>::value, 
         const Translation_expr<typename is_acceptable_rhs_translation<E1>::scalar_type>&>::type
// ...for translation * translation
operator + (const E1& v1, const E2& v2) {
  return *new Translation_expr_add<typename is_acceptable_rhs_translation<E1>::scalar_type>(
          is_acceptable_rhs_translation<E1>::cast(v1),
          is_acceptable_rhs_translation<E1>::cast(v2));
}

/*** Rotation expressions **/

template <typename Scalar>
struct is_acceptable_rhs_rotation {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_rotation<Rotation<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Rotation_expr<Scalar>& cast(const Rotation<Scalar>& v) {
    return *new Rotation_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_rotation<Rotation_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Rotation_expr<Scalar>& cast(const Rotation_expr<Scalar>& v) {
    return v;
  }
};

template<typename E1, typename E2>
// sets return type to Rotation_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_rotation<E2>::value, 
         const Rotation_expr<typename is_acceptable_rhs_rotation<E1>::scalar_type>&>::type
// ...for rotation * rotation
operator + (const E1& v1, const E2& v2) {
  return *new Rotation_expr_mul<typename is_acceptable_rhs_rotation<E1>::scalar_type>(
          is_acceptable_rhs_rotation<E1>::cast(v1),
          is_acceptable_rhs_rotation<E1>::cast(v2));
}

}

#endif
