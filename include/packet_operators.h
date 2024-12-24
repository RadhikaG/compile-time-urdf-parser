#ifndef PACKET_OPERATORS_H
#define PACKET_OPERATORS_H

#include "packet_impl.h"
#include <type_traits>

namespace ctup {

/*** Packet expressions **/

// type trait for checking if expr types are compatible with each other
template <typename Scalar>
struct is_acceptable_rhs_packet {
  static const bool value = false;
};

template <typename Scalar>
struct is_acceptable_rhs_packet<Packet<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Packet_expr<Scalar> &cast(const Packet<Scalar> &v) {
    return *new Packet_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_packet<Packet_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Packet_expr<Scalar> &cast(const Packet_expr<Scalar> &v) {
    return v;
  }
};

template <typename E1, typename E2>
// sets return type to Packet_expr<Scalar>...
typename std::enable_if<is_acceptable_rhs_packet<E2>::value,
                        const Packet_expr<typename is_acceptable_rhs_packet<E1>::scalar_type> &>::type
// ...for packet * packet
operator*(const E1 &v1, const E2 &v2) {
  return *new Packet_expr_mul<typename is_acceptable_rhs_packet<E1>::scalar_type>(
      is_acceptable_rhs_packet<E1>::cast(v1), is_acceptable_rhs_packet<E2>::cast(v2));
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
  static const Translation_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};
template <typename Scalar>
struct is_acceptable_rhs_translation<Translation_expr<Scalar>> {
  static const bool value = true;
  using scalar_type = Scalar;
  static const Translation_expr<Scalar> &cast(const Translation_expr<Scalar> &v) {
    return v;
  }
};

template <typename E1, typename E2>
// sets return type to Translation_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_translation<E2>::value,
    const Translation_expr<typename is_acceptable_rhs_translation<E1>::scalar_type> &>::type
// ...for translation + translation
operator+(const E1 &v1, const E2 &v2) {
  return *new Translation_expr_add<typename is_acceptable_rhs_translation<E1>::scalar_type>(
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
};

template <template <typename> class T, typename Scalar>
struct is_acceptable_rhs_matrix<T<Scalar>> {
  static const bool value = std::is_base_of<Matrix<Scalar>, T<Scalar>>::value ||
                            std::is_base_of<Matrix_expr<Scalar>, T<Scalar>>::value;

  using scalar_type = Scalar;

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
  using scalar_type = Scalar;
  static const Matrix_expr<Scalar> &cast(const Translation<Scalar> &v) {
    return *new Translation_expr_leaf<Scalar>(v);
  }
};

template <typename E1, typename E2>
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E2>::value,
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator*(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_mul<typename is_acceptable_rhs_matrix<E1>::scalar_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

template <typename E1>
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E1>::value,
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::scalar_type> &>::type
// ...for - matrix
operator-(const E1 &v1) {
  return *new Matrix_expr_unary_minus<typename is_acceptable_rhs_matrix<E1>::scalar_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1));
}

template <typename E1, typename E2>
// sets return type to Matrix_expr<Scalar>...
typename std::enable_if<
    is_acceptable_rhs_matrix<E2>::value,
    const Matrix_expr<typename is_acceptable_rhs_matrix<E1>::scalar_type> &>::type
// ...for matrix * matrix
operator+(const E1 &v1, const E2 &v2) {
  return *new Matrix_expr_add<typename is_acceptable_rhs_matrix<E1>::scalar_type>(
      is_acceptable_rhs_matrix<E1>::cast(v1), is_acceptable_rhs_matrix<E2>::cast(v2));
}

} // namespace ctup

#endif
