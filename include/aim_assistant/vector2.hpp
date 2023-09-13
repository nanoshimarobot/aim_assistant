/**
 * @file vector2.hpp
 * @author Takumi Odashima (Kotakkucu@gmail.com)
 * @brief
 * @date 2021-01-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <cmath>
#include <cassert>

namespace abu2023 {
namespace cpp_general {

constexpr inline double lerp(const double a, const double b, const double t) { return a + (b - a) * t; }
constexpr inline bool approx_eq(const double a, const double b) { return (std::abs(a - b) < 1e-12); }
constexpr inline bool approx_zero(const double a) { return (std::abs(a) < 1e-12); }

struct Vector2 {
  double x, y;

  Vector2() = default;

  constexpr Vector2(double vx, double vy) : x(vx), y(vy) {}

  constexpr Vector2(const Vector2 &) = default;

  /**
   * @brief 値の代入
   *
   * @param vx double
   * @param vy
   */
  constexpr void set(const double vx, const double vy) {
    x = vx;
    y = vy;
  }

  /**
   * @brief 極形式での値の代入
   *
   * @param radius 半径
   * @param theta 角度[rad]
   */
  void set_polar(const double radius, const double theta) {
    x = radius * std::cos(theta);
    y = radius * std::sin(theta);
  }

  /**
   * @brief 内積
   *
   * @param v
   * @return constexpr double
   */
  constexpr double dot(const Vector2 &v) const { return (x * v.x + y * v.y); }

  /**
   * @brief 外積
   *
   * @param v
   * @return constexpr double
   */
  constexpr double cross(const Vector2 &v) const { return (x * v.y - y * v.x); }

  /**
   * @brief ベクトルの大きさを返す
   *
   * @return constexpr double
   */
  constexpr double norm() const { return std::sqrt(norm_sq()); }

  /**
   * @brief 大きさの2乗
   *
   * @return double
   */
  constexpr double norm_sq() const { return dot(*this); }

  /**
   * @brief 単位ベクトル化
   *
   */
  void normalize() { *this /= norm(); }

  /**
   * @brief ベクトルの角度を返す
   *
   * @return double
   */
  constexpr double angle() const { return std::atan2(y, x); }

  /**
   * @brief 単位ベクトル化したベクトルを返す
   *
   * @return Vector2
   */
  constexpr Vector2 get_normalized() const {
    Vector2 v = *this;
    v /= v.norm();
    return v;
  }

  /**
   * @brief 原点中心に回転
   *
   * @param theta [rad]
   */
  void rotate(const double theta) {
    Vector2 v = *this;
    x = v.x * std::cos(theta) - v.y * std::sin(theta);
    y = v.x * std::sin(theta) + v.y * std::cos(theta);
  }

  /**
   * @brief 原点中心に回転させたベクトルを返す
   *
   * @param theta
   * @return Vector2
   */
  constexpr Vector2 get_rotated(const double theta) const {
    Vector2 v = {x * std::cos(theta) - y * std::sin(theta), x * std::sin(theta) + y * std::cos(theta)};

    return v;
  }

  constexpr Vector2 get_complex_product(const Vector2 arg_vec) const {
    return Vector2(x * arg_vec.x - y * arg_vec.y, x * arg_vec.y + y * arg_vec.x);
  }

  constexpr bool is_zero() const { return approx_zero(x) && approx_zero(y); }

  constexpr bool has_nan() const { return std::isnan(x) || std::isnan(y); }

  constexpr Vector2 yx() const { return {y, x}; }

  constexpr Vector2 nyx() const { return {-y, x}; }

  constexpr Vector2 ynx() const { return {y, -x}; }

  constexpr Vector2 nxy() const { return {-x, y}; }

  constexpr Vector2 xny() const { return {x, -y}; }

  /**
   * @brief 内積
   *
   * @param a
   * @param b
   * @return constexpr double
   */
  static constexpr double dot(const Vector2 &a, const Vector2 &b) { return a.dot(b); }
  static constexpr double cross(const Vector2 &a, const Vector2 &b) { return a.cross(b); }
  static constexpr Vector2 polar(const double radius, const double theta) {
    return Vector2(radius * cos(theta), radius * sin(theta));
  }

  /**
   * @brief 2つのベクトルのなす角
   *
   * @param a
   * @param b
   * @return double
   */
  static constexpr double angle(const Vector2 &a, const Vector2 &b) {
    double value = a.dot(b) / (a.norm() * b.norm());
    return std::acos(value);
  }

  static constexpr double distance(const Vector2 &a, const Vector2 &b) { return (b - a).norm(); }

  /**
   * @brief 内分点
   *
   * @param a
   * @param b
   * @param t
   * @return constexpr Vector2
   */
  static constexpr Vector2 lerp(const Vector2 &a, const Vector2 &b, const double t) {
    return {abu2023::cpp_general::lerp(a.x, b.x, t), abu2023::cpp_general::lerp(a.y, b.y, t)};
  }

  /**
   * @brief
   *
   * @param b
   * @param t
   * @return constexpr Vector2
   */
  constexpr Vector2 lerp(const Vector2 &b, const double t) const { return lerp(*this, b, t); }

  // constatnt vector
  static constexpr Vector2 zero() { return {0, 0}; }

  static constexpr Vector2 up() { return {0, 1}; }

  static constexpr Vector2 down() { return {0, -1}; }

  static constexpr Vector2 right() { return {1, 0}; }

  static constexpr Vector2 left() { return {-1, 0}; }

  constexpr Vector2 operator+() const { return *this; }

  constexpr Vector2 operator-() const { return {-x, -y}; }

  constexpr bool operator==(const Vector2 &v) const { return (x == v.x) && (y == v.y); }

  constexpr bool operator!=(const Vector2 &v) const { return !((x == v.x) && (y == v.y)); }

  constexpr Vector2 operator+(const Vector2 &v) const { return {x + v.x, y + v.y}; }

  constexpr Vector2 operator-(const Vector2 &v) const { return {x - v.x, y - v.y}; }

  friend constexpr Vector2 operator/(const Vector2 &v, const double value) { return {v.x / value, v.y / value}; }

  constexpr Vector2 &operator+=(const Vector2 &v) {
    x += v.x;
    y += v.y;

    return *this;
  }

  constexpr Vector2 &operator-=(const Vector2 &v) {
    x -= v.x;
    y -= v.y;

    return *this;
  }

  constexpr Vector2 &operator*=(const double value) {
    x *= value;
    y *= value;

    return *this;
  }

  constexpr Vector2 &operator/=(const double value) {
    x /= value;
    y /= value;

    return *this;
  }

  double &operator[](const size_t index) {
    assert(index < 2);
    if (index == 0)
      return x;
    return y;
  }

  constexpr double operator[](const size_t index) const {
    assert(index < 2);
    if (index == 0)
      return x;
    return y;
  }

  /**
   * @brief
   *
   * @tparam Scaler
   * @param v
   * @param s
   * @return constexpr Vector2
   */
  template <typename Scaler> friend constexpr Vector2 operator*(const Vector2 &v, const Scaler s) noexcept {
    return {v.x * static_cast<double>(s), v.y * static_cast<double>(s)};
  }

  template <typename Scaler> friend constexpr Vector2 operator*(const Scaler s, const Vector2 &v) noexcept {
    return (v * static_cast<double>(s));
  }
};
} // namespace cpp_general
} // namespace abu2023