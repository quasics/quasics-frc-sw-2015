// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

/**
 * Implements a data-sharing mechanism, allowing the different subsystems to
 * "post" status information to a centralized place.
 *
 * This is intended to help prevent folks from inadvertently/inappropriately
 * calling methods directly on the subsystems that could have interactions with
 * the underlying hardware, when in a command that hasn't specified a dependency
 * *on* that subsystem object.
 *
 * Another, potentially simpler, approach would be to ensure that the subsystems
 * consistently provide "const" methods to clarify when a given method is safe
 * to invoke in such cases.  However, the WPILib hasn't historically been
 * const-correct, and it's also common for inexperienced developers (e.g., FIRST
 * team members) to break const-correctness; providing an indirect mechanism for
 * data sharing that doesn't depend on this is therefore potentially helpful.
 *
 * Yet another option would be to use a "publish/subscribe" mechanism, or some
 * other event-based approach.  But I'll leave that for some other time....
 *
 * @see https://isocpp.org/wiki/faq/const-correctness
 * @see https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern
 *
 * TODO: Add unit tests
 * (https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html)
 */
class BulletinBoard {
 public:
  /**
   * Defines the set of data types that can be "posted" to the bulletin board.
   *
   * Note that this isn't needed in the Java version of the code, since Java has
   * a single-root inheritance heirarchy with reference semantics, and thus
   * "Object" can be used to represent *anything*.  C++ uses multi-root, and
   * defaults to value semantics, so using a std::variant object is the simplest
   * approach to take.
   *
   * @see https://en.cppreference.com/w/cpp/utility/variant
   */
  typedef std::variant<std::string, frc::Pose2d> Value;

 public:
  static void updateValueForKey(std::string_view key, const Value& value) {
    dataSet.emplace(key, value);
  }

  static void clearValue(const std::string& key) {
    dataSet.erase(key);
  }

  static std::optional<Value> getValue(const std::string& key) {
    const auto iter = dataSet.find(key);
    if (iter == dataSet.end()) {
      return std::nullopt;
    }

    return std::optional<Value>(iter->second);
  }

  template <typename T>
  static std::optional<T> getValue(const std::string& key) {
    const auto iter = dataSet.find(key);
    if (iter == dataSet.end()) {
      return std::nullopt;
    }
    if (!std::holds_alternative<T>(iter->second)) {
      return std::nullopt;
    }

    return std::optional<T>(std::get<T>(iter->second));
  }

  template <typename T>
  static std::optional<T> getValue(const std::string_view& key) {
    return getValue<T>(std::string(key));
  }

  template <typename T>
  static void updateValue(std::string_view key, const T& value);

 private:
  static std::unordered_map<std::string, Value> dataSet;

 private:
  BulletinBoard() = delete;
};

template <typename T>
inline void BulletinBoard::updateValue(std::string_view key, const T& value) {
  updateValueForKey(key, Value(value));
}

template <>
inline void BulletinBoard::updateValue(std::string_view key,
                                       const std::string_view& value) {
  updateValueForKey(key, Value(std::string(value)));
}
