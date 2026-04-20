/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_ENTITY_HH_
#define GZ_SIM_ECS_ENTITY_HH_

#include <cstdint>
#include <functional>

namespace gz::sim::ecs
{
  /// \brief 64-bit entity handle. Low 32 bits are a dense slot index,
  /// high 32 bits are the slot generation. Generation is bumped on destroy
  /// so stale handles can be detected on lookup.
  class Entity
  {
    public: constexpr Entity() = default;

    public: constexpr Entity(uint32_t _index, uint32_t _gen)
      : value_((static_cast<uint64_t>(_gen) << 32) |
               static_cast<uint64_t>(_index))
    {
    }

    public: static constexpr Entity FromRaw(uint64_t _raw)
    {
      Entity e; e.value_ = _raw; return e;
    }

    public: constexpr uint32_t Index() const
    {
      return static_cast<uint32_t>(this->value_ & 0xFFFFFFFFull);
    }

    public: constexpr uint32_t Generation() const
    {
      return static_cast<uint32_t>(this->value_ >> 32);
    }

    public: constexpr uint64_t Raw() const { return this->value_; }

    public: constexpr bool operator==(const Entity &_o) const
    {
      return this->value_ == _o.value_;
    }

    public: constexpr bool operator!=(const Entity &_o) const
    {
      return this->value_ != _o.value_;
    }

    public: constexpr bool operator<(const Entity &_o) const
    {
      return this->value_ < _o.value_;
    }

    private: uint64_t value_{0};
  };

  /// \brief Invalid / null entity sentinel.
  inline constexpr Entity kNullEntity = Entity();
}

namespace std
{
  template <>
  struct hash<gz::sim::ecs::Entity>
  {
    size_t operator()(const gz::sim::ecs::Entity &_e) const noexcept
    {
      return std::hash<uint64_t>{}(_e.Raw());
    }
  };
}

#endif
