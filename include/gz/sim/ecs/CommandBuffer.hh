/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_COMMANDBUFFER_HH_
#define GZ_SIM_ECS_COMMANDBUFFER_HH_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"
#include "gz/sim/ecs/Entity.hh"

namespace gz::sim::ecs
{
  enum class CommandKind : uint8_t
  {
    CreateEntity,
    DestroyEntity,
    AddComponent,
    RemoveComponent,
    SetComponent
  };

  struct Command
  {
    CommandKind     kind;
    Entity          entity;
    ComponentTypeId type{kInvalidComponentTypeId};
    uint32_t        payload_offset{0};
    uint32_t        payload_size{0};
    // For CreateEntity commands: a caller-allocated slot where the freshly
    // minted entity handle is written at commit time. nullptr means the
    // caller doesn't need the result.
    Entity         *out_entity{nullptr};
    // For CreateEntity: how many (type, offset, size) triples follow this
    // command in the parallel create_components_ vector.
    uint32_t        num_create_comps{0};
  };

  /// \brief Per-phase queue of deferred mutations. Drained by World::Commit.
  class CommandBuffer
  {
    public: CommandBuffer() = default;
    public: CommandBuffer(const CommandBuffer &) = delete;
    public: CommandBuffer &operator=(const CommandBuffer &) = delete;
    public: CommandBuffer(CommandBuffer &&) = default;
    public: CommandBuffer &operator=(CommandBuffer &&) = default;

    public: struct CreateComp
    {
      ComponentTypeId type;
      uint32_t        payload_offset;
      uint32_t        payload_size;
    };

    /// \brief Append raw bytes for a component payload. Returns the offset
    /// into the blob arena.
    public: uint32_t AppendPayload(const void *_src, size_t _n);

    public: const std::vector<Command> &Ops() const { return this->ops_; }
    public: std::vector<Command> &Ops() { return this->ops_; }

    public: const std::vector<std::byte> &Blob() const { return this->blob_; }
    public: std::vector<std::byte> &Blob() { return this->blob_; }

    public: const std::vector<CreateComp> &CreateComps() const
    { return this->create_components_; }
    public: std::vector<CreateComp> &CreateComps()
    { return this->create_components_; }

    /// \brief Reset ops and blob contents, keep capacity.
    public: void Reset()
    {
      this->ops_.clear();
      this->blob_.clear();
      this->create_components_.clear();
    }

    public: bool Empty() const
    {
      return this->ops_.empty();
    }

    private: std::vector<Command>      ops_;
    private: std::vector<std::byte>    blob_;
    private: std::vector<CreateComp>   create_components_;
  };
}

#endif
