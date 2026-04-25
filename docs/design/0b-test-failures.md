# Phase 0b — Test failures under `GZ_SIM_ARCHETYPE_ECM=ON`

Status: working document. Captures every failing test in the
`GZ_SIM_ARCHETYPE_ECM=ON` build at the point the journal moved to
this phase, the analysis of why it fails, the proposed fix, and
whether the fix is in non-test code or requires a test edit.

Working assumption (per direction): **the test is correct, the new
code is wrong.** Test edits are called out explicitly with
justification.

A summary table appears at the end (§30).

---

## §1. Two dominant root-cause patterns

Reading 28 failures, two patterns explain ~90 % of them.

### Pattern A — `CreateComponent<ParentEntity>` does not update the entity graph

The legacy `EntityComponentManager::CreateComponentImplementation`
([src/EntityComponentManager.cc:1203](../../src/EntityComponentManager.cc#L1203))
includes a special case:

```cpp
if (_componentTypeId == components::ParentEntity::typeId)
{
  auto parentComp = this->Component<components::ParentEntity>(_entity);
  this->SetParentEntity(_entity, parentComp->Data());
}
```

Adding a `ParentEntity` component automatically wires a parent-child
edge into the `EntityGraph`. This is what `ChildrenByComponents` /
`AdjacentsFrom` traversals rely on.

The archetype facade's `CreateComponentImplementation` does **not**
have this special case. So tests that build a parent-child topology
by attaching `ParentEntity` components see entities exist but no
graph edges. `ChildrenByComponents`, `Joint::Sensors`, `Link::Sensors`,
`Model::Links`, `World::ModelByName`, etc. all return empty.

**Fix** (non-test): mirror the legacy special case in
`EntityComponentManagerArchetype.cc`'s
`CreateComponentImplementation` — when typeId equals
`components::ParentEntity::typeId`, call `SetParentEntity` after
storing.

### Pattern B — `RemoveComponent` destroys immediately

Legacy `RemoveComponent` puts the component into
`componentsMarkedAsRemoved`; the actual destructor runs at
`ClearRemovedComponents` (end of step). Pointers obtained before
the remove stay valid for the rest of the step.

Archetype facade's `RemoveComponent` calls `eit->second.erase(typeId)`
on the shadow `unique_ptr<BaseComponent>` map → destructor fires
immediately. Pointers obtained before `RemoveComponent` are dangling.

**Fix** (non-test): mirror legacy semantics — move the cloned
`unique_ptr` from `shadowComponents` into a "pending destruction"
list. Keep the BaseComponent alive but mark it removed so it stops
appearing in `Component<T>` reads. Drain the list in
`ClearRemovedComponents`.

### Pattern C — `RequestRemoveEntity(_recursive=true)` ignores the recursion flag

Legacy: `RequestRemoveEntity(entity, true)` walks the entity graph
descendants and queues them all for removal.

Archetype facade today: `(void)_recursive;` — only the named entity
gets queued. Children stay.

**Fix** (non-test): walk the entity graph (or `Descendants`) and
queue all reachable entities.

---

## §2. UNIT_EntityComponentManager_TEST (SEGFAULT)

**Failure:** test crashes with SIGSEGV at line 2264.

```cpp
auto c1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
…
manager.RemoveComponent<IntComponent>(e1);
manager.UpdatePeriodicChangeCache(changeTracker);
EXPECT_EQ(changeTracker.size(), 1u);
EXPECT_EQ(changeTracker[c1->TypeId()].size(), 0u);   // <- segfault
```

**Cause:** Pattern B. `c1` points into the shadow store; the
`unique_ptr` is destroyed by `RemoveComponent`, dangling pointer
→ `c1->TypeId()` deref-after-free.

**Fix:** non-test, Pattern B. Defer destruction until
`ClearRemovedComponents`.

---

## §3. UNIT_AddedMass_TEST (Failed)

**Failure:** physics integration values differ from expected by
multiple metres / radians per second² (`actual: 2.87… vs 0.01`).

**Cause:** test runs the hydrodynamics + added-mass plugin and
compares simulated free-body acceleration against a
closed-form analytic answer. Plugin uses `Each<Pose, LinearVelocity,
…>`. Under the archetype facade, components created via
SdfEntityCreator land in the shadow store; `Each<Pose, …>` walks the
shadow store via `AllEntitiesArchetypeFacade` so entities are seen
— but my unified iteration calls `Component<T>(e)` for every entity
× every requested type, returning either shadow store data or
archetype World data (whichever is present). For the typical case
where SdfEntityCreator stored everything in shadow, this works.

The arithmetic mismatch is large (orders of magnitude off), which
points at something deeper than missed iteration: probably the
plugin reads a pose and writes it back, but our shadow-store path
returns a **clone** rather than a live pointer into the storage,
so writes don't persist. Need to verify by looking at the
hydrodynamics plugin's read/write pattern.

Actually — looking at my `ComponentImplementation` impl: shadow
store returns `eit->second.find(_type).get()` (raw pointer to the
stored `unique_ptr`'s pointee). Writes through that pointer DO
persist. So that's not the issue.

More likely: the plugin reads a `WorldLinearVelocity` (which it
expects an upstream physics system to compute) and the upstream
system never wrote it because `Each<Pose, Inertial, …>` missed
some entity due to the **ParentEntity-graph-not-updated** issue
(Pattern A). The hydro plugin operates per-link, finds links via
`Each<Link>`, but per-link hydrodynamic torque computation uses
parent-model relationships that depend on the graph.

**Fix:** non-test, Pattern A. Then re-run; this category often
clears with the Pattern A fix.

---

## §4. UNIT_Joint_TEST.Sensors (Failed)

**Failure:** [src/Joint_TEST.cc:129](../../src/Joint_TEST.cc#L129)

```cpp
EXPECT_EQ(2u, sensors.size());   // got 0
```

**Cause:** Pattern A. Test creates entities with
`CreateComponent<ParentEntity>` linkage; `Joint::Sensors` calls
`ecm.ChildrenByComponents` which uses `AdjacentsFrom` on the entity
graph; graph is empty.

**Fix:** non-test, Pattern A.

---

## §5. UNIT_Link_TEST.Sensors (Failed)

**Failure:** same shape as §4 at line 131.

**Cause:** Pattern A.

**Fix:** non-test, Pattern A.

---

## §6. UNIT_Model_TEST.Links / .Models (Failed)

**Failure:** `Model::Links()` / `Model::Models()` return empty
sets when the test expects 2 / 1.

**Cause:** Pattern A. `Model::Links()` walks `ChildrenByComponents`.

**Fix:** non-test, Pattern A.

---

## §7. UNIT_SdfEntityCreator_TEST (Failed)

**Failure:** rapid-fire `Expected: true` assertions starting at
line 95 (CreateEntities of model + child link + sensor).

**Cause:** Pattern A. SdfEntityCreator builds parent-child via
`SetParentEntity`. Wait — *if* it uses SetParentEntity, the graph
should populate. Let me re-check: the ECM's `SetParentEntity` does
work in my facade. But `SdfEntityCreator` may also rely on
`CreateComponent<ParentEntity>` to update the graph in some
shorter paths. Let me grep to confirm.

After grep: SdfEntityCreator uses both `SetParentEntity` (graph)
and `CreateComponent<ParentEntity>` (the component). Under legacy
those do the same thing because `CreateComponent<ParentEntity>` is
self-syncing. Under archetype the component is set but the graph
isn't. Whichever path the test exercises, Pattern A is the cause.

**Fix:** non-test, Pattern A.

---

## §8. UNIT_SimulationRunner_TEST (Failed)

**Failure:** dense `Expected: true` failures starting at line 128
(EntityCount/HasEntity/etc. after server-load).

**Cause:** Pattern A. Test loads a world, queries the resulting
ECM by entity-tree shape (`Model::Links`, `Link::Sensors`, etc.).

**Fix:** non-test, Pattern A.

---

## §9. UNIT_JointPositionController_TEST.PublishCommand (Failed)

**Failure:** GUI plugin test that runs an in-process server,
publishes a command, expects to read back state.

**Cause:** mixed — likely cascade of Pattern A (entity graph) plus
the change-tracking emission from the SetState path on the GUI
side. Needs a deeper look but the diagnosis tools (gz topic echo
under archetype build) point at server-side state filtering.

**Fix:** non-test, Pattern A first; may also need a re-look at
SetState semantics on the GUI's secondary ECM.

---

## §10. INTEGRATION_battery_plugin (Failed)

**Failure:** dense `Expected: true` at line 99 — the test publishes
a battery state and expects the plugin to react. Plugin discovers
the battery via `Each<BatterySoC>`.

**Cause:** Pattern A (battery is on a link entity; link parent
relationship via `ParentEntity` component; without graph edge the
plugin can't find the model).

**Fix:** non-test, Pattern A.

---

## §11. INTEGRATION_contact_system (Failed)

**Failure:** `contactMsgs.size() < 1` (expected ≥ 1).

**Cause:** physics doesn't generate contacts because collisions
aren't registered. `Physics.cc`'s `EachNew<Collision, Name, Pose,
ParentEntity, …>` callback fires (since EachNew now works), but
the parent (link) → collision relationship requires an entity-graph
edge. Without it, Physics's internal ID map can't tie collision
back to a parent — see also §12. Pattern A.

**Fix:** non-test, Pattern A.

---

## §12. INTEGRATION_entity_erase (Failed)

**Failure:** after `server.RequestRemoveEntity("vehicle_blue")`,
chassis/wheels/joints still exist (test expects cascading deletion).

**Cause:** Pattern C — recursive flag ignored. Plus Pattern A
because `Descendants` uses the entity graph which isn't populated
properly to start.

**Fix:** non-test, Patterns A and C.

---

## §13. INTEGRATION_joint (Failed — `SensorByName`, `ParentModel`)

**Cause:** Pattern A. Tests query parent/child via the entity
graph.

**Fix:** non-test, Pattern A.

---

## §14. INTEGRATION_light.Parent (Failed)

**Cause:** Pattern A. `Light::Parent` returns `kNullEntity`.

**Fix:** non-test, Pattern A.

---

## §15. INTEGRATION_link (`VisualByName`, `SensorByName`, `CollisionByName`) (Failed)

**Cause:** Pattern A.

**Fix:** non-test, Pattern A.

---

## §16. INTEGRATION_load_system_static_registry.LoadWorks (Failed)

**Failure:** `Expected: true` at line 81 — checks the loaded plugin
ran a Configure callback that writes a component the test then
reads back.

**Cause:** likely cascade. The test writes a model component, but
the plugin's Configure looks up the world entity by name (via
`EntityByName`) — which now works (we implemented it) but is O(N)
linear scan. If everything else is right, this should pass; if it
still fails after Pattern A is fixed, dig deeper.

**Fix:** non-test, likely Pattern A. Re-test after.

---

## §17. INTEGRATION_model (`ModelByName`, `LinkByName`, `JointByName`, …) (Failed)

**Cause:** Pattern A.

**Fix:** non-test, Pattern A.

---

## §18. INTEGRATION_model_photo_shoot_default_joints (Failed)

**Failure:** `Expected: false` at ModelPhotoShootTest.hh:343 —
test expects a generated PNG to differ from a reference; got
identical (i.e. the simulation didn't render anything new).

**Cause:** likely cascade: model loading fails because of Pattern A
→ no entities visible to scene_broadcaster → empty render → match
"empty" reference.

**Fix:** non-test, Pattern A. Also check model_photo_shoot system
specifically — it uses `EachNew` to discover models.

---

## §19. INTEGRATION_model_photo_shoot_random_joints (Failed)

Same as §18.

**Fix:** non-test, Pattern A.

---

## §20. INTEGRATION_model_photo_shoot_reset (Failed)

Same family. Adds a `Reset` step that calls `ResetTo` — which is
still a `STUB(0b)` in my facade.

**Cause:** primarily Pattern A. Secondary: `ResetTo` stub.

**Fix:** non-test. Pattern A unblocks the bulk; `ResetTo`
implementation handles the Reset-specific portion. ResetTo is
documented as remaining stub work; once Pattern A is in, decide
whether this test needs a follow-up commit for ResetTo.

---

## §21. INTEGRATION_physics_system (Failed)

**Failure:**
```
[error] [Physics.cc:3188] Internal error: a physics entity ptr
  with an ID of [5] does not exist.
```

**Cause:** Pattern A. `Physics.cc`'s `UpdatePhysics` looks up the
parent of a link entity via `Component<ParentEntity>` to find which
model the link belongs to. Without the entity-graph special-case,
the physics map is keyed by entity but the parent-child topology
is broken — Physics tries to update a child link by ID but its
internal map keys on the model entity → "ID does not exist".

**Fix:** non-test, Pattern A.

---

## §22. INTEGRATION_recreate_entities (Failed)

**Failure:** `Expected: (kNullEntity) != (boxModelEntity)` —
the test recreates an entity by name after destroying the old one.

**Cause:** depends on `EntityByName` (works now) plus Pattern A
(graph relationships in the recreated entity tree).

**Fix:** non-test, Pattern A.

---

## §23. INTEGRATION_reset.HandleReset (Failed)

**Cause:** uses `Reset` which calls `ResetTo`. Pattern A gets us
partway; the rest needs `ResetTo` implementation. Same as §20.

**Fix:** non-test, Pattern A + eventual `ResetTo`.

---

## §24. INTEGRATION_sensor.Parent (Failed)

**Cause:** Pattern A.

**Fix:** non-test, Pattern A.

---

## §25. INTEGRATION_touch_plugin.OneLink (Failed)

**Failure:** plugin doesn't fire the touch signal because contact
detection isn't seeing the collision pair.

**Cause:** same as §11 — collisions don't register because
Pattern A breaks the link-to-collision graph.

**Fix:** non-test, Pattern A.

---

## §26. INTEGRATION_user_commands (Failed)

**Failure:** dense run of `Expected: true` at lines 474–556,
covering pose-set, model-spawn, and link-add commands.

**Cause:** mixed. The user_commands system uses `EntityByName`
(works) and writes via `CreateComponent<ParentEntity>` for newly
spawned entities — Pattern A bites the spawn path.

**Fix:** non-test, Pattern A.

---

## §27. INTEGRATION_log_system (Failed)

**Failure:** at line 799,
`Expected: (nullptr) != (nameComp), actual: (nullptr) vs NULL` —
playback couldn't find a Name component on a re-created entity.

**Cause:** Log playback uses `SetState(SerializedStateMap)` to apply
recorded state. My SetState implements component creation through
the Factory + CreateComponentImplementation. If the original state
contained a `ParentEntity` component, the SetState path creates
the component but doesn't update the entity graph (Pattern A).
Subsequent `Component<Name>` lookups walk the graph and fail.

**Fix:** non-test, Pattern A. The fix in CreateComponentImplementation
covers SetState's call into it.

---

## §28. INTEGRATION_world (Failed)

**Failure:** `WorldIntegrationTest.ModelByName` etc. at line 190+.

**Cause:** Pattern A. `World::ModelByName` walks model children via
the entity graph.

**Fix:** non-test, Pattern A.

---

## §29. PERFORMANCE_each (Failed)

**Failure:**
```
Expected: (cacheEntityAvg) < (cachelessEntityAvg),
  actual: 31.65 vs 29.3
```

The test assumes the cached `Each` path is faster than `EachNoCache`.
Under the archetype facade I aliased both to the same unified
walk, so they execute identical code → identical timing. The
"cached" version is no longer measurably faster.

**Cause:** intentional facade unification. The legacy view-cache
mechanism is a no-op under the archetype design (queries
self-invalidate; there's no cache to be hit-rate'd). The test's
assumption no longer holds.

**Fix:** **test edit.** This is the design change: the archetype
has no view caching, so the relative-timing assertion can't
pass. Change the test to:
- Skip the assertion when `GZ_SIM_USE_ARCHETYPE_ECM` is defined,
  with a note pointing at the design doc, OR
- Drop the assertion in favor of an absolute-throughput floor
  (e.g., `cacheEntityAvg < 100 ns`). The performance is still
  better than legacy in absolute terms.

This is the only failure where the only correct path is a test
edit — every other listed test is correct as written and the new
code needs to honor its preconditions.

---

## §30. Summary table

| #  | Test | Fix in non-test code? | Fix breaks ECS design? | Pattern |
|---:|---|:---:|:---:|---|
|  2 | UNIT_EntityComponentManager_TEST | Yes | No | B (deferred destroy) |
|  3 | UNIT_AddedMass_TEST | Yes | No | A (cascade) |
|  4 | UNIT_Joint_TEST | Yes | No | A |
|  5 | UNIT_Link_TEST | Yes | No | A |
|  6 | UNIT_Model_TEST | Yes | No | A |
|  7 | UNIT_SdfEntityCreator_TEST | Yes | No | A |
|  8 | UNIT_SimulationRunner_TEST | Yes | No | A |
|  9 | UNIT_JointPositionController_TEST | Yes | No | A + GUI cascade |
| 10 | INTEGRATION_battery_plugin | Yes | No | A |
| 11 | INTEGRATION_contact_system | Yes | No | A |
| 12 | INTEGRATION_entity_erase | Yes | No | A + C (recursion) |
| 13 | INTEGRATION_joint | Yes | No | A |
| 14 | INTEGRATION_light | Yes | No | A |
| 15 | INTEGRATION_link | Yes | No | A |
| 16 | INTEGRATION_load_system_static_registry | Yes | No | A (cascade) |
| 17 | INTEGRATION_model | Yes | No | A |
| 18 | INTEGRATION_model_photo_shoot_default_joints | Yes | No | A (cascade) |
| 19 | INTEGRATION_model_photo_shoot_random_joints | Yes | No | A (cascade) |
| 20 | INTEGRATION_model_photo_shoot_reset | Yes | No | A + ResetTo |
| 21 | INTEGRATION_physics_system | Yes | No | A |
| 22 | INTEGRATION_recreate_entities | Yes | No | A |
| 23 | INTEGRATION_reset | Yes | No | A + ResetTo |
| 24 | INTEGRATION_sensor | Yes | No | A |
| 25 | INTEGRATION_touch_plugin | Yes | No | A (cascade via contact) |
| 26 | INTEGRATION_user_commands | Yes | No | A |
| 27 | INTEGRATION_log_system | Yes | No | A |
| 28 | INTEGRATION_world | Yes | No | A |
| 29 | PERFORMANCE_each | **No — test edit** | N/A | view-cache premise |

**Aggregate:**
- 27 of 28 are non-test code fixes.
- 26 of those resolve via Pattern A (`CreateComponent<ParentEntity>`
  must update the entity graph). One additional fix (Pattern B,
  deferred component destruction) handles the one SEGFAULT case.
  One additional fix (Pattern C, `RequestRemoveEntity` recursion)
  handles `entity_erase` fully.
- 1 of 28 (PERFORMANCE_each) requires a test edit because the test
  asserts a property of the legacy view cache that doesn't exist
  under the archetype design. None of these fixes break the
  archetype design — Patterns A/B/C are facade-level compatibility
  shims sitting outside the archetype core.

**Suggested fix order:**
1. Land Pattern A (one-line addition to
   `CreateComponentImplementation` mirroring legacy at L1203).
   This alone should turn ~25 tests green.
2. Land Pattern B (defer `RemoveComponent` destruction to
   `ClearRemovedComponents`).
3. Land Pattern C (recursive `RequestRemoveEntity` walks
   `Descendants`).
4. Re-run the full suite. If any cascade-suspect tests
   (model_photo_shoot, load_system_static_registry,
   recreate_entities) still fail, dig into them individually —
   they likely need targeted fixes once the Pattern A fog
   clears.
5. Edit `PERFORMANCE_each` to drop the cache-vs-cacheless
   assertion under archetype builds, with a short comment
   pointing at this document.
