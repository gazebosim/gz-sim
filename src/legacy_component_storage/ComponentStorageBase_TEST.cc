/*
 * Standalone differential check for the legacy_component_storage reference
 * reconstruction: not wired into any build file, not run by CI. Compares
 * ComponentStorageRef (vector, the historical behavior) against
 * ComponentStorageOurs (deque) for (a) identical stored values and ids, and
 * (b) the "expanded" (reallocation-triggered-rebuild) flag: never true for
 * ours, true ~E/100 times for the reference, confirming the mechanism the
 * optimization removes.
 *
 * Build and run manually, e.g.:
 *   c++ -O2 -std=c++17 ComponentStorageBase_TEST.cc -o storage_test && ./storage_test
 */

#include <cassert>
#include <cstdio>
#include "ComponentStorageBase_reference.hh"
#include "ComponentStorageBase_ours.hh"

using legacy_component_storage::ComponentStorageRef;
using legacy_component_storage::ComponentStorageOurs;

struct Pose
{
  double x, y, z;
  bool operator==(const Pose &_o) const
  {
    return x == _o.x && y == _o.y && z == _o.z;
  }
};

int main()
{
  const int kNumEntities = 8000;

  ComponentStorageRef<Pose> ref;
  ComponentStorageOurs<Pose> ours;

  int refExpandedCount = 0;
  int oursExpandedCount = 0;

  for (int i = 0; i < kNumEntities; ++i)
  {
    Pose p{static_cast<double>(i), static_cast<double>(i) * 2.0, static_cast<double>(i) * 3.0};

    auto [refId, refExpanded] = ref.Create(&p);
    auto [oursId, oursExpanded] = ours.Create(&p);

    if (refExpanded) ++refExpandedCount;
    if (oursExpanded) ++oursExpandedCount;

    assert(refId == oursId);

    const Pose *refP = ref.Component(refId);
    const Pose *oursP = ours.Component(oursId);
    assert(refP != nullptr && oursP != nullptr);
    assert(*refP == *oursP);
  }

  // Spot-check every 100th entity's value is still correct after all
  // insertions (would catch a stale-pointer/reallocation bug directly).
  for (int i = 0; i < kNumEntities; i += 100)
  {
    const Pose *refP = ref.Component(i);
    const Pose *oursP = ours.Component(i);
    assert(refP != nullptr && oursP != nullptr);
    assert(*refP == *oursP);
    assert(refP->x == static_cast<double>(i));
  }

  std::printf("entities=%d  ref expanded (full-rebuild-triggering) count=%d  "
              "ours expanded count=%d\n",
              kNumEntities, refExpandedCount, oursExpandedCount);
  assert(oursExpandedCount == 0);
  assert(refExpandedCount > 0);

  std::printf("OK: identical ids/values across %d creates; reference "
              "reallocated (and would have triggered RebuildViews) %d times, "
              "ours never did\n", kNumEntities, refExpandedCount);
  return 0;
}
