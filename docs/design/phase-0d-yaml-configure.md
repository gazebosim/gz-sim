# Phase 0d — YAML Configure ABI

Status: Draft
Owner: Nathan Koenig
Scope: Phase 0d of the Adjustable-Fidelity initiative
Depends on: [0a](phase-0a-archetype-ecm.md), [0b](phase-0b-ecm-integration.md); can land in parallel with [0c](phase-0c-system-port.md)
Blocks: 0e (flip default), Phase 1 (scenarios consume YAML params)

## 1. Context

Phase 1 introduces scenarios whose `overrides.plugins[].params` are YAML
maps, not SDF XML. Today every plugin's `Configure` hook takes an
`sdf::ElementPtr`. We decided (design discussion, hybrid C) that plugins
**default** to a synthetic-SDF conversion path and **opt in** to native
YAML parsing via an export symbol.

0d lands that machinery: the converter, the opt-in symbol, documentation,
and a worked example port of one internal plugin.

## 2. Goals / Non-goals

Goals:
- `YamlToSdf` converter: `YAML::Node → sdf::ElementPtr` with well-defined
  rules for scalars, maps, sequences, and attributes.
- `SdfToYaml` inverse (used by `scenario convert` in Phase 1).
- A plugin-side API: optional `ConfigureFromYaml(const YAML::Node&, ...)`
  method, wired via a `GZ_ADD_YAML_CONFIGURE(PluginClass)` macro.
- Every in-tree plugin keeps working unchanged (default synthetic-SDF path).
- One internal plugin (`gz-sim-diff-drive-system`) ported as the reference
  example.
- Unit tests covering the conversion rules, including round-trip
  (`yaml → sdf → yaml`).

Non-goals:
- Deprecating or removing the existing `Configure(sdf::ElementPtr)` path.
- Converting every plugin. One reference port only; others migrate
  opportunistically.
- Schema validation of plugin params (each plugin owns its own).

## 3. API surface

### 3.1 Existing `System` interface (unchanged)

```cpp
class ISystemConfigure {
  virtual void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) = 0;
};
```

This stays. Every existing plugin continues to work.

### 3.2 New `ISystemConfigureFromYaml` interface

```cpp
// gz-sim/include/gz/sim/System.hh (additions)
namespace gz::sim {
  class ISystemConfigureFromYaml {
    public: virtual ~ISystemConfigureFromYaml() = default;
    public: virtual void ConfigureFromYaml(
        const Entity &_entity,
        const YAML::Node &_params,
        EntityComponentManager &_ecm,
        EventManager &_eventMgr) = 0;
  };
}
```

Optional: a plugin that wants native YAML inherits from both
`ISystemConfigure` (for SDF callers) and `ISystemConfigureFromYaml` (for
scenario callers).

### 3.3 Opt-in macro

```cpp
// Inside the plugin's source file, alongside GZ_ADD_PLUGIN:
GZ_ADD_PLUGIN(MyPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigureFromYaml)   // <— added
```

No new macro needed — plugin listing already supports multiple interfaces.
"Opt-in" is just declaring the additional interface and implementing it.

### 3.4 SystemLoader dispatch

`SystemLoader`, when instantiating a plugin for a scenario-sourced plugin
spec, checks:
```
if plugin implements ISystemConfigureFromYaml:
    ConfigureFromYaml(entity, yaml_params, ecm, eventMgr)
else:
    sdf_elem = YamlToSdf(yaml_params, plugin_name)
    Configure(entity, sdf_elem, ecm, eventMgr)
```

Both paths receive the same logical information. For SDF-originated plugin
specs, the original SDF element is used directly (no conversion round-trip
for the common legacy path).

## 4. `YamlToSdf` conversion rules

The goal: users writing YAML don't need to know the XML shape their plugin
expects; the rules map a reasonable YAML structure to a reasonable SDF
element.

### 4.1 Scalars

YAML scalar at a key → child element with that key as name and scalar as
text:
```yaml
wheel_radius: 0.1
```
→
```xml
<wheel_radius>0.1</wheel_radius>
```

YAML booleans, integers, floats are stringified in SDF-compatible form
(`true`/`false`, no trailing zeros on ints).

### 4.2 Maps

YAML map at a key → nested SDF element:
```yaml
pid:
  p: 10.0
  i: 0.1
  d: 1.0
```
→
```xml
<pid>
  <p>10.0</p>
  <i>0.1</i>
  <d>1.0</d>
</pid>
```

### 4.3 Sequences

YAML sequence of maps → repeated SDF elements with the parent key:
```yaml
joints:
  - name: left_wheel
    radius: 0.1
  - name: right_wheel
    radius: 0.1
```
→
```xml
<joints>
  <joint><name>left_wheel</name><radius>0.1</radius></joint>
  <joint><name>right_wheel</name><radius>0.1</radius></joint>
</joints>
```

Note the singularization rule: `joints` → repeated `<joint>`. This is
implicit and only fires when the YAML key ends in `s` *and* the value is a
list of maps. For ambiguous cases, users explicitly use the long form:

```yaml
joints:
  _element: joint                   # explicit override
  _list:
    - { name: left_wheel, radius: 0.1 }
```

YAML sequence of scalars → space-separated SDF text:
```yaml
pose: [1.0, 2.0, 0, 0, 0, 1.57]
```
→
```xml
<pose>1.0 2.0 0 0 0 1.57</pose>
```

This matches SDF convention for poses, vectors, and quaternions.

### 4.4 Attributes

XML distinguishes attributes (`<joint name="x">`) from children
(`<joint><name>x</name></joint>`). SDF in practice uses both. Two
approaches:

**A (chosen)** — special key prefix `@`:
```yaml
joint:
  "@name": left_wheel       # YAML attribute
  "@type": revolute
  radius: 0.1               # YAML child
```
→
```xml
<joint name="left_wheel" type="revolute">
  <radius>0.1</radius>
</joint>
```

**B (rejected)** — attempt to infer based on SDF schema lookup. Too much
complexity; requires SDF schema awareness inside the converter.

❓ **Q24.** Confirm the `@`-prefix approach for attributes. Alternative:
a dedicated `_attrs:` map keyword. I prefer `@` (familiar from JSON tools,
short, stays inline).

### 4.5 Disambiguation reserved keys

Keys starting with `_` are reserved for converter directives:

- `_element: <name>` — rename wrapper element (overrides YAML key).
- `_list: [...]` — explicit sequence when singularization rule is wrong.
- `_text: "..."` — element text when siblings are also present.
- `_comment: "..."` — discarded; useful for readability.

### 4.6 Round-trip guarantees

`YamlToSdf` followed by `SdfToYaml` recovers the input **up to** the
following lossy transforms:
- Whitespace in XML attribute/text is normalized.
- YAML comments (`#`) are lost.
- Unquoted numeric scalars may round-trip as quoted strings (e.g.,
  `1.0` → `"1.0"`) because SDF text is typeless.

`SdfToYaml` followed by `YamlToSdf` recovers the input byte-for-byte
**when** the SDF only uses constructs the converter understands. Unknown
SDF (custom elements, multi-namespace XML) preserved via `_raw_xml: "..."`
escape hatch. Users who hit this path are prompted to consider the
`ISystemConfigureFromYaml` opt-in.

## 5. Reference port: `gz-sim-diff-drive-system`

Chosen because it's representative (has nested params, lists, a pose).

**Before** — SDF only:
```cpp
void DiffDrive::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &) {
  if (_sdf->HasElement("wheel_radius"))
    this->wheelRadius_ = _sdf->Get<double>("wheel_radius");
  // ... more SDF traversal ...
}
```

**After** — SDF path unchanged; add YAML path:
```cpp
void DiffDrive::ConfigureFromYaml(
    const Entity &_entity,
    const YAML::Node &_params,
    EntityComponentManager &_ecm,
    EventManager &) {
  this->wheelRadius_ = _params["wheel_radius"].as<double>(0.1);
  // ... direct YAML traversal ...
}
```

Both paths build the same plugin state. A shared `ApplyConfig` helper can
dedupe if a plugin prefers:
```cpp
void DiffDrive::Configure(... sdf_elem) { ApplyConfig(SdfToYaml(sdf_elem)); }
void DiffDrive::ConfigureFromYaml(... yaml) { ApplyConfig(yaml); }
```
— but that's optional, not prescribed.

## 6. Plugin-side migration guidance

For plugins considering the native YAML path:
- Heavy parameter schemas → native YAML is cleaner (direct `node[key].as<T>()`
  beats SDF `Get<T>("key")` incantations).
- Nested / repeated structures → YAML wins on readability.
- Parameter schemas likely to change → stay on SDF; it's harder to
  accidentally break callers.

Recommendation in `tutorials/authoring_plugins.md`: default to SDF unless
your plugin is scenario-first.

## 7. Testing

### 7.1 Unit tests for converters

`YamlToSdf_TEST.cc`:
- Scalars of each YAML type → correct SDF text.
- Nested map.
- Sequence of maps with singularization and with `_list` override.
- Scalar sequence as space-separated text (pose, vector).
- Attribute via `@key`.
- `_raw_xml` escape hatch.

`SdfToYaml_TEST.cc`: symmetric coverage.

`RoundTrip_TEST.cc`: corpus of real SDF `<plugin>` tags from in-tree
examples, round-trip through `SdfToYaml → YamlToSdf`, compare SDF AST.

### 7.2 Integration test

One scenario that loads the ported `diff_drive` via YAML, one that loads
it via SDF, both drive a model in a tiny world; compare odom output
trajectories. Identical to tolerance.

## 8. Risks

**R1 — Ambiguity in YAML→SDF inference.** The singularization rule is
convenient but surprising. Mitigation: resolved report (Phase 1) shows
the synthesized SDF for any YAML-configured plugin, so users can see what
we actually handed to their plugin.

**R2 — `_raw_xml` escape hatch abuse.** Users drop into raw XML to avoid
writing an explicit mapping; effectively forks the plugin's param surface.
Acceptable — it's a clear pressure valve, not the norm.

**R3 — yaml-cpp 0.7 API churn.** yaml-cpp historically has semantics
surprises with `operator=`. Contain usage to the converter; callers
receive fully-baked strings.

❓ **Q25.** When a plugin implements `ISystemConfigureFromYaml` but a
legacy SDF caller loads it, do we (a) route via `YamlToSdf → Configure`
(requires a round-trip even for native callers), (b) require the plugin
to also implement `ISystemConfigure`, or (c) both — plugin implements
whichever interfaces it wants, loader picks the one that matches the
caller? My default: (c) — plugin decides, loader dispatches. Means a
YAML-native plugin is inaccessible to SDF-only consumers until the
plugin also adds SDF support, which is fine.

## 9. Milestones

| Week | Deliverable |
|---|---|
| 1 | `YamlToSdf` + `SdfToYaml` landed with unit tests. |
| 2 | `ISystemConfigureFromYaml` interface; `SystemLoader` dispatch. |
| 3 | `diff_drive` reference port; integration test; docs. |

Three weeks. Runs in parallel with 0c. Small and contained.

## 10. Exit criteria

- Converter + tests landed.
- Interface added to `gz/sim/System.hh`.
- SystemLoader dispatch logic landed.
- Reference port landed.
- Tutorial entry drafted.
