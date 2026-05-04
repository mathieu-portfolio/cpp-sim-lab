#pragma once

#include <tuple>

namespace simfw::ui {

// ---------- Field descriptors ----------

template <typename Stats, typename T> struct StatField {
  const char *name;
  T Stats::*member;
};

template <typename Config, typename T> struct TunableField {
  const char *name;
  T Config::*member;
  float step;
  float minValue;
  float maxValue;
};

template <typename Stats, typename T>
constexpr StatField<Stats, T> makeStatField(const char *name,
                                            T Stats::*member) {
  return StatField<Stats, T>{name, member};
}

template <typename Config, typename T>
constexpr TunableField<Config, T>
 makeTunableField(const char *name, T Config::*member, float step,
                 float minValue, float maxValue) {
  return TunableField<Config, T>{name, member, step, minValue, maxValue};
}

// ---------- Traits (to specialize) ----------

template <typename T> struct StatsUiTraits;

template <typename T> struct ConfigUiTraits;

} // namespace simfw::ui
