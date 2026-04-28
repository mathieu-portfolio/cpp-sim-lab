#pragma once

#include <tuple>

namespace simfw::ui {

// ---------- Field descriptors ----------

template <typename Stats, typename T>
struct StatField {
    const char* name;
    T Stats::*member;
};

template <typename Config, typename T>
struct TunableField {
    const char* name;
    T Config::*member;
    float minValue;
    float normalStep;
    float fastStep;
};

// ---------- Traits (to specialize) ----------

template <typename T>
struct StatsUiTraits;

template <typename T>
struct ConfigUiTraits;

} // namespace simfw::ui
