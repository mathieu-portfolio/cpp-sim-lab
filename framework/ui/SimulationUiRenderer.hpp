#pragma once

#include "SimulationUiTraits.hpp"
#include "RaylibDebugUi.hpp"

#include <tuple>

namespace simfw::ui {

// -------- tuple iteration --------
template <typename Tuple, typename Func, std::size_t... I>
void forEachImpl(Tuple&& t, Func&& f, std::index_sequence<I...>) {
    (f(std::get<I>(t)), ...);
}

template <typename Tuple, typename Func>
void forEach(Tuple&& t, Func&& f) {
    constexpr auto size = std::tuple_size_v<std::decay_t<Tuple>>;
    forEachImpl(std::forward<Tuple>(t), std::forward<Func>(f),
                std::make_index_sequence<size>{});
}

// -------- stats --------
template <typename Stats>
void drawStats(TextCursor& cursor, const Stats& stats) {
    forEach(StatsUiTraits<Stats>::fields, [&](auto field) {
        cursor.draw(
            TextFormat("%s: %d",
                field.name,
                static_cast<int>(stats.*(field.member))
            )
        );
    });
}

// -------- tunables --------
template <typename Config>
void adjustTunables(
    Config& config,
    std::size_t selected,
    float direction,
    float dt,
    bool fast
) {
    std::size_t index = 0;

    forEach(ConfigUiTraits<Config>::fields, [&](auto field) {
        if (index == selected) {
            TunableParameter p{
                field.name,
                &(config.*(field.member)),
                field.minValue,
                field.normalStep,
                field.fastStep
            };

            adjustTunable(p, direction, dt, fast);
        }
        ++index;
    });
}

template <typename Config>
void drawTunables(
    TextCursor& cursor,
    const Config& config,
    std::size_t selected
) {
    std::size_t index = 0;

    forEach(ConfigUiTraits<Config>::fields, [&](auto field) {
        const float value = config.*(field.member);

        cursor.draw(
            TextFormat("%s: %.2f%s",
                field.name,
                value,
                (index == selected ? " <" : "")
            ),
            16,
            (index == selected ? YELLOW : LIGHTGRAY)
        );

        ++index;
    });
}

} // namespace simfw::ui
