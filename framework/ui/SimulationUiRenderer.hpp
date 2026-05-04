#pragma once

#include "SimulationUiTraits.hpp"
#include "UiHelpers.hpp"

#include <cmath>
#include <tuple>
#include <type_traits>

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
void drawStats(TextCursor& cursor, const Stats& stats, Color color = LIGHTGRAY) {
    forEach(StatsUiTraits<Stats>::fields, [&](auto field) {
        cursor.draw(
            TextFormat("%s: %d",
                field.name,
                static_cast<int>(stats.*(field.member))
            ),
            16,
            color
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
            using FieldType = std::remove_reference_t<decltype(config.*(field.member))>;

            if constexpr (std::is_same_v<FieldType, float>) {
                TunableParameter p{
                    field.name,
                    &(config.*(field.member)),
                    field.step,
                    field.minValue,
                    field.maxValue
                };

                adjustTunable(p, direction, dt, fast);
            } else {
                float value = static_cast<float>(config.*(field.member));
                TunableParameter p{
                    field.name,
                    &value,
                    field.step,
                    field.minValue,
                    field.maxValue
                };

                adjustTunable(p, direction, dt, fast);
                config.*(field.member) = static_cast<FieldType>(std::lround(value));
            }
        }
        ++index;
    });
}

template <typename Config>
void drawTunables(
    TextCursor& cursor,
    const Config& config,
    std::size_t selected,
    Color normalColor = LIGHTGRAY,
    Color selectedColor = YELLOW
) {
    std::size_t index = 0;

    forEach(ConfigUiTraits<Config>::fields, [&](auto field) {
        const auto value = config.*(field.member);

        if constexpr (std::is_integral_v<std::decay_t<decltype(value)>>) {
            cursor.draw(
                TextFormat("%s: %d%s",
                    field.name,
                    static_cast<int>(value),
                    (index == selected ? " <" : "")
                ),
                16,
                (index == selected ? selectedColor : normalColor)
            );
        } else {
            cursor.draw(
                TextFormat("%s: %.2f%s",
                    field.name,
                    static_cast<float>(value),
                    (index == selected ? " <" : "")
                ),
                16,
                (index == selected ? selectedColor : normalColor)
            );
        }

        ++index;
    });
}

} // namespace simfw::ui
