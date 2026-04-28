# agents_cpu first version

Copy these files over the repository root.

Adds / updates:
- top-level `CMakeLists.txt` to include `simulations/agents_cpu`
- `simulations/agents_cpu/include/Agent.hpp`
- `simulations/agents_cpu/include/Simulation.hpp`
- `simulations/agents_cpu/include/SimulationUiTraits.hpp`
- `simulations/agents_cpu/src/Simulation.cpp`
- `simulations/agents_cpu/main.cpp`

Features:
- steering agents with seek / arrive behavior
- separation using `simfw::SpatialHashGrid`
- naive vs spatial-grid toggle
- shared `SimulationBase`
- shared debug UI traits
- shared simulation controls
- runtime tunables
- left-click target assignment
