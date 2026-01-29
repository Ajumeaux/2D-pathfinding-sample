# Platformer PathFinder (C++)

A **platformer-oriented pathfinding** prototype in C++ based on a **3-step graph construction**:

1. **Walk nodes**: detect positions where an agent can stand and move horizontally on a platform.
2. **Drop edges**: detect valid falls from platform edges to a walk node on a different platform.
3. **Jump edges**: simulate jump trajectories to land on a walk node on a different platform.

This repository is intentionally kept small as a **code sample** extracted from a larger 2D side-scroller project.  
It focuses on the **graph-building logic** (walk / drop / jump) and a minimal CLI example to prove it compiles.

---

## Key ideas

### 1) Walk graph
A "walk node" exists at grid coordinate `(x, y)` (feet position) when:

- the cell below `(x, y+1)` is **solid**
- the cell at `(x, y)` is **empty**

Walk edges connect adjacent nodes `(x±1, y)` **on the same platform**.

### 2) Drop edges (falling)
Drop edges are created from **platform ends only** (leftmost / rightmost walkable positions of a platform).  
The algorithm searches downward near the edge and links to the first valid landing node found on a **different platform**.

### 3) Jump edges
Jump edges are discovered by simulating multiple **jump profiles** (horizontal speed + initial vertical speed) and sampling the trajectory over time.

A jump is considered valid when:
- the landing position has **full support** under the agent’s feet (same target platform under the whole feet width)
- the landing platform is **different** from the origin platform
- the agent’s bounding box has **clearance** (no collision in the sampled positions)

For debugging and visualization, the code can optionally store:
- jump start nodes
- landing positions
- sampled trajectories

---

## Build

### Requirements
- CMake >= 3.20
- A C++20 compiler (GCC/Clang/MSVC)

### Compile
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

---

## Run the CLI example

The CLI uses a tiny ASCII grid example (hard-coded) and asks for start/goal positions.

```bash
./build/pathfinder_cli
```

Then enter:
- `Start (x y)`
- `Goal (x y)`

The output prints a list of path steps:
- `Stand`
- `Drop`
- `Jump` (with jump profile id)

> Note: Coordinates are expressed in **world units** where **1 tile = 1 unit**.

---

## Project structure

```text
.
├── CMakeLists.txt
├── include/
│   ├── Grid.hpp        # Vec2 + IGrid interface (no external dependencies)
│   └── PathFinder.hpp
├── src/
│   └── PathFinder.cpp
└── examples/
    └── cli_main.cpp
```

---

## Design notes

### Platform IDs
Platforms are identified by scanning solid tiles that have empty space above them and grouping contiguous tiles.
This provides a fast way to:
- identify "same platform" movements
- filter drop/jump targets (must be different platform)

### Costs
- Walk edges: constant cost (1 tile)
- Drop edges: vertical distance (delta Y)
- Jump edges: flight time (number of samples × dt)

### Why a graph instead of tile A*?
Because platformers are not just "walkable tiles". Movement depends on:
- platform boundaries
- falling rules
- jump physics and clearance

So the algorithm first generates a **movement graph** and then runs A* on it.

---

## Limitations

- This is a prototype / code sample: the graph is built from a grid snapshot.
- Path search uses a naive nearest-node lookup (linear scan).
- Jump discovery can be expensive depending on the number of jump profiles and the size of the map.
- **Stairs / step-up logic is not implemented yet** (planned).

---

## Roadmap / Improvement ideas

### Performance improvements (important for large or dynamic worlds)
- **Keep the best jump edge online** (avoid generating many jump edges then filtering them afterward).
- Reduce the number of **jump profiles** (smarter sampling of dx / heights).
- Use a faster `nearest()` lookup:
    - cell→node direct mapping with local neighborhood search
    - spatial hash / grid buckets
- Optimize collision checks:
    - edge/contour sampling with fallback full scan
    - chunk-aware rectangle queries for chunked worlds
- Incremental updates:
    - rebuild only affected platforms/edges after local terrain changes.

### Movement features
- **Stairs / step-up**: allow walking up small height differences (common in platformers).
- Ledges / one-way platforms (jump-through).
- Optional diagonal movement rules (if relevant).
- "Coyote time" / jump buffering rules (game-feel features that influence navigation constraints).

### Quality / usability
- Add unit tests for:
    - platform detection
    - drop landings
    - jump validity (support + clearance)
- Optional debug visualization output (JSON, simple CSV, etc.).
- Document the jump physics assumptions (gravity, dt, bounding box model).

---

## License

Do whatever you want with this code — just keep the copyright notice.  
Released under the MIT License. See the `LICENSE` file.
