#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "PathFinder.hpp"
#include "Grid.hpp"

class AsciiGrid : public IGrid {
public:
  explicit AsciiGrid(std::vector<std::string> rows) : rows_(std::move(rows)) {}

  int width() const override { return rows_.empty() ? 0 : (int)rows_[0].size(); }
  int height() const override { return (int)rows_.size(); }

  bool isSolid(int x, int y) const override {
    if (y < 0 || y >= height() || x < 0 || x >= width()) return true; // outside = solid
    return rows_[y][x] == '#';
  }

private:
  std::vector<std::string> rows_;
};

static Vec2 readVec2(const char* label) {
  std::cout << label << " (x y): ";
  std::string line;
  std::getline(std::cin, line);
  std::istringstream iss(line);
  Vec2 p{};
  iss >> p.x >> p.y;
  return p;
}

int main() {
  // Small demo level: bottom platforms + gaps
  AsciiGrid grid({
    "........................",
    "........................",
    ".............####.......",
    "........................",
    ".....####...............",
    "........................",
    "########################"
  });

  PathFinder pf(
    grid,
    Vec2{0.9f, 1.8f},  // agent size
    /*jumpSpeed*/ 6.f,
    /*horizontalSpeed*/ 5.f,
    /*maxAirSpeed*/ 8.f,
    /*gravity*/ 9.81f
  );

  pf.init();       // build jump profiles etc.
  pf.buildGraph(); // compute nodes/edges

  std::cout << "Map size: " << grid.width() << "x" << grid.height() << "\n";
  std::cout << "Enter coordinates in world units (same as grid cells).\n";

  Vec2 start = readVec2("Start");
  Vec2 goal  = readVec2("Goal");

  auto path = pf.findPath(start, goal);
  if (path.empty()) {
    std::cout << "No path found.\n";
    return 0;
  }

  std::cout << "Path steps: " << path.size() << "\n";
  for (const auto& s : path) {
    const char* t = (s.type == PathFinder::Stand ? "Stand"
                    : s.type == PathFinder::Drop ? "Drop" : "Jump");
    std::cout << "- " << t
              << " -> (" << s.pos.x << ", " << s.pos.y << ")";
    if (s.type == PathFinder::Jump) std::cout << " [profile " << s.jumpProfileId << "]";
    std::cout << "\n";
  }
}
