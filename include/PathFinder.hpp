#pragma once

#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>

#include "Grid.hpp"

class PathFinder {
public:
    enum EdgeType { Stand = 0, Drop = 1, Jump = 2 };

    struct Node {
        int  cellX = 0;
        int  cellY = 0;       // grid coords of "feet" cell
        Vec2 worldPos{};      // {cellX + 0.5, cellY + 0.5}
        int  platformId = -1; // id of the platform under these feet
    };

    struct JumpProfile {
        float vx = 0.f;        // horizontal speed
        float initialVy = 0.f; // initial vertical speed
    };

    struct Edge {
        int      to = -1;           // target node index
        float    cost = 0.f;        // movement cost (time for jumps, distance for walk, deltaY for drop)
        EdgeType type = Stand;
        int      toPlatformId = -1; // platformId of the destination node
        int      jumpProfileId = -1;
        Vec2     landPos{};         // precise landing position for jumps / drop offset position
    };

    struct PathStep {
        Vec2     pos{};
        EdgeType type = Stand;
        int      jumpProfileId = -1;
    };

    PathFinder(const IGrid& grid,
               Vec2 agentSize,
               float jumpSpeed,
               float horizontalSpeed,
               float maxAirSpeed,
               float gravity);

    // Build jump profiles (call once before buildGraph)
    void init();

    // Build navigation graph (stand/drop/jump edges), alloc A* caches
    void buildGraph();

    // A* on the graph; returns world-space steps
    std::vector<PathStep> findPath(Vec2 start, Vec2 goal) const;

    // Debug helpers (optional)
    const std::vector<Vec2>& getWalkNodes() const;
    const std::vector<Vec2>& getDropNodes() const;
    const std::vector<Vec2>& getJumpNodes() const;
    const std::vector<Vec2>& getLandingNodes() const;
    const std::vector<JumpProfile>& getJumpProfiles() const;
    const std::vector<std::vector<Vec2>>& getJumpTrajectories() const;

private:
    const IGrid& _grid;

    Vec2  _agentSize{};
    float _jumpSpeed = 0.f;
    float _horizSpeed = 0.f;
    float _maxAirSpeed = 0.f;
    float _gravity = 9.81f;

    int _width = 0;
    int _height = 0;

    std::vector<Node> _nodes;
    std::unordered_map<int, int> _nodeIndex;
    std::vector<std::vector<Edge>> _adj;

    // Debug / visualization
    std::vector<Vec2> _walkNodes;
    std::vector<Vec2> _dropNodes;
    std::vector<Vec2> _jumpNodes;
    std::vector<Vec2> _landingNodes;
    std::vector<std::vector<Vec2>> _jumpTrajectories;

    std::vector<JumpProfile> _jumpProfiles;

    // A* caches
    mutable std::vector<float> _gScore;
    mutable std::vector<int>   _cameFrom;
    mutable std::vector<int>   _stamp;
    mutable int                _curStamp = 1;

private:
    void computePlatformIds(std::vector<std::vector<int>>& outPlatformId) const;

    void buildWalkNodes();
    void buildDropNodes();
    void buildJumpNodes();

    float heuristic(int aIdx, int bIdx) const;

    void addWalkEdgesAround(int nodeIndex);
};
