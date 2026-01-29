#include "PathFinder.hpp"

#include <cstdint>
#include <queue>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

static inline int clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

PathFinder::PathFinder(const IGrid& grid,
                       Vec2 agentSize,
                       float jumpSpeed,
                       float horizontalSpeed,
                       float maxAirSpeed,
                       float gravity)
    : _grid(grid),
      _agentSize(agentSize),
      _jumpSpeed(jumpSpeed),
      _horizSpeed(horizontalSpeed),
      _maxAirSpeed(maxAirSpeed),
      _gravity(gravity),
      _width(grid.width()),
      _height(grid.height()) {}

void PathFinder::init() {
    _jumpProfiles.clear();

    // 1) Desired horizontal distances (in tiles)
    const std::vector<float> desiredDx = {
        0.5f, 1.f, 1.5f, 2.f, 2.5f, 3.f, 3.5f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f,
        10.f, 11.f, 12.f, 13.f, 14.f, 15.f
    };

    // 2) Desired jump heights (0.5 increments)
    int hMax = static_cast<int>(std::round(_jumpSpeed));
    std::vector<float> desiredHeights;
    desiredHeights.reserve(hMax * 2);

    for (int h = 1; h <= hMax * 2; ++h) {
        desiredHeights.push_back(static_cast<float>(h) / 2.f);
    }

    // 3) Build profiles: dx × height × direction
    const std::vector<int> dirs = { +1, -1 };

    for (float dxTiles : desiredDx) {
        float wantX = dxTiles; // 1 tile = 1 world unit

        for (float h : desiredHeights) {
            float vy0 = std::sqrt(2.f * _gravity * h);
            float Tvol = 2.f * vy0 / _gravity; // total flight duration

            float baseVx = (Tvol > 0.f) ? (wantX / Tvol) : 0.f;

            for (int d : dirs) {
                float vx0 = static_cast<float>(d) * baseVx;
                vx0 = std::clamp(vx0, -_maxAirSpeed, +_maxAirSpeed);

                _jumpProfiles.push_back({ vx0, vy0 });
            }
        }
    }
}

void PathFinder::computePlatformIds(std::vector<std::vector<int>>& outPlatformId) const {
    const int W = _width;
    const int H = _height;

    outPlatformId.assign(H, std::vector<int>(W, -1));
    std::vector<std::vector<bool>> seen(H, std::vector<bool>(W, false));

    int nextPid = 0;

    // A platform tile is "solid" at (x,y) with non-solid above (x,y-1).
    for (int y = 1; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!seen[y][x] && _grid.isSolid(x, y) && !_grid.isSolid(x, y - 1)) {
                std::queue<std::pair<int, int>> q;
                seen[y][x] = true;
                q.push({ x, y });

                while (!q.empty()) {
                    const auto [cx, cy] = q.front();
                    q.pop();

                    outPlatformId[cy][cx] = nextPid;

                    for (int dx : { -1, +1 }) {
                        int nx = cx + dx;
                        if (nx < 0 || nx >= W) continue;

                        if (!seen[cy][nx] &&
                            _grid.isSolid(nx, cy) &&
                            !_grid.isSolid(nx, cy - 1)) {
                            seen[cy][nx] = true;
                            q.push({ nx, cy });
                        }
                    }
                }

                ++nextPid;
            }
        }
    }
}

void PathFinder::buildGraph() {
    // 1) Compute platform ids
    std::vector<std::vector<int>> platformId;
    computePlatformIds(platformId);

    // 2) Collect "stand" nodes (feet positions above solid tiles)
    _nodes.clear();
    _nodeIndex.clear();

    for (int y = 1; y < _height; ++y) {
        for (int x = 0; x < _width; ++x) {
            if (_grid.isSolid(x, y) && !_grid.isSolid(x, y - 1)) {
                Node n{};
                n.cellX = x;
                n.cellY = y - 1;
                n.worldPos = { static_cast<float>(x) + 0.5f, static_cast<float>(y - 1) + 0.5f };
                n.platformId = platformId[y][x];

                int idx = static_cast<int>(_nodes.size());
                _nodes.push_back(n);

                int key = n.cellY * _width + n.cellX;
                _nodeIndex[key] = idx;
            }
        }
    }

    int N = static_cast<int>(_nodes.size());
    _adj.assign(N, {});

    // 3) Platform bounds (minX/maxX per platform)
    int maxPid = -1;
    for (const auto& n : _nodes) maxPid = std::max(maxPid, n.platformId);

    std::vector<int> minX(maxPid + 1, std::numeric_limits<int>::max());
    std::vector<int> maxX(maxPid + 1, std::numeric_limits<int>::min());

    for (const auto& n : _nodes) {
        int pid = n.platformId;
        minX[pid] = std::min(minX[pid], n.cellX);
        maxX[pid] = std::max(maxX[pid], n.cellX);
    }

    // 4) Walk edges (+/-1 on same platform)
    for (int i = 0; i < N; ++i) {
        const auto& a = _nodes[i];

        for (int dx : { -1, +1 }) {
            int nx = a.cellX + dx;
            int ny = a.cellY;
            if (nx < 0 || nx >= _width) continue;

            int key = ny * _width + nx;
            auto it = _nodeIndex.find(key);
            if (it == _nodeIndex.end()) continue;

            int j = it->second;
            if (_nodes[j].platformId != a.platformId) continue;

            Edge e{};
            e.to = j;
            e.cost = 1.f;
            e.type = Stand;
            e.toPlatformId = a.platformId;
            e.jumpProfileId = -1;
            e.landPos = _nodes[j].worldPos; // not really used for Stand, but nice default
            _adj[i].push_back(e);
        }
    }

    // 5) Drop edges (only from platform ends)
    float halfAgent = _agentSize.x * 0.5f;
    constexpr float halfCell = 0.5f;
    constexpr float epsilon = 0.01f;
    const float dropOffset = halfAgent + halfCell + epsilon;

    for (int i = 0; i < N; ++i) {
        const auto& node = _nodes[i];
        int cellX = node.cellX;
        int cellY = node.cellY;
        int pid = node.platformId;

        bool isLeftEdge = (cellX == minX[pid]);
        bool isRightEdge = (cellX == maxX[pid]);
        if (!isLeftEdge && !isRightEdge) continue;

        for (float sign : { -1.f, +1.f }) {
            if (sign < 0 && !isLeftEdge) continue;
            if (sign > 0 && !isRightEdge) continue;

            Vec2 startOff = node.worldPos;
            startOff.x += sign * dropOffset;

            int scanX = cellX + (sign > 0 ? +1 : -1);
            int diagY = cellY + 1;
            if (scanX < 0 || scanX >= _width || diagY >= _height) continue;
            if (_grid.isSolid(scanX, diagY)) continue;

            int y = cellY + 1;
            while (y < _height && _grid.isSolid(scanX, y)) ++y;   // follow wall
            while (y < _height && !_grid.isSolid(scanX, y)) ++y;  // fall

            if (y >= _height) continue;

            int destY = y - 1;
            if (destY == cellY) continue;

            int key = destY * _width + scanX;
            auto it = _nodeIndex.find(key);
            if (it == _nodeIndex.end()) continue;

            int j = it->second;
            if (_nodes[j].platformId == pid) continue;

            Edge e{};
            e.landPos = startOff;
            e.to = j;
            e.cost = static_cast<float>(destY - cellY);
            e.type = Drop;
            e.toPlatformId = _nodes[j].platformId;
            e.jumpProfileId = -1;
            _adj[i].push_back(e);
        }
    }

    // 6) Jump edges (analytical sampling)
    _jumpNodes.clear();
    _landingNodes.clear();
    _jumpTrajectories.clear();

    constexpr float eps = 0.2f;
    float halfW = _agentSize.x * 0.5f;
    float halfH = _agentSize.y * 0.5f;
    float halfW_eps = halfW + eps;
    float halfH_eps = halfH + eps;

    // We will reuse platformId matrix for landing checks
    for (int i = 0; i < N; ++i) {
        const auto& a = _nodes[i];

        bool wallLeft = (a.cellX == 0) || _grid.isSolid(a.cellX - 1, a.cellY);
        bool wallRight = (a.cellX == _width - 1) || _grid.isSolid(a.cellX + 1, a.cellY);
        if (wallLeft || wallRight) continue;

        bool canStand = false;
        for (const auto& e : _adj[i]) {
            if (e.type == Stand) { canStand = true; break; }
        }
        if (!canStand) continue;

        const float startCx = a.worldPos.x;
        const float startCy = a.worldPos.y - halfH + 0.5f;

        for (int pidIndex = 0; pidIndex < static_cast<int>(_jumpProfiles.size()); ++pidIndex) {
            const auto& prof = _jumpProfiles[pidIndex];
            const float absVx = std::abs(prof.vx);
            const float vy0 = prof.initialVy;

            const float totalT = 2.f * vy0 / _gravity;
            constexpr float dtMax = 0.016f;
            const int maxStepsBase = static_cast<int>(std::ceil(totalT / dtMax));

            for (int dir = +1; dir >= -1; dir -= 2) {
                float vx0 = static_cast<float>(dir) * absVx;
                int maxSteps = maxStepsBase;

                bool landed = false;
                std::vector<Vec2> traj;
                traj.reserve(static_cast<size_t>(maxSteps));

                for (int step = 1; step <= maxSteps && !landed; ++step) {
                    float t = static_cast<float>(step) * dtMax;
                    if (t > totalT) t = totalT;

                    float px = startCx + vx0 * t;
                    float py = startCy - vy0 * t + 0.5f * _gravity * t * t;

                    traj.push_back({ px, py });

                    float left = px - halfW_eps;
                    float right = px + halfW_eps;
                    float top = py - halfH_eps;
                    float bottom = py + halfH;

                    int minXtile = clampi(static_cast<int>(std::floor(left)), 0, _width - 1);
                    int maxXtile = clampi(static_cast<int>(std::floor(right)), 0, _width - 1);
                    int minYtile = clampi(static_cast<int>(std::floor(top)), 0, _height - 1);
                    int maxYtile = clampi(static_cast<int>(std::floor(bottom)), 0, _height - 1);

                    // --- Landing test: full support under feet (no eps) ---
                    float leftSupport = px - halfW;
                    float rightSupport = px + halfW;
                    float bottomY = py + halfH;
                    int belowTileY = static_cast<int>(std::floor(bottomY));

                    if (belowTileY >= 0 && belowTileY < _height) {
                        int firstX = static_cast<int>(std::floor(leftSupport));
                        int lastX = static_cast<int>(std::floor(rightSupport));

                        bool fullSupport = true;
                        int supportPid = -1;
                        int landingTx = -1;

                        for (int tx = firstX; tx <= lastX; ++tx) {
                            if (tx < 0 || tx >= _width) { fullSupport = false; break; }

                            float tileL = static_cast<float>(tx);
                            float tileR = tileL + 1.f;

                            float overlapL = std::max(tileL, leftSupport);
                            float overlapR = std::min(tileR, rightSupport);
                            if (overlapR <= overlapL) continue;

                            int pid = platformId[belowTileY][tx];
                            if (pid < 0 || pid == a.platformId) { fullSupport = false; break; }

                            if (supportPid < 0) supportPid = pid;
                            else if (pid != supportPid) { fullSupport = false; break; }

                            if (landingTx < 0) landingTx = tx;
                        }

                        if (fullSupport && landingTx >= 0) {
                            // --- Clearance check (with eps) ---
                            bool clearance = true;

                            for (int tx = minXtile; tx <= maxXtile && clearance; ++tx) {
                                float tileL = static_cast<float>(tx);
                                float tileR = tileL + 1.f;
                                float overlapLx = std::max(tileL, left);
                                float overlapRx = std::min(tileR, right);
                                if (overlapRx <= overlapLx) continue;

                                for (int ty = minYtile; ty <= maxYtile; ++ty) {
                                    if (ty == belowTileY) continue;

                                    float tileT = static_cast<float>(ty);
                                    float tileB = tileT + 1.f;
                                    float overlapTy = std::max(tileT, top);
                                    float overlapBy = std::min(tileB, bottom);

                                    if (overlapBy <= overlapTy) continue;

                                    if (_grid.isSolid(tx, ty)) {
                                        clearance = false;
                                        break;
                                    }
                                }
                            }

                            if (!clearance) continue;

                            int newPid = supportPid;
                            if (newPid >= 0 && newPid != a.platformId) {
                                int key = belowTileY * _width + landingTx;
                                auto itNode = _nodeIndex.find(key);

                                int toIndex = -1;
                                if (itNode == _nodeIndex.end()) {
                                    Node newNode{};
                                    newNode.cellX = landingTx;
                                    newNode.cellY = belowTileY - 1;
                                    newNode.worldPos = {
                                        static_cast<float>(landingTx) + 0.5f,
                                        static_cast<float>(belowTileY - 1) + 0.5f
                                    };
                                    newNode.platformId = newPid;

                                    toIndex = static_cast<int>(_nodes.size());
                                    _nodes.push_back(newNode);
                                    _adj.emplace_back();
                                    _nodeIndex[key] = toIndex;

                                    addWalkEdgesAround(toIndex);
                                } else {
                                    toIndex = itNode->second;
                                }

                                Edge e{};
                                e.to = toIndex;
                                e.cost = static_cast<float>(step) * dtMax;
                                e.type = Jump;
                                e.toPlatformId = newPid;
                                e.jumpProfileId = pidIndex;
                                e.landPos = { px, py };
                                _adj[i].push_back(e);

                                _jumpNodes.push_back(a.worldPos);
                                _landingNodes.push_back(e.landPos);
                                _jumpTrajectories.push_back(traj);

                                landed = true;
                            }
                        }

                        if (landed) continue;
                    }

                    // --- Collision in flight (with eps) ---
                    bool hit = false;
                    for (int tx = minXtile; tx <= maxXtile && !hit; ++tx) {
                        float tileL = static_cast<float>(tx);
                        float tileR = tileL + 1.f;
                        float overlapLx = std::max(tileL, left);
                        float overlapRx = std::min(tileR, right);
                        if (overlapRx <= overlapLx) continue;

                        for (int ty = minYtile; ty <= maxYtile; ++ty) {
                            float tileT = static_cast<float>(ty);
                            float tileB = tileT + 1.f;
                            float overlapTy = std::max(tileT, top);
                            float overlapBy = std::min(tileB, bottom);
                            if (overlapBy <= overlapTy) continue;

                            if (_grid.isSolid(tx, ty)) {
                                hit = true;
                                break;
                            }
                        }
                    }
                    if (hit) break;
                }
            }
        }
    }

    // Post-processing: keep only best jump trajectory per (fromPlatform, toPlatform, direction)
    std::vector<std::vector<Vec2>> newTraj;
    std::vector<Vec2> newJumpNodes;
    std::vector<Vec2> newLandingNodes;

    using Key = std::uint64_t;
    auto makeKey = [](int fromPID, int toPID, int dirFlag) -> Key {
        return (static_cast<Key>(fromPID) << 21) |
               (static_cast<Key>(toPID) << 1) |
               static_cast<Key>(dirFlag & 1);
    };

    struct KeepInfo { int node; std::size_t edge; std::size_t dbg; float cost; };
    std::unordered_map<Key, KeepInfo> bestEdge;

    std::size_t dbgIdx = 0;

    for (int n = 0; n < static_cast<int>(_adj.size()); ++n) {
        int fromPID = _nodes[n].platformId;
        auto& edges = _adj[n];

        for (std::size_t ei = 0; ei < edges.size(); ++ei) {
            Edge& e = edges[ei];
            if (e.type != Jump) { continue; }

            int dirFlag = (_nodes[n].worldPos.x < e.landPos.x) ? 0 : 1;
            Key key = makeKey(fromPID, e.toPlatformId, dirFlag);

            auto it = bestEdge.find(key);
            if (it == bestEdge.end() || e.cost < it->second.cost) {
                bestEdge[key] = { n, ei, dbgIdx, e.cost };
            }

            ++dbgIdx;
        }
    }

    dbgIdx = 0;

    for (int n = 0; n < static_cast<int>(_adj.size()); ++n) {
        int fromPID = _nodes[n].platformId;
        auto& edges = _adj[n];

        std::vector<Edge> filtered;
        filtered.reserve(edges.size());

        for (std::size_t ei = 0; ei < edges.size(); ++ei) {
            Edge& e = edges[ei];

            if (e.type == Jump) {
                int dirFlag = (_nodes[n].worldPos.x < e.landPos.x) ? 0 : 1;
                Key key = makeKey(fromPID, e.toPlatformId, dirFlag);
                const KeepInfo& info = bestEdge.at(key);

                // keep only the best edge for this key
                if (info.node == n && info.edge == ei) {
                    if (dbgIdx < _jumpTrajectories.size()) {
                        newTraj.push_back(_jumpTrajectories[dbgIdx]);
                        newJumpNodes.push_back(_jumpNodes[dbgIdx]);
                        newLandingNodes.push_back(_landingNodes[dbgIdx]);
                    }
                    filtered.push_back(e);
                }
                ++dbgIdx;
            } else {
                filtered.push_back(e);
            }
        }

        edges.swap(filtered);
    }

    _jumpTrajectories.swap(newTraj);
    _jumpNodes.swap(newJumpNodes);
    _landingNodes.swap(newLandingNodes);

    // Allocate A* caches after graph build
    const int NN = static_cast<int>(_nodes.size());
    _gScore.assign(NN, std::numeric_limits<float>::infinity());
    _cameFrom.assign(NN, -1);
    _stamp.assign(NN, 0);

    buildWalkNodes();
    buildDropNodes();
    buildJumpNodes();
}

float PathFinder::heuristic(int aIdx, int bIdx) const {
    const auto& a = _nodes[aIdx].worldPos;
    const auto& b = _nodes[bIdx].worldPos;
    return std::fabs(a.x - b.x) + std::fabs(a.y - b.y);
}

std::vector<PathFinder::PathStep> PathFinder::findPath(Vec2 start, Vec2 goal) const {
    auto nearest = [&](const Vec2& p) {
        int best = -1;
        float bd = std::numeric_limits<float>::infinity();

        for (int i = 0; i < static_cast<int>(_nodes.size()); ++i) {
            const float d = std::fabs(_nodes[i].worldPos.x - p.x) +
                            std::fabs(_nodes[i].worldPos.y - p.y);
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    };

    int startIdx = nearest(start);
    int goalIdx = nearest(goal);
    if (startIdx < 0 || goalIdx < 0) return {};

    const int N = static_cast<int>(_nodes.size());
    ++_curStamp;
    if (_curStamp > 100000000) {
        std::fill(_stamp.begin(), _stamp.end(), 0);
        _curStamp = 1;
    }

    std::fill(_gScore.begin(), _gScore.end(), std::numeric_limits<float>::infinity());
    std::fill(_cameFrom.begin(), _cameFrom.end(), -1);

    _gScore[startIdx] = 0.f;

    using PQ = std::pair<float, int>; // (fScore, node)
    std::priority_queue<PQ, std::vector<PQ>, std::greater<>> open;
    open.emplace(heuristic(startIdx, goalIdx), startIdx);

    while (!open.empty()) {
        auto [f, cur] = open.top();
        open.pop();

        if (cur == goalIdx) break;
        if (_stamp[cur] == _curStamp) continue;
        _stamp[cur] = _curStamp;

        for (const auto& e : _adj[cur]) {
            int j = e.to;
            float cand = _gScore[cur] + e.cost;
            if (cand < _gScore[j]) {
                _gScore[j] = cand;
                _cameFrom[j] = cur;
                open.emplace(cand + heuristic(j, goalIdx), j);
            }
        }
    }

    std::vector<PathStep> path;

    if (_cameFrom[goalIdx] >= 0 || startIdx == goalIdx) {
        int cur = goalIdx;

        while (cur != -1) {
            int prev = _cameFrom[cur];

            if (prev >= 0) {
                bool found = false;
                for (const auto& e : _adj[prev]) {
                    if (e.to == cur) {
                        path.push_back({
                            (e.type == Stand ? _nodes[cur].worldPos : e.landPos),
                            e.type,
                            (e.type == Jump ? e.jumpProfileId : -1)
                        });
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    path.push_back({ _nodes[cur].worldPos, Stand, -1 });
                }
            } else {
                path.push_back({ _nodes[cur].worldPos, Stand, -1 });
            }

            cur = prev;
        }

        std::reverse(path.begin(), path.end());
    }

    return path;
}

const std::vector<Vec2>& PathFinder::getWalkNodes() const { return _walkNodes; }
const std::vector<Vec2>& PathFinder::getDropNodes() const { return _dropNodes; }
const std::vector<Vec2>& PathFinder::getJumpNodes() const { return _jumpNodes; }
const std::vector<Vec2>& PathFinder::getLandingNodes() const { return _landingNodes; }
const std::vector<PathFinder::JumpProfile>& PathFinder::getJumpProfiles() const { return _jumpProfiles; }
const std::vector<std::vector<Vec2>>& PathFinder::getJumpTrajectories() const { return _jumpTrajectories; }

void PathFinder::buildWalkNodes() {
    _walkNodes.clear();
    for (int i = 0; i < static_cast<int>(_adj.size()); ++i) {
        for (const auto& e : _adj[i]) {
            if (e.type == Stand) { _walkNodes.push_back(_nodes[i].worldPos); break; }
        }
    }
}

void PathFinder::buildDropNodes() {
    _dropNodes.clear();
    for (int i = 0; i < static_cast<int>(_adj.size()); ++i) {
        for (const auto& e : _adj[i]) {
            if (e.type == Drop) { _dropNodes.push_back(_nodes[i].worldPos); break; }
        }
    }
}

void PathFinder::buildJumpNodes() {
    // Unique "from nodes" that have a Jump edge
    std::vector<Vec2> unique;
    unique.reserve(_adj.size());

    for (int i = 0; i < static_cast<int>(_adj.size()); ++i) {
        bool hasJump = false;
        for (const auto& e : _adj[i]) {
            if (e.type == Jump) { hasJump = true; break; }
        }
        if (hasJump) unique.push_back(_nodes[i].worldPos);
    }

    _jumpNodes.swap(unique);
}

void PathFinder::addWalkEdgesAround(int idx) {
    Node& n = _nodes[idx];
    int x = static_cast<int>(std::floor(n.worldPos.x));
    int y = static_cast<int>(std::floor(n.worldPos.y));

    for (int dx : { -1, +1 }) {
        int nx = x + dx;
        int ny = y;
        if (nx < 0 || nx >= _width) continue;

        int key = ny * _width + nx;
        auto it = _nodeIndex.find(key);
        if (it == _nodeIndex.end()) continue;

        int otherIdx = it->second;
        if (_nodes[otherIdx].platformId != n.platformId) continue;

        Edge e1{};
        e1.to = otherIdx;
        e1.cost = 1.f;
        e1.type = Stand;
        e1.toPlatformId = n.platformId;
        e1.jumpProfileId = -1;
        e1.landPos = _nodes[otherIdx].worldPos;

        Edge e2{};
        e2.to = idx;
        e2.cost = 1.f;
        e2.type = Stand;
        e2.toPlatformId = n.platformId;
        e2.jumpProfileId = -1;
        e2.landPos = n.worldPos;

        _adj[idx].push_back(e1);
        _adj[otherIdx].push_back(e2);
    }
}
