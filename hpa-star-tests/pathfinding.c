#include "pathfinding.h"
#include "raylib.h"
#include <stdlib.h>

// State
Entrance entrances[MAX_ENTRANCES];
int entranceCount = 0;
GraphEdge graphEdges[MAX_EDGES];
int graphEdgeCount = 0;
Point path[MAX_PATH];
int pathLength = 0;
int nodesExplored = 0;
double lastPathTime = 0.0;
Point startPos = {-1, -1};
Point goalPos = {-1, -1};
AStarNode nodeData[GRID_HEIGHT][GRID_WIDTH];
bool chunkDirty[CHUNKS_Y][CHUNKS_X];

int Heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void MarkChunkDirty(int cellX, int cellY) {
    int cx = cellX / CHUNK_SIZE;
    int cy = cellY / CHUNK_SIZE;
    if (cx >= 0 && cx < CHUNKS_X && cy >= 0 && cy < CHUNKS_Y) {
        chunkDirty[cy][cx] = true;
        needsRebuild = true;
    }
}

static void AddEntrance(int x, int y, int chunk1, int chunk2) {
    if (entranceCount < MAX_ENTRANCES)
        entrances[entranceCount++] = (Entrance){x, y, chunk1, chunk2};
}

static void AddEntrancesForRun(int startX, int startY, int length, int horizontal, int chunk1, int chunk2) {
    int remaining = length, pos = 0;
    while (remaining > 0) {
        int segLen = (remaining > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : remaining;
        int mid = pos + segLen / 2;
        int ex = horizontal ? startX + mid : startX;
        int ey = horizontal ? startY : startY + mid;
        AddEntrance(ex, ey, chunk1, chunk2);
        pos += segLen;
        remaining -= segLen;
    }
}

void BuildEntrances(void) {
    entranceCount = 0;
    for (int cy = 0; cy < CHUNKS_Y - 1; cy++) {
        for (int cx = 0; cx < CHUNKS_X; cx++) {
            int borderY = (cy + 1) * CHUNK_SIZE;
            int startX = cx * CHUNK_SIZE;
            int chunk1 = cy * CHUNKS_X + cx;
            int chunk2 = (cy + 1) * CHUNKS_X + cx;
            int runStart = -1;
            for (int i = 0; i < CHUNK_SIZE; i++) {
                int x = startX + i;
                bool open = (grid[borderY - 1][x] == CELL_WALKABLE && grid[borderY][x] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    AddEntrancesForRun(startX + runStart, borderY, i - runStart, 1, chunk1, chunk2);
                    runStart = -1;
                }
            }
            if (runStart >= 0)
                AddEntrancesForRun(startX + runStart, borderY, CHUNK_SIZE - runStart, 1, chunk1, chunk2);
        }
    }
    for (int cy = 0; cy < CHUNKS_Y; cy++) {
        for (int cx = 0; cx < CHUNKS_X - 1; cx++) {
            int borderX = (cx + 1) * CHUNK_SIZE;
            int startY = cy * CHUNK_SIZE;
            int chunk1 = cy * CHUNKS_X + cx;
            int chunk2 = cy * CHUNKS_X + (cx + 1);
            int runStart = -1;
            for (int i = 0; i < CHUNK_SIZE; i++) {
                int y = startY + i;
                bool open = (grid[y][borderX - 1] == CELL_WALKABLE && grid[y][borderX] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    AddEntrancesForRun(borderX, startY + runStart, i - runStart, 0, chunk1, chunk2);
                    runStart = -1;
                }
            }
            if (runStart >= 0)
                AddEntrancesForRun(borderX, startY + runStart, CHUNK_SIZE - runStart, 0, chunk1, chunk2);
        }
    }
    for (int cy = 0; cy < CHUNKS_Y; cy++)
        for (int cx = 0; cx < CHUNKS_X; cx++)
            chunkDirty[cy][cx] = false;
    needsRebuild = false;
}

int AStarChunk(int sx, int sy, int gx, int gy, int minX, int minY, int maxX, int maxY) {
    for (int y = minY; y < maxY; y++)
        for (int x = minX; x < maxX; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};

    nodeData[sy][sx].g = 0;
    nodeData[sy][sx].f = Heuristic(sx, sy, gx, gy);
    nodeData[sy][sx].open = true;

    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};

    while (1) {
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = minY; y < maxY; y++)
            for (int x = minX; x < maxX; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        if (bestX < 0) return -1;
        if (bestX == gx && bestY == gy) return nodeData[gy][gx].g;
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        for (int i = 0; i < 4; i++) {
            int nx = bestX + dx[i], ny = bestY + dy[i];
            if (nx < minX || nx >= maxX || ny < minY || ny >= maxY) continue;
            if (grid[ny][nx] == CELL_WALL || nodeData[ny][nx].closed) continue;
            int ng = nodeData[bestY][bestX].g + 1;
            if (ng < nodeData[ny][nx].g) {
                nodeData[ny][nx].g = ng;
                nodeData[ny][nx].f = ng + Heuristic(nx, ny, gx, gy);
                nodeData[ny][nx].open = true;
            }
        }
    }
}

void BuildGraph(void) {
    graphEdgeCount = 0;
    double startTime = GetTime();
    for (int chunk = 0; chunk < CHUNKS_X * CHUNKS_Y; chunk++) {
        int cx = chunk % CHUNKS_X;
        int cy = chunk / CHUNKS_X;
        int minX = cx * CHUNK_SIZE;
        int minY = cy * CHUNK_SIZE;
        int maxX = (cx + 1) * CHUNK_SIZE + 1;
        int maxY = (cy + 1) * CHUNK_SIZE + 1;
        if (maxX > GRID_WIDTH) maxX = GRID_WIDTH;
        if (maxY > GRID_HEIGHT) maxY = GRID_HEIGHT;

        int chunkEnts[128];
        int numEnts = 0;
        for (int i = 0; i < entranceCount && numEnts < 128; i++) {
            if (entrances[i].chunk1 == chunk || entrances[i].chunk2 == chunk)
                chunkEnts[numEnts++] = i;
        }
        for (int i = 0; i < numEnts; i++) {
            for (int j = i + 1; j < numEnts; j++) {
                int e1 = chunkEnts[i], e2 = chunkEnts[j];
                int cost = AStarChunk(entrances[e1].x, entrances[e1].y, entrances[e2].x, entrances[e2].y, minX, minY, maxX, maxY);
                if (cost >= 0 && graphEdgeCount < MAX_EDGES - 1) {
                    graphEdges[graphEdgeCount++] = (GraphEdge){e1, e2, cost};
                    graphEdges[graphEdgeCount++] = (GraphEdge){e2, e1, cost};
                }
            }
        }
    }
    TraceLog(LOG_INFO, "Built graph: %d edges in %.2fms", graphEdgeCount, (GetTime() - startTime) * 1000);
}

void RunAStar(void) {
    if (startPos.x < 0 || goalPos.x < 0) return;
    pathLength = 0;
    nodesExplored = 0;
    double startTime = GetTime();

    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};

    nodeData[startPos.y][startPos.x].g = 0;
    nodeData[startPos.y][startPos.x].f = Heuristic(startPos.x, startPos.y, goalPos.x, goalPos.y);
    nodeData[startPos.y][startPos.x].open = true;

    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};

    while (1) {
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = 0; y < GRID_HEIGHT; y++)
            for (int x = 0; x < GRID_WIDTH; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        if (bestX < 0) break;
        if (bestX == goalPos.x && bestY == goalPos.y) {
            int cx = goalPos.x, cy = goalPos.y;
            while (cx >= 0 && cy >= 0 && pathLength < MAX_PATH) {
                path[pathLength++] = (Point){cx, cy};
                int px = nodeData[cy][cx].parentX;
                int py = nodeData[cy][cx].parentY;
                cx = px; cy = py;
            }
            break;
        }
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        nodesExplored++;
        for (int i = 0; i < 4; i++) {
            int nx = bestX + dx[i], ny = bestY + dy[i];
            if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT) continue;
            if (grid[ny][nx] == CELL_WALL || nodeData[ny][nx].closed) continue;
            int ng = nodeData[bestY][bestX].g + 1;
            if (ng < nodeData[ny][nx].g) {
                nodeData[ny][nx].g = ng;
                nodeData[ny][nx].f = ng + Heuristic(nx, ny, goalPos.x, goalPos.y);
                nodeData[ny][nx].parentX = bestX;
                nodeData[ny][nx].parentY = bestY;
                nodeData[ny][nx].open = true;
            }
        }
    }
    lastPathTime = (GetTime() - startTime) * 1000.0;
}
