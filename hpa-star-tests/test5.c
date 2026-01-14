#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>

#define GRID_WIDTH  (128*4)
#define GRID_HEIGHT (128*4)
#define CELL_SIZE   32
#define CHUNK_SIZE  32
#define MAX_ENTRANCE_WIDTH 6
#define MAX_ENTRANCES (4096*4)

#define CHUNKS_X (GRID_WIDTH / CHUNK_SIZE)
#define CHUNKS_Y (GRID_HEIGHT / CHUNK_SIZE)
#define MAX_PATH 65536
#define MAX_EDGES 65536

typedef enum { CELL_WALKABLE, CELL_WALL } CellType;
typedef struct { int x, y; } Point;
typedef struct {
    int x, y;
    int chunk1, chunk2;
} Entrance;

typedef struct {
    int g, f;
    int parentX, parentY;
    bool open, closed;
} AStarNode;

typedef struct {
    int from, to;  // entrance indices
    int cost;
} GraphEdge;

CellType grid[GRID_HEIGHT][GRID_WIDTH];
Entrance entrances[MAX_ENTRANCES];
int entranceCount = 0;
bool chunkDirty[CHUNKS_Y][CHUNKS_X];
bool needsRebuild = false;
float zoom = 1.0f;
Vector2 offset = {0, 0};
Texture2D texGrass;
Texture2D texWall;

Point startPos = {-1, -1};
Point goalPos = {-1, -1};
Point path[MAX_PATH];
int pathLength = 0;
int nodesExplored = 0;
double lastPathTime = 0.0;
AStarNode nodeData[GRID_HEIGHT][GRID_WIDTH];
GraphEdge graphEdges[MAX_EDGES];
int graphEdgeCount = 0;
bool showGraph = false;

void InitGrid(void) {
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            grid[y][x] = CELL_WALKABLE;
}

void GenerateSparse(float density) {
    InitGrid();
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            if ((float)GetRandomValue(0, 100) / 100.0f < density)
                grid[y][x] = CELL_WALL;
    needsRebuild = true;
}

void GenerateCity(void) {
    InitGrid();

    // Horizontal walls with gaps
    for (int wy = CHUNK_SIZE; wy < GRID_HEIGHT; wy += CHUNK_SIZE / 2) {
        for (int wx = 0; wx < GRID_WIDTH; wx++) {
            int gapPos = GetRandomValue(6, 20);
            int gapSize = GetRandomValue(3, 6);
            for (int x = wx; x < wx + gapPos && x < GRID_WIDTH; x++) {
                grid[wy][x] = CELL_WALL;
                if (wy + 1 < GRID_HEIGHT) grid[wy + 1][x] = CELL_WALL;
            }
            wx += gapPos + gapSize;
        }
    }

    // Vertical walls with gaps
    for (int wx = CHUNK_SIZE; wx < GRID_WIDTH; wx += CHUNK_SIZE / 2) {
        for (int wy = 0; wy < GRID_HEIGHT; wy++) {
            int gapPos = GetRandomValue(6, 20);
            int gapSize = GetRandomValue(3, 6);
            for (int y = wy; y < wy + gapPos && y < GRID_HEIGHT; y++) {
                grid[y][wx] = CELL_WALL;
                if (wx + 1 < GRID_WIDTH) grid[y][wx + 1] = CELL_WALL;
            }
            wy += gapPos + gapSize;
        }
    }

    // Add some sparse noise on top
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            if (grid[y][x] == CELL_WALKABLE && GetRandomValue(0, 100) < 5)
                grid[y][x] = CELL_WALL;

    needsRebuild = true;
}

void GenerateMixed(void) {
    InitGrid();

    // Create zone map: 0 = forest, 1 = city
    // Use large regions (4x4 chunks each)
    int zoneSize = CHUNK_SIZE * 4;
    int zonesX = (GRID_WIDTH + zoneSize - 1) / zoneSize;
    int zonesY = (GRID_HEIGHT + zoneSize - 1) / zoneSize;
    int zones[16][16];

    for (int zy = 0; zy < zonesY && zy < 16; zy++)
        for (int zx = 0; zx < zonesX && zx < 16; zx++)
            zones[zy][zx] = GetRandomValue(0, 100) < 50 ? 1 : 0;

    // Generate city walls only in city zones
    for (int wy = CHUNK_SIZE; wy < GRID_HEIGHT; wy += CHUNK_SIZE / 2) {
        for (int wx = 0; wx < GRID_WIDTH; wx++) {
            int zx = wx / zoneSize, zy = wy / zoneSize;
            if (zx >= 16 || zy >= 16 || zones[zy][zx] == 0) {
                wx += GetRandomValue(10, 30);
                continue;
            }
            int gapPos = GetRandomValue(6, 20);
            int gapSize = GetRandomValue(3, 6);
            for (int x = wx; x < wx + gapPos && x < GRID_WIDTH; x++) {
                int zx2 = x / zoneSize;
                if (zx2 < 16 && zones[zy][zx2] == 1) {
                    grid[wy][x] = CELL_WALL;
                    if (wy + 1 < GRID_HEIGHT) grid[wy + 1][x] = CELL_WALL;
                }
            }
            wx += gapPos + gapSize;
        }
    }

    for (int wx = CHUNK_SIZE; wx < GRID_WIDTH; wx += CHUNK_SIZE / 2) {
        for (int wy = 0; wy < GRID_HEIGHT; wy++) {
            int zx = wx / zoneSize, zy = wy / zoneSize;
            if (zx >= 16 || zy >= 16 || zones[zy][zx] == 0) {
                wy += GetRandomValue(10, 30);
                continue;
            }
            int gapPos = GetRandomValue(6, 20);
            int gapSize = GetRandomValue(3, 6);
            for (int y = wy; y < wy + gapPos && y < GRID_HEIGHT; y++) {
                int zy2 = y / zoneSize;
                if (zy2 < 16 && zones[zy2][zx] == 1) {
                    grid[y][wx] = CELL_WALL;
                    if (wx + 1 < GRID_WIDTH) grid[y][wx + 1] = CELL_WALL;
                }
            }
            wy += gapPos + gapSize;
        }
    }

    // Forest (sparse) in forest zones, light noise in city
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (grid[y][x] == CELL_WALKABLE) {
                int zx = x / zoneSize, zy = y / zoneSize;
                bool isCity = (zx < 16 && zy < 16 && zones[zy][zx] == 1);
                int chance = isCity ? 3 : 15;
                if (GetRandomValue(0, 100) < chance)
                    grid[y][x] = CELL_WALL;
            }
        }
    }

    needsRebuild = true;
}

void MarkChunkDirty(int cellX, int cellY) {
    int cx = cellX / CHUNK_SIZE;
    int cy = cellY / CHUNK_SIZE;
    if (cx >= 0 && cx < CHUNKS_X && cy >= 0 && cy < CHUNKS_Y) {
        chunkDirty[cy][cx] = true;
        needsRebuild = true;
    }
}

void AddEntrance(int x, int y, int chunk1, int chunk2) {
    if (entranceCount < MAX_ENTRANCES)
        entrances[entranceCount++] = (Entrance){x, y, chunk1, chunk2};
}

int Heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void AddEntrancesForRun(int startX, int startY, int length, int horizontal, int chunk1, int chunk2) {
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

// A* within chunk bounds, returns distance or -1 if no path
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

    // For each chunk, connect all entrances that touch it
    for (int chunk = 0; chunk < CHUNKS_X * CHUNKS_Y; chunk++) {
        int cx = chunk % CHUNKS_X;
        int cy = chunk / CHUNKS_X;

        // Chunk bounds (expand by 1 to include border entrances)
        int minX = cx * CHUNK_SIZE;
        int minY = cy * CHUNK_SIZE;
        int maxX = (cx + 1) * CHUNK_SIZE + 1;
        int maxY = (cy + 1) * CHUNK_SIZE + 1;
        if (maxX > GRID_WIDTH) maxX = GRID_WIDTH;
        if (maxY > GRID_HEIGHT) maxY = GRID_HEIGHT;

        // Find all entrances touching this chunk
        int chunkEnts[128];
        int numEnts = 0;
        for (int i = 0; i < entranceCount && numEnts < 128; i++) {
            if (entrances[i].chunk1 == chunk || entrances[i].chunk2 == chunk)
                chunkEnts[numEnts++] = i;
        }

        // Connect each pair with intra-chunk A*
        for (int i = 0; i < numEnts; i++) {
            for (int j = i + 1; j < numEnts; j++) {
                int e1 = chunkEnts[i], e2 = chunkEnts[j];
                int cost = AStarChunk(
                    entrances[e1].x, entrances[e1].y,
                    entrances[e2].x, entrances[e2].y,
                    minX, minY, maxX, maxY
                );
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

void DrawCellGrid(void) {
    Rectangle src = {0, 0, 16, 16};
    float size = CELL_SIZE * zoom;
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            if (grid[y][x] == CELL_WALKABLE) {
                Rectangle dest = {offset.x + x * size, offset.y + y * size, size, size};
                DrawTexturePro(texGrass, src, dest, (Vector2){0,0}, 0, WHITE);
            }
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++)
            if (grid[y][x] == CELL_WALL) {
                Rectangle dest = {offset.x + x * size, offset.y + y * size, size, size};
                DrawTexturePro(texWall, src, dest, (Vector2){0,0}, 0, WHITE);
            }
}

void DrawChunkBoundaries(void) {
    float cellSize = CELL_SIZE * zoom;
    float chunkPixels = CHUNK_SIZE * cellSize;
    for (int cy = 0; cy <= CHUNKS_Y; cy++) {
        Vector2 s = {offset.x, offset.y + cy * chunkPixels};
        Vector2 e = {offset.x + CHUNKS_X * chunkPixels, offset.y + cy * chunkPixels};
        DrawLineEx(s, e, 3.0f, RED);
    }
    for (int cx = 0; cx <= CHUNKS_X; cx++) {
        Vector2 s = {offset.x + cx * chunkPixels, offset.y};
        Vector2 e = {offset.x + cx * chunkPixels, offset.y + CHUNKS_Y * chunkPixels};
        DrawLineEx(s, e, 3.0f, RED);
    }
}

void DrawEntrances(void) {
    float size = CELL_SIZE * zoom;
    float ms = size * 0.5f;
    for (int i = 0; i < entranceCount; i++) {
        float px = offset.x + entrances[i].x * size + (size - ms) / 2;
        float py = offset.y + entrances[i].y * size + (size - ms) / 2;
        DrawRectangle((int)px, (int)py, (int)ms, (int)ms, WHITE);
    }
}

void DrawGraph(void) {
    if (!showGraph) return;
    float size = CELL_SIZE * zoom;
    for (int i = 0; i < graphEdgeCount; i += 2) {  // skip duplicates
        int e1 = graphEdges[i].from, e2 = graphEdges[i].to;
        Vector2 p1 = {offset.x + entrances[e1].x * size + size/2, offset.y + entrances[e1].y * size + size/2};
        Vector2 p2 = {offset.x + entrances[e2].x * size + size/2, offset.y + entrances[e2].y * size + size/2};
        DrawLineEx(p1, p2, 2.0f, YELLOW);
    }
}

void DrawPath(void) {
    float size = CELL_SIZE * zoom;
    if (startPos.x >= 0)
        DrawRectangle((int)(offset.x + startPos.x * size), (int)(offset.y + startPos.y * size), (int)size, (int)size, GREEN);
    if (goalPos.x >= 0)
        DrawRectangle((int)(offset.x + goalPos.x * size), (int)(offset.y + goalPos.y * size), (int)size, (int)size, RED);
    for (int i = 0; i < pathLength; i++) {
        float px = offset.x + path[i].x * size + size * 0.25f;
        float py = offset.y + path[i].y * size + size * 0.25f;
        DrawRectangle((int)px, (int)py, (int)(size * 0.5f), (int)(size * 0.5f), BLUE);
    }
}

Vector2 ScreenToGrid(Vector2 screen) {
    float size = CELL_SIZE * zoom;
    return (Vector2){(screen.x - offset.x) / size, (screen.y - offset.y) / size};
}

void HandleInput(void) {
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mw = ScreenToGrid(GetMousePosition());
        zoom += wheel * 0.1f;
        if (zoom < 0.2f) zoom = 0.2f;
        if (zoom > 3.0f) zoom = 3.0f;
        float size = CELL_SIZE * zoom;
        offset.x = GetMousePosition().x - mw.x * size;
        offset.y = GetMousePosition().y - mw.y * size;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 d = GetMouseDelta();
        offset.x += d.x;
        offset.y += d.y;
    }

    bool setStart = IsKeyDown(KEY_S);
    bool setGoal = IsKeyDown(KEY_G);

    if (!setStart && !setGoal && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT && grid[y][x] != CELL_WALL) {
            grid[y][x] = CELL_WALL;
            MarkChunkDirty(x, y);
        }
    }

    if (!setStart && !setGoal && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT && grid[y][x] != CELL_WALKABLE) {
            grid[y][x] = CELL_WALKABLE;
            MarkChunkDirty(x, y);
        }
    }

    if (setStart && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT && grid[y][x] == CELL_WALKABLE) {
            startPos = (Point){x, y};
            pathLength = 0;
        }
    }

    if (setGoal && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT && grid[y][x] == CELL_WALKABLE) {
            goalPos = (Point){x, y};
            pathLength = 0;
        }
    }

    if (IsKeyPressed(KEY_C)) InitGrid();
    if (IsKeyPressed(KEY_ONE)) GenerateSparse(0.15f);
    if (IsKeyPressed(KEY_TWO)) GenerateCity();
    if (IsKeyPressed(KEY_THREE)) GenerateMixed();
    if (IsKeyPressed(KEY_E)) BuildEntrances();
    if (IsKeyPressed(KEY_B)) BuildGraph();
    if (IsKeyPressed(KEY_P)) RunAStar();
    if (IsKeyPressed(KEY_V)) showGraph = !showGraph;
    if (IsKeyPressed(KEY_R)) {
        zoom = 1.0f;
        offset.x = (1280 - GRID_WIDTH * CELL_SIZE * zoom) / 2.0f;
        offset.y = (800 - GRID_HEIGHT * CELL_SIZE * zoom) / 2.0f;
    }
}

int main(void) {
    int screenWidth = 1280, screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "A* Pathfinding - Grid Editor");
    texGrass = LoadTexture("grass.png");
    texWall = LoadTexture("wall.png");
    SetTargetFPS(60);
    InitGrid();
    offset.x = (screenWidth - GRID_WIDTH * CELL_SIZE * zoom) / 2.0f;
    offset.y = (screenHeight - GRID_HEIGHT * CELL_SIZE * zoom) / 2.0f;

    while (!WindowShouldClose()) {
        HandleInput();
        BeginDrawing();
        ClearBackground(BLACK);
        DrawCellGrid();
        DrawChunkBoundaries();
        DrawGraph();
        DrawEntrances();
        DrawPath();
        DrawFPS(5, 5);
        DrawText(TextFormat("Entrances: %d | Edges: %d", entranceCount, graphEdgeCount), 5, 25, 16, WHITE);
        DrawText(TextFormat("Path: %d | Explored: %d | Time: %.2fms", pathLength, nodesExplored, lastPathTime), 5, 45, 16, WHITE);
        DrawText("S/G+Click | P: Path | 1: Forest | 2: City | 3: Mixed | E: Entrances | B: Graph | V: Show", 5, screenHeight - 20, 14, GRAY);
        EndDrawing();
    }
    UnloadTexture(texGrass);
    UnloadTexture(texWall);
    CloseWindow();
    return 0;
}
