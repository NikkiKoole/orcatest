#include "raylib.h"
#include "grid.h"
#include "terrain.h"
#include "pathfinding.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define CELL_SIZE   32

float zoom = 1.0f;
Vector2 offset = {0, 0};
Texture2D texGrass;
Texture2D texWall;
bool showGraph = false;

// Pathfinding algorithm selection: 0=A*, 1=HPA*, 2=JPS
int pathAlgorithm = 0;
const char* pathAlgorithmNames[] = {"A*", "HPA*", "JPS"};

void DrawCellGrid(void) {
    Rectangle src = {0, 0, 16, 16};
    float size = CELL_SIZE * zoom;
    for (int y = 0; y < gridHeight; y++)
        for (int x = 0; x < gridWidth; x++)
            if (grid[y][x] == CELL_WALKABLE) {
                Rectangle dest = {offset.x + x * size, offset.y + y * size, size, size};
                DrawTexturePro(texGrass, src, dest, (Vector2){0,0}, 0, WHITE);
            }
    for (int y = 0; y < gridHeight; y++)
        for (int x = 0; x < gridWidth; x++)
            if (grid[y][x] == CELL_WALL) {
                Rectangle dest = {offset.x + x * size, offset.y + y * size, size, size};
                DrawTexturePro(texWall, src, dest, (Vector2){0,0}, 0, WHITE);
            }
}

void DrawChunkBoundaries(void) {
    float cellSize = CELL_SIZE * zoom;
    float chunkPixelsX = chunkWidth * cellSize;
    float chunkPixelsY = chunkHeight * cellSize;
    for (int cy = 0; cy <= chunksY; cy++) {
        Vector2 s = {offset.x, offset.y + cy * chunkPixelsY};
        Vector2 e = {offset.x + chunksX * chunkPixelsX, offset.y + cy * chunkPixelsY};
        DrawLineEx(s, e, 3.0f, RED);
    }
    for (int cx = 0; cx <= chunksX; cx++) {
        Vector2 s = {offset.x + cx * chunkPixelsX, offset.y};
        Vector2 e = {offset.x + cx * chunkPixelsX, offset.y + chunksY * chunkPixelsY};
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
    for (int i = 0; i < graphEdgeCount; i += 2) {
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
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && grid[y][x] != CELL_WALL) {
            grid[y][x] = CELL_WALL;
            MarkChunkDirty(x, y);
        }
    }
    if (!setStart && !setGoal && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && grid[y][x] != CELL_WALKABLE) {
            grid[y][x] = CELL_WALKABLE;
            MarkChunkDirty(x, y);
        }
    }
    if (setStart && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && grid[y][x] == CELL_WALKABLE) {
            startPos = (Point){x, y};
            pathLength = 0;
        }
    }
    if (setGoal && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x, y = (int)gp.y;
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && grid[y][x] == CELL_WALKABLE) {
            goalPos = (Point){x, y};
            pathLength = 0;
        }
    }
    if (IsKeyPressed(KEY_C)) InitGrid();
    if (IsKeyPressed(KEY_ONE)) GenerateSparse(0.15f);
    if (IsKeyPressed(KEY_TWO)) GenerateCity();
    if (IsKeyPressed(KEY_THREE)) GenerateMixed();
    if (IsKeyPressed(KEY_FOUR)) GeneratePerlin();
    if (IsKeyPressed(KEY_E)) BuildEntrances();
    if (IsKeyPressed(KEY_B)) BuildGraph();
    if (IsKeyPressed(KEY_U)) UpdateDirtyChunks();  // Incremental update
    if (IsKeyPressed(KEY_P)) {
        // Auto-update dirty chunks before HPA* pathfinding
        if (pathAlgorithm == 1 && needsRebuild) {
            UpdateDirtyChunks();
        }
        switch (pathAlgorithm) {
            case 0: RunAStar(); break;
            case 1: RunHPAStar(); break;
            case 2: RunJPS(); break;
        }
    }
    if (IsKeyPressed(KEY_T)) pathAlgorithm = (pathAlgorithm + 1) % 3;  // Cycle through algorithms
    if (IsKeyPressed(KEY_D)) use8Dir = !use8Dir;  // Toggle 4-dir/8-dir
    if (IsKeyPressed(KEY_V)) showGraph = !showGraph;
    if (IsKeyPressed(KEY_R)) {
        zoom = 1.0f;
        offset.x = (1280 - gridWidth * CELL_SIZE * zoom) / 2.0f;
        offset.y = (800 - gridHeight * CELL_SIZE * zoom) / 2.0f;
    }
}

int main(void) {
    int screenWidth = 1280, screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "HPA* Pathfinding");
    texGrass = LoadTexture("grass.png");
    texWall = LoadTexture("wall.png");
    SetTargetFPS(60);
    InitGrid();
    offset.x = (screenWidth - gridWidth * CELL_SIZE * zoom) / 2.0f;
    offset.y = (screenHeight - gridHeight * CELL_SIZE * zoom) / 2.0f;

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
        DrawText(TextFormat("Algo: %s | Dir: %s | Entrances: %d | Edges: %d", 
                 pathAlgorithmNames[pathAlgorithm], use8Dir ? "8-dir" : "4-dir", entranceCount, graphEdgeCount), 5, 25, 16, WHITE);
        if (pathAlgorithm == 1 && hpaAbstractTime > 0) {
            DrawText(TextFormat("Path: %d | Explored: %d | Time: %.2fms (abstract: %.2fms, refine: %.2fms)", 
                     pathLength, nodesExplored, lastPathTime, hpaAbstractTime, hpaRefinementTime), 5, 45, 16, WHITE);
        } else {
            DrawText(TextFormat("Path: %d | Explored: %d | Time: %.2fms", pathLength, nodesExplored, lastPathTime), 5, 45, 16, WHITE);
        }
        DrawText("S/G+Click | P: Path | T: Algo | D: Dir | 1-4: Gen | E: Entrances | B: Graph | U: Update", 5, screenHeight - 20, 14, GRAY);
        EndDrawing();
    }
    UnloadTexture(texGrass);
    UnloadTexture(texWall);
    CloseWindow();
    return 0;
}
