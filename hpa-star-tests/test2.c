#include "raylib.h"

#define GRID_WIDTH  128*4
#define GRID_HEIGHT 128*4
#define CELL_SIZE   32
#define CHUNK_SIZE  32

typedef enum {
    CELL_WALKABLE,
    CELL_WALL
} CellType;

CellType grid[GRID_HEIGHT][GRID_WIDTH];
float zoom = 1.0f;
Vector2 offset = {0, 0};
Texture2D texGrass;
Texture2D texWall;

void InitGrid(void) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            grid[y][x] = CELL_WALKABLE;
        }
    }
}

void GenerateSparse(float density) {
    InitGrid();
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if ((float)GetRandomValue(0, 100) / 100.0f < density) {
                grid[y][x] = CELL_WALL;
            }
        }
    }
}

void GenerateRooms(int count) {
    InitGrid();
    for (int i = 0; i < count; i++) {
        int x = GetRandomValue(0, GRID_WIDTH - 1);
        int y = GetRandomValue(0, GRID_HEIGHT - 1);
        int len = GetRandomValue(5, 20);
        int horizontal = GetRandomValue(0, 1);
        int gap = GetRandomValue(2, len - 2);

        for (int j = 0; j < len; j++) {
            if (j == gap || j == gap + 1) continue;  // leave a gap
            int wx = horizontal ? x + j : x;
            int wy = horizontal ? y : y + j;
            if (wx >= 0 && wx < GRID_WIDTH && wy >= 0 && wy < GRID_HEIGHT) {
                grid[wy][wx] = CELL_WALL;
            }
        }
    }
}

void DrawCellGrid(void) {
    Rectangle src = { 0, 0, 16, 16 };
    float size = CELL_SIZE * zoom;

    // Pass 1: Draw all grass
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (grid[y][x] == CELL_WALKABLE) {
                Rectangle dest = { offset.x + x * size, offset.y + y * size, size, size };
                DrawTexturePro(texGrass, src, dest, (Vector2){0,0}, 0, WHITE);
            }
        }
    }

    // Pass 2: Draw all walls
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (grid[y][x] == CELL_WALL) {
                Rectangle dest = { offset.x + x * size, offset.y + y * size, size, size };
                DrawTexturePro(texWall, src, dest, (Vector2){0,0}, 0, WHITE);
            }
        }
    }
}

void DrawChunkBoundaries(void) {
    float cellSize = CELL_SIZE * zoom;
    float chunkPixels = CHUNK_SIZE * cellSize;
    int chunksX = GRID_WIDTH / CHUNK_SIZE;
    int chunksY = GRID_HEIGHT / CHUNK_SIZE;

    for (int cy = 0; cy <= chunksY; cy++) {
        Vector2 start = { offset.x, offset.y + cy * chunkPixels };
        Vector2 end = { offset.x + chunksX * chunkPixels, offset.y + cy * chunkPixels };
        DrawLineEx(start, end, 3.0f, RED);
    }

    for (int cx = 0; cx <= chunksX; cx++) {
        Vector2 start = { offset.x + cx * chunkPixels, offset.y };
        Vector2 end = { offset.x + cx * chunkPixels, offset.y + chunksY * chunkPixels };
        DrawLineEx(start, end, 3.0f, RED);
    }
}

Vector2 ScreenToGrid(Vector2 screen) {
    float size = CELL_SIZE * zoom;
    return (Vector2){
        (screen.x - offset.x) / size,
        (screen.y - offset.y) / size
    };
}

void HandleInput(void) {
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mouseWorld = ScreenToGrid(GetMousePosition());

        zoom += wheel * 0.1f;
        if (zoom < 0.2f) zoom = 0.2f;
        if (zoom > 3.0f) zoom = 3.0f;

        float size = CELL_SIZE * zoom;
        offset.x = GetMousePosition().x - mouseWorld.x * size;
        offset.y = GetMousePosition().y - mouseWorld.y * size;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 delta = GetMouseDelta();
        offset.x += delta.x;
        offset.y += delta.y;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x;
        int y = (int)gp.y;

        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            grid[y][x] = CELL_WALL;
        }
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x;
        int y = (int)gp.y;

        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            grid[y][x] = CELL_WALKABLE;
        }
    }

    if (IsKeyPressed(KEY_C)) {
        InitGrid();
    }

    if (IsKeyPressed(KEY_ONE)) {
        GenerateSparse(0.15f);
    }

    if (IsKeyPressed(KEY_TWO)) {
        GenerateRooms(200);
    }

    if (IsKeyPressed(KEY_R)) {
        zoom = 1.0f;
        offset.x = (1280 - GRID_WIDTH * CELL_SIZE * zoom) / 2.0f;
        offset.y = (800 - GRID_HEIGHT * CELL_SIZE * zoom) / 2.0f;
    }
}

int main(void) {
    int screenWidth = 1280;
    int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "A* Pathfinding - Grid Editor");
    texGrass = LoadTexture("grass.png");
    texWall = LoadTexture("wall.png");
    SetTargetFPS(60);

    InitGrid();

    // Center camera
    offset.x = (screenWidth - GRID_WIDTH * CELL_SIZE * zoom) / 2.0f;
    offset.y = (screenHeight - GRID_HEIGHT * CELL_SIZE * zoom) / 2.0f;

    while (!WindowShouldClose()) {
        HandleInput();

        BeginDrawing();
        ClearBackground(BLACK);
        DrawCellGrid();
        DrawChunkBoundaries();
        DrawFPS(5, 5);

        DrawText("LMB: Wall | RMB: Clear | MMB: Pan | Scroll: Zoom | C: Clear | R: Reset View | 1: Sparse | 2: Rooms",
                 5, screenHeight - 20, 14, GRAY);
        EndDrawing();
    }

    UnloadTexture(texGrass);
    UnloadTexture(texWall);
    CloseWindow();
    return 0;
}
