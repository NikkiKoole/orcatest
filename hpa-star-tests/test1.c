#include "raylib.h"

#define GRID_WIDTH  128
#define GRID_HEIGHT 128
#define CELL_SIZE   32

typedef enum {
    CELL_WALKABLE,
    CELL_WALL
} CellType;

CellType grid[GRID_HEIGHT][GRID_WIDTH];
float zoom = 1.0f;
Vector2 offset = {0, 0};

void InitGrid(void) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            grid[y][x] = CELL_WALKABLE;
        }
    }
}

void DrawCellGrid(void) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            float size = CELL_SIZE * zoom;
            Rectangle cell = {
                offset.x + x * size,
                offset.y + y * size,
                size,
                size
            };

            Color color = (grid[y][x] == CELL_WALL) ? DARKGRAY : RAYWHITE;
            DrawRectangleRec(cell, color);
            DrawRectangleLinesEx(cell, 1, LIGHTGRAY);
        }
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
    // Zoom with mouse wheel
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mouseWorld = ScreenToGrid(GetMousePosition());

        zoom += wheel * 0.1f;
        if (zoom < 0.2f) zoom = 0.2f;
        if (zoom > 3.0f) zoom = 3.0f;

        // Adjust offset to zoom toward mouse position
        float size = CELL_SIZE * zoom;
        offset.x = GetMousePosition().x - mouseWorld.x * size;
        offset.y = GetMousePosition().y - mouseWorld.y * size;
    }

    // Pan with middle mouse button
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 delta = GetMouseDelta();
        offset.x += delta.x;
        offset.y += delta.y;
    }

    // Draw walls
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x;
        int y = (int)gp.y;

        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            grid[y][x] = CELL_WALL;
        }
    }

    // Erase walls
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 gp = ScreenToGrid(GetMousePosition());
        int x = (int)gp.x;
        int y = (int)gp.y;

        if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            grid[y][x] = CELL_WALKABLE;
        }
    }

    // Reset
    if (IsKeyPressed(KEY_C)) {
        InitGrid();
    }

    // Reset view
    if (IsKeyPressed(KEY_R)) {
        zoom = 1.0f;
        offset = (Vector2){0, 0};
    }
}

int main(void) {
    int screenWidth = 1280;
    int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "A* Pathfinding - Grid Editor");
    SetTargetFPS(60);;

    InitGrid();

    while (!WindowShouldClose()) {
        HandleInput();

        BeginDrawing();
        ClearBackground(BLACK);
        DrawCellGrid();
        DrawFPS(5, 5);

        DrawText("LMB: Wall | RMB: Clear | MMB: Pan | Scroll: Zoom | C: Clear | R: Reset View",
                 5, screenHeight - 20, 14, GRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
