// crowd_raylib.c
// Simple 2D top-down "open world" with 5000 agents.
// Each agent moves toward a random goal, waits 1s on arrival, then picks a new goal.

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

#define AGENT_COUNT 50000

#define WORLD_W 4000.0f
#define WORLD_H 4000.0f

#define AGENT_SIZE 4.0f
#define ARRIVE_EPS 2.0f
#define WAIT_SECONDS 1.0f

typedef struct Agent {
    Vector2 pos;
    Vector2 goal;
    float speed;     // units/sec
    float wait;      // seconds remaining
} Agent;

static float Randf(float minv, float maxv) {
    // GetRandomValue returns int; this keeps it simple and fast.
    int r = GetRandomValue(0, 10000);
    float t = (float)r / 10000.0f;
    return minv + (maxv - minv) * t;
}

static Vector2 RandGoal(void) {
    return (Vector2){ Randf(0.0f, WORLD_W), Randf(0.0f, WORLD_H) };
}

static float Len(Vector2 v) { return sqrtf(v.x*v.x + v.y*v.y); }

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;

    InitWindow(screenW, screenH, "Raylib crowd: 5000 agents (no obstacles)");
    SetTargetFPS(60);

    srand((unsigned)time(NULL));

    // Camera for panning/zooming around the world
    Camera2D cam = { 0 };
    cam.offset = (Vector2){ screenW * 0.5f, screenH * 0.5f };
    cam.target = (Vector2){ WORLD_W * 0.5f, WORLD_H * 0.5f };
    cam.zoom = 1.0f;

    Agent agents[AGENT_COUNT];

    for (int i = 0; i < AGENT_COUNT; i++) {
        agents[i].pos  = RandGoal();
        agents[i].goal = RandGoal();
        agents[i].speed = Randf(40.0f, 120.0f);
        agents[i].wait = 0.0f;
    }

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Camera controls
        float panSpeed = 800.0f / cam.zoom;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  cam.target.x -= panSpeed * dt;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) cam.target.x += panSpeed * dt;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    cam.target.y -= panSpeed * dt;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  cam.target.y += panSpeed * dt;

        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            cam.zoom *= (1.0f + wheel * 0.1f);
            if (cam.zoom < 0.1f) cam.zoom = 0.1f;
            if (cam.zoom > 6.0f) cam.zoom = 6.0f;
        }

        // Clamp camera target (optional)
        if (cam.target.x < 0) cam.target.x = 0;
        if (cam.target.y < 0) cam.target.y = 0;
        if (cam.target.x > WORLD_W) cam.target.x = WORLD_W;
        if (cam.target.y > WORLD_H) cam.target.y = WORLD_H;

        // Update agents
        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent *a = &agents[i];

            if (a->wait > 0.0f) {
                a->wait -= dt;
                if (a->wait <= 0.0f) {
                    a->goal = RandGoal();
                    a->wait = 0.0f;
                }
                continue;
            }

            Vector2 to = (Vector2){ a->goal.x - a->pos.x, a->goal.y - a->pos.y };
            float d = Len(to);

            if (d <= ARRIVE_EPS) {
                a->pos = a->goal;
                a->wait = WAIT_SECONDS;
                continue;
            }

            // Move toward goal
            Vector2 dir = (Vector2){ to.x / d, to.y / d };
            float step = a->speed * dt;
            if (step > d) step = d;

            a->pos.x += dir.x * step;
            a->pos.y += dir.y * step;
        }

        // Draw
        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);

        // World bounds
        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        // Agents (simple moving rectangles)
        for (int i = 0; i < AGENT_COUNT; i++) {
            Vector2 p = agents[i].pos;
            // center rectangle on agent pos
            Rectangle r = { p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f, AGENT_SIZE, AGENT_SIZE };
            //DrawRectangleRec(r, (Color){ 200, 220, 255, 255 });
               DrawCircleV(p, AGENT_SIZE, (Color){ 200, 220, 255, 255 });
        }

        EndMode2D();

        DrawText("WASD/Arrows: pan | Mouse wheel: zoom | 5000 agents", 12, 12, 18, RAYWHITE);
        DrawFPS(12, 36);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
