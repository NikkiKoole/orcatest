// crowd_raylib.c
// Open world + boids-style separation (npc.gd-like) using a spatial hash grid (NOT O(N^2)).

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

#define AGENT_COUNT 5000

#define WORLD_W 4000.0f
#define WORLD_H 4000.0f

#define AGENT_SIZE   4.0f
#define AGENT_RADIUS (AGENT_SIZE * 0.5f)

#define ARRIVE_EPS    2.0f
#define WAIT_SECONDS  1.0f

// ---- npc.gd-ish tuning ----
#define AVOID_RADIUS         40.0f
#define AVOID_STRENGTH       60.0f
#define AVOID_MAX_NEIGHBORS  10

#define STUCK_THRESHOLD      1.0f
#define WIGGLE_STRENGTH      40.0f
#define PROGRESS_TOLERANCE   0.30f
#define STUCK_RECOVERY_RATE  0.50f

#define MAX_SPEED_MULTIPLIER 1.30f

// Spatial hash grid cell size
#define CELL_SIZE AVOID_RADIUS

typedef struct Agent {
    Vector2 pos;
    Vector2 vel;

    Vector2 goal;
    float speed;
    float wait;

    float stuck;
    float lastDist;
    Vector2 wiggleDir;
} Agent;

// ---------------- vector helpers ----------------
static inline Vector2 vadd(Vector2 a, Vector2 b) { return (Vector2){a.x+b.x, a.y+b.y}; }
static inline Vector2 vsub(Vector2 a, Vector2 b) { return (Vector2){a.x-b.x, a.y-b.y}; }
static inline Vector2 vmul(Vector2 a, float s)   { return (Vector2){a.x*s, a.y*s}; }
static inline float   vdot(Vector2 a, Vector2 b) { return a.x*b.x + a.y*b.y; }
static inline float   vlen2(Vector2 a)           { return vdot(a,a); }
static inline float   vlen(Vector2 a)            { return sqrtf(vlen2(a)); }

static inline Vector2 vnormalize(Vector2 a) {
    float l = vlen(a);
    if (l > 1e-6f) return vmul(a, 1.0f/l);
    return (Vector2){0,0};
}

static inline Vector2 vclamp_len(Vector2 v, float maxLen) {
    float l2 = vlen2(v);
    if (l2 > maxLen*maxLen) {
        float l = sqrtf(l2);
        return vmul(v, maxLen / l);
    }
    return v;
}

static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// ---------------- random helpers ----------------
static float Randf(float minv, float maxv) {
    int r = GetRandomValue(0, 10000);
    float t = (float)r / 10000.0f;
    return minv + (maxv - minv) * t;
}

static Vector2 RandGoal(void) {
    return (Vector2){ Randf(0.0f, WORLD_W), Randf(0.0f, WORLD_H) };
}

static Vector2 RandDir(void) {
    Vector2 d = (Vector2){ Randf(-1.0f, 1.0f), Randf(-1.0f, 1.0f) };
    if (vlen2(d) < 1e-6f) d = (Vector2){1, 0};
    return vnormalize(d);
}

// ---------------- spatial hash ----------------
// head[cell] = first agent index in cell, next[i] = next agent in same cell
static void BuildGrid(const Agent* agents, int agentCount,
                      int gridW, int gridH, int cellCount,
                      int* head, int* next)
{
    for (int c = 0; c < cellCount; c++) head[c] = -1;

    for (int i = 0; i < agentCount; i++) {
        int cx = (int)floorf(agents[i].pos.x / CELL_SIZE);
        int cy = (int)floorf(agents[i].pos.y / CELL_SIZE);
        cx = clampi(cx, 0, gridW - 1);
        cy = clampi(cy, 0, gridH - 1);
        int idx = cy * gridW + cx;

        next[i] = head[idx];
        head[idx] = i;
    }
}

// npc.gd-style separation: sum normalized(to_self)*((1 - dist/r)^2), cap neighbors=10
static Vector2 ComputeAvoidance(const Agent* agents, int self,
                                int gridW, int gridH,
                                const int* head, const int* next,
                                bool enabled)
{
    if (!enabled) return (Vector2){0,0};

    const Agent* a = &agents[self];
    Vector2 avoidance = (Vector2){0,0};

    const float r  = AVOID_RADIUS;
    const float r2 = r * r;

    int cx = (int)floorf(a->pos.x / CELL_SIZE);
    int cy = (int)floorf(a->pos.y / CELL_SIZE);
    cx = clampi(cx, 0, gridW - 1);
    cy = clampi(cy, 0, gridH - 1);

    int found = 0;

    for (int oy = -1; oy <= 1; oy++) {
        int ny = cy + oy;
        if (ny < 0 || ny >= gridH) continue;

        for (int ox = -1; ox <= 1; ox++) {
            int nx = cx + ox;
            if (nx < 0 || nx >= gridW) continue;

            int cell = ny * gridW + nx;
            for (int j = head[cell]; j != -1; j = next[j]) {
                if (j == self) continue;

                Vector2 toSelf = vsub(a->pos, agents[j].pos);
                float dsq = vlen2(toSelf);
                if (dsq <= 1e-10f || dsq >= r2) continue;

                float dist = sqrtf(dsq);

                float t = 1.0f - (dist / r);
                float strength = t * t;

                // normalized(toSelf) * strength
                avoidance = vadd(avoidance, vmul(toSelf, strength / dist));

                if (++found >= AVOID_MAX_NEIGHBORS) return avoidance;
            }
        }
    }

    return avoidance;
}

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;

    InitWindow(screenW, screenH, "Raylib crowd: boids separation + spatial hash");
    SetTargetFPS(60);

    srand((unsigned)time(NULL));

    Camera2D cam = {0};
    cam.offset = (Vector2){ screenW * 0.5f, screenH * 0.5f };
    cam.target = (Vector2){ WORLD_W * 0.5f, WORLD_H * 0.5f };
    cam.zoom = 1.0f;

    Agent* agents = (Agent*)MemAlloc((size_t)AGENT_COUNT * sizeof(Agent));

    for (int i = 0; i < AGENT_COUNT; i++) {
        agents[i].pos  = RandGoal();
        agents[i].goal = RandGoal();
        agents[i].speed = Randf(40.0f, 120.0f);
        agents[i].wait = 0.0f;

        agents[i].vel = (Vector2){0,0};
        agents[i].stuck = 0.0f;
        agents[i].wiggleDir = (Vector2){0,0};
        agents[i].lastDist = vlen(vsub(agents[i].goal, agents[i].pos));
    }

    // grid dims
    const int gridW = (int)ceilf(WORLD_W / CELL_SIZE);
    const int gridH = (int)ceilf(WORLD_H / CELL_SIZE);
    const int cellCount = gridW * gridH;

    int* head = (int*)MemAlloc((size_t)cellCount * sizeof(int));
    int* next = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));

    bool avoidanceOn = true;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        if (dt > 0.05f) dt = 0.05f; // avoid giant steps

        if (IsKeyPressed(KEY_V)) avoidanceOn = !avoidanceOn;

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

        cam.target.x = fminf(fmaxf(cam.target.x, 0.0f), WORLD_W);
        cam.target.y = fminf(fmaxf(cam.target.y, 0.0f), WORLD_H);

        // 1) Build grid from current positions
        BuildGrid(agents, AGENT_COUNT, gridW, gridH, cellCount, head, next);

        // 2) Update agents (avoidance queries the grid)
        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent* a = &agents[i];

            if (a->wait > 0.0f) {
                a->wait -= dt;
                if (a->wait <= 0.0f) {
                    a->goal = RandGoal();
                    a->wait = 0.0f;
                    a->stuck = 0.0f;
                    a->wiggleDir = (Vector2){0,0};
                    a->lastDist = vlen(vsub(a->goal, a->pos));
                }
                a->vel = (Vector2){0,0};
                continue;
            }

            Vector2 to = vsub(a->goal, a->pos);
            float dist = vlen(to);

            if (dist <= ARRIVE_EPS) {
                a->pos = a->goal;
                a->wait = WAIT_SECONDS;
                a->vel = (Vector2){0,0};
                a->stuck = 0.0f;
                a->wiggleDir = (Vector2){0,0};
                a->lastDist = 0.0f;
                continue;
            }

            Vector2 desiredDir = (dist > 1e-6f) ? vmul(to, 1.0f / dist) : (Vector2){0,0};

            // progress-based stuck detection
            float progress = a->lastDist - dist;
            float expected = a->speed * dt * PROGRESS_TOLERANCE;

            if (progress < expected) {
                a->stuck += dt;
            } else {
                a->stuck = fmaxf(0.0f, a->stuck - dt * STUCK_RECOVERY_RATE);
                if (a->stuck <= 0.0f) a->wiggleDir = (Vector2){0,0};
            }
            a->lastDist = dist;

            // when stuck: disable avoidance and add wiggle
            bool avoidEnabled = avoidanceOn && (a->stuck <= STUCK_THRESHOLD);
            Vector2 avoidance = ComputeAvoidance(agents, i, gridW, gridH, head, next, avoidEnabled);

            Vector2 wiggle = (Vector2){0,0};
            if (a->stuck > STUCK_THRESHOLD) {
                if (vlen2(a->wiggleDir) < 1e-6f) a->wiggleDir = RandDir();
                wiggle = vmul(a->wiggleDir, WIGGLE_STRENGTH);
            }

            Vector2 vel = (Vector2){0,0};
            vel = vadd(vel, vmul(desiredDir, a->speed));
            vel = vadd(vel, vmul(avoidance, AVOID_STRENGTH));
            vel = vadd(vel, wiggle);

            vel = vclamp_len(vel, a->speed * MAX_SPEED_MULTIPLIER);
            a->vel = vel;

            // integrate
            a->pos = vadd(a->pos, vmul(a->vel, dt));

            // keep inside world
            a->pos.x = fminf(fmaxf(a->pos.x, 0.0f), WORLD_W);
            a->pos.y = fminf(fmaxf(a->pos.y, 0.0f), WORLD_H);
        }

        // Draw
        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);
        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        for (int i = 0; i < AGENT_COUNT; i++) {
            const Agent* a = &agents[i];
            Color col = (Color){ 200, 220, 255, 255 };
            if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
            else if (a->stuck > STUCK_THRESHOLD) col = (Color){ 255, 90, 90, 255 };

            DrawCircleV(a->pos, AGENT_RADIUS, col);
        }
        EndMode2D();

        DrawText("WASD/Arrows: pan | Wheel: zoom | V: toggle avoidance", 12, 12, 18, RAYWHITE);
        DrawText(avoidanceOn ? "Avoidance: ON (grid)" : "Avoidance: OFF", 12, 36, 18, avoidanceOn ? GREEN : RED);
        DrawFPS(12, 60);

        EndDrawing();
    }

    MemFree(head);
    MemFree(next);
    MemFree(agents);

    CloseWindow();
    return 0;
}
