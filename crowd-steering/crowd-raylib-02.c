// crowd_raylib.c
// Simple 2D top-down "open world" with boids-style separation (from npc.gd).
// Each agent moves toward a random goal, waits on arrival, then picks a new goal.
//
// Added from npc.gd:
//  - avoidance_radius, avoidance_strength
//  - repulsion falloff: (1 - dist/r)^2
//  - cap to max neighbors (10)
//  - stuck detection via "progress toward goal"
//  - when stuck: disable avoidance + apply wiggle force
//  - clamp to max_speed = speed * MAX_SPEED_MULTIPLIER
//
// Controls:
//  - WASD/Arrows: pan
//  - Mouse wheel: zoom
//  - V: toggle avoidance

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

// ---- "npc.gd" avoidance/stuck tuning ----
#define AVOID_RADIUS         40.0f   // avoidance_radius
#define AVOID_STRENGTH       60.0f   // avoidance_strength (scaled by falloff)
#define AVOID_MAX_NEIGHBORS  10      // intersect_shape(..., 10)

#define STUCK_THRESHOLD      1.0f    // stuck_threshold seconds
#define WIGGLE_STRENGTH      40.0f   // wiggle_strength

#define PROGRESS_TOLERANCE   0.3f    // Need 30% of expected speed to count as progress
#define STUCK_RECOVERY_RATE  0.5f    // stuck timer decreases when moving

#define MAX_SPEED_MULTIPLIER 1.3f    // allow slight overspeed while avoiding

// Spatial hash
#define CELL_SIZE AVOID_RADIUS

typedef struct Agent {
    Vector2 pos;
    Vector2 vel;

    Vector2 goal;
    float speed;     // "desired speed"
    float wait;      // seconds remaining

    // stuck logic (npc.gd-inspired)
    float stuck;     // seconds "stuck"
    float lastDist;  // last distance to goal
    Vector2 wiggleDir;
} Agent;

// ----------------- small helpers -----------------
static inline Vector2 vadd(Vector2 a, Vector2 b) { return (Vector2){a.x + b.x, a.y + b.y}; }
static inline Vector2 vsub(Vector2 a, Vector2 b) { return (Vector2){a.x - b.x, a.y - b.y}; }
static inline Vector2 vmul(Vector2 a, float s) { return (Vector2){a.x * s, a.y * s}; }

static inline float vdot(Vector2 a, Vector2 b) { return a.x*b.x + a.y*b.y; }
static inline float vlen2(Vector2 a) { return vdot(a, a); }
static inline float vlen(Vector2 a) { return sqrtf(vlen2(a)); }

static inline Vector2 vnormalize(Vector2 a) {
    float l = vlen(a);
    if (l > 1e-6f) return vmul(a, 1.0f / l);
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

static float Randf(float minv, float maxv) {
    int r = GetRandomValue(0, 10000);
    float t = (float)r / 10000.0f;
    return minv + (maxv - minv) * t;
}

static Vector2 RandGoal(void) {
    return (Vector2){ Randf(0.0f, WORLD_W), Randf(0.0f, WORLD_H) };
}

static Vector2 RandDir(void) {
    // random direction in [-1,1] range, normalized
    Vector2 d = (Vector2){ Randf(-1.0f, 1.0f), Randf(-1.0f, 1.0f) };
    if (vlen2(d) < 1e-6f) d = (Vector2){1, 0};
    return vnormalize(d);
}

// ----------------- avoidance computation (npc.gd style) -----------------
static Vector2 ComputeAvoidance(const Agent* agents, int self,
                                const int* head, const int* next,
                                int gridW, int gridH,
                                bool enabled)
{
    if (!enabled) return (Vector2){0,0};

    const Agent* a = &agents[self];
    Vector2 avoidance = (Vector2){0,0};

    const float r = AVOID_RADIUS;
    const float r2 = r * r;

    // gather up to 10 neighbors from nearby cells
    int found = 0;

    int cx = (int)floorf(a->pos.x / CELL_SIZE);
    int cy = (int)floorf(a->pos.y / CELL_SIZE);
    cx = clampi(cx, 0, gridW - 1);
    cy = clampi(cy, 0, gridH - 1);

    for (int oy = -1; oy <= 1; oy++) {
        int ny = cy + oy;
        if (ny < 0 || ny >= gridH) continue;

        for (int ox = -1; ox <= 1; ox++) {
            int nx = cx + ox;
            if (nx < 0 || nx >= gridW) continue;

            int idx = ny * gridW + nx;
            for (int j = head[idx]; j != -1; j = next[j]) {
                if (j == self) continue;

                Vector2 toSelf = vsub(a->pos, agents[j].pos);
                float dsq = vlen2(toSelf);
                if (dsq <= 1e-10f || dsq >= r2) continue;

                float dist = sqrtf(dsq);

                // npc.gd: strength = pow(1 - dist/r, 2)
                float t = 1.0f - (dist / r);
                float strength = t * t;

                avoidance = vadd(avoidance, vmul(toSelf, strength / dist)); // normalized(toSelf)*strength

                if (++found >= AVOID_MAX_NEIGHBORS) return avoidance;
            }
        }
    }

    return avoidance;
}

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;

    InitWindow(screenW, screenH, "Raylib crowd: boids separation (npc.gd-style)");
    SetTargetFPS(60);

    srand((unsigned)time(NULL));

    Camera2D cam = { 0 };
    cam.offset = (Vector2){ screenW * 0.5f, screenH * 0.5f };
    cam.target = (Vector2){ WORLD_W * 0.5f, WORLD_H * 0.5f };
    cam.zoom = 1.0f;

    // Allocate big arrays on heap (safer for 50k)
    Agent* agents = (Agent*)MemAlloc((size_t)AGENT_COUNT * sizeof(Agent));

    for (int i = 0; i < AGENT_COUNT; i++) {
        agents[i].pos  = RandGoal();
        agents[i].goal = RandGoal();
        agents[i].speed = Randf(40.0f, 120.0f);
        agents[i].wait = 0.0f;

        agents[i].vel = (Vector2){0,0};
        agents[i].stuck = 0.0f;
        agents[i].wiggleDir = (Vector2){0,0};

        // initialize lastDist to current goal distance
        agents[i].lastDist = vlen(vsub(agents[i].goal, agents[i].pos));
    }

    // Spatial hash arrays
    const int gridW = (int)ceilf(WORLD_W / CELL_SIZE);
    const int gridH = (int)ceilf(WORLD_H / CELL_SIZE);
    const int cellCount = gridW * gridH;

    int* head = (int*)MemAlloc((size_t)cellCount * sizeof(int));
    int* next = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));

    bool avoidanceOn = true;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        if (dt > 0.05f) dt = 0.05f; // avoid huge jumps when window loses focus

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

        // Build spatial hash lists
        for (int c = 0; c < cellCount; c++) head[c] = -1;

        for (int i = 0; i < AGENT_COUNT; i++) {
            int cx = (int)floorf(agents[i].pos.x / CELL_SIZE);
            int cy = (int)floorf(agents[i].pos.y / CELL_SIZE);
            cx = clampi(cx, 0, gridW - 1);
            cy = clampi(cy, 0, gridH - 1);
            int idx = cy * gridW + cx;
            next[i] = head[idx];
            head[idx] = i;
        }

        // Update agents
        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent* a = &agents[i];

            // waiting
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

            // arrival
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

            // npc.gd: avoidance disabled when stuck to allow pushing through
            bool avoidEnabled = avoidanceOn && (a->stuck <= STUCK_THRESHOLD);
            Vector2 avoidance = ComputeAvoidance(agents, i, head, next, gridW, gridH, avoidEnabled);

            // npc.gd stuck detection: progress toward goal
            float progress = a->lastDist - dist;
            float expected = a->speed * dt * PROGRESS_TOLERANCE;

            if (progress < expected) {
                a->stuck += dt;
            } else {
                if (progress > expected * 2.0f) {
                    a->stuck = 0.0f;
                    a->wiggleDir = (Vector2){0,0};
                } else {
                    a->stuck = fmaxf(0.0f, a->stuck - dt * STUCK_RECOVERY_RATE);
                }
            }
            a->lastDist = dist;

            // npc.gd wiggle when stuck
            Vector2 wiggle = (Vector2){0,0};
            if (a->stuck > STUCK_THRESHOLD) {
                if (vlen2(a->wiggleDir) < 1e-6f) {
                    a->wiggleDir = RandDir();
                }
                wiggle = vmul(a->wiggleDir, WIGGLE_STRENGTH);
            }

            // Combine: desired movement + avoidance + wiggle
            Vector2 finalVel = (Vector2){0,0};
            finalVel = vadd(finalVel, vmul(desiredDir, a->speed));
            finalVel = vadd(finalVel, vmul(avoidance, AVOID_STRENGTH));
            finalVel = vadd(finalVel, wiggle);

            // Clamp to slightly higher than base speed while avoiding (npc.gd style)
            float maxSpeed = a->speed * MAX_SPEED_MULTIPLIER;
            finalVel = vclamp_len(finalVel, maxSpeed);

            a->vel = finalVel;

            // Integrate
            a->pos = vadd(a->pos, vmul(a->vel, dt));

            // Keep inside world
            a->pos.x = fminf(fmaxf(a->pos.x, 0.0f), WORLD_W);
            a->pos.y = fminf(fmaxf(a->pos.y, 0.0f), WORLD_H);
        }

        // Draw
        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);

        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        // Draw agents as circles (smooth)
        for (int i = 0; i < AGENT_COUNT; i++) {
            const Agent* a = &agents[i];

            Color col = (Color){ 200, 220, 255, 255 };
            if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
            else if (a->stuck > STUCK_THRESHOLD) col = (Color){ 255, 90, 90, 255 };

            DrawCircleV(a->pos, AGENT_RADIUS, col);
        }

        EndMode2D();

        DrawText("WASD/Arrows: pan | Mouse wheel: zoom | V: toggle avoidance", 12, 12, 18, RAYWHITE);
        DrawText(avoidanceOn ? "Avoidance: ON (boids separation)" : "Avoidance: OFF", 12, 36, 18, avoidanceOn ? GREEN : RED);
        DrawFPS(12, 60);

        EndDrawing();
    }

    MemFree(head);
    MemFree(next);
    MemFree(agents);

    CloseWindow();
    return 0;
}
