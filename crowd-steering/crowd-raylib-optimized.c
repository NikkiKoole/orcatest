// crowd_raylib.c
// Open world + boids-style separation using a FAST bucketed spatial grid (prefix sums).
// Press V to toggle avoidance. When OFF, we also SKIP grid building + stuck/wiggle logic
// so it matches the true "no avoidance" baseline workload.

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#define AGENT_COUNT 150000  // try 50000 later

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

// ---- perf knobs ----
#define CELL_SIZE        (AVOID_RADIUS * 0.5f) // smaller cells => fewer agents per cell
#define AVOID_MAX_SCAN   48                    // cap total candidates scanned
#define AVOID_PERIOD     3                     // compute avoidance every N frames per agent

typedef struct Agent {
    Vector2 pos;
    Vector2 vel;

    Vector2 goal;
    float speed;
    float wait;

    float stuck;
    float lastDist;
    Vector2 wiggleDir;

    Vector2 avoidCache; // cached avoidance for staggering
} Agent;

// ---------------- vector helpers ----------------
static inline Vector2 vadd(Vector2 a, Vector2 b) { return (Vector2){a.x+b.x, a.y+b.y}; }
static inline Vector2 vsub(Vector2 a, Vector2 b) { return (Vector2){a.x-b.x, a.y-b.y}; }
static inline Vector2 vmul(Vector2 a, float s)   { return (Vector2){a.x*s, a.y*s}; }
static inline float   vdot(Vector2 a, Vector2 b) { return a.x*b.x + a.y*b.y; }
static inline float   vlen2(Vector2 a)           { return vdot(a,a); }

static inline Vector2 vclamp_len(Vector2 v, float maxLen) {
    float l2 = vlen2(v);
    if (l2 > maxLen*maxLen) {
        float inv = 1.0f / sqrtf(l2);
        return vmul(v, maxLen * inv);
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
    float l2 = vlen2(d);
    if (l2 < 1e-12f) return (Vector2){1,0};
    float inv = 1.0f / sqrtf(l2);
    return vmul(d, inv);
}

// ---------------- bucket grid build ----------------
static void BuildGridBuckets(const Agent* agents, int N,
                             int gridW, int gridH, int cellCount,
                             float invCell,
                             int* cellCountArr, int* cellStart, int* cellWrite,
                             int* agentCell, int* cellAgents)
{
    for (int c = 0; c < cellCount; c++) cellCountArr[c] = 0;

    for (int i = 0; i < N; i++) {
        int cx = (int)(agents[i].pos.x * invCell);
        int cy = (int)(agents[i].pos.y * invCell);
        cx = clampi(cx, 0, gridW - 1);
        cy = clampi(cy, 0, gridH - 1);
        int idx = cy * gridW + cx;
        agentCell[i] = idx;
        cellCountArr[idx]++;
    }

    cellStart[0] = 0;
    for (int c = 0; c < cellCount; c++) {
        cellStart[c + 1] = cellStart[c] + cellCountArr[c];
    }

    for (int c = 0; c < cellCount; c++) cellWrite[c] = cellStart[c];

    for (int i = 0; i < N; i++) {
        int idx = agentCell[i];
        cellAgents[cellWrite[idx]++] = i;
    }
}

static Vector2 ComputeAvoidanceBucket(const Agent* agents, int self,
                                     int gridW, int gridH,
                                     const int* cellStart, const int* cellAgents,
                                     float invCell,
                                     int radCells,
                                     bool enabled)
{
    if (!enabled) return (Vector2){0,0};

    const Agent* a = &agents[self];
    Vector2 avoidance = (Vector2){0,0};

    const float r = AVOID_RADIUS;
    const float r2 = r * r;
    const float invR = 1.0f / r;

    int cx = (int)(a->pos.x * invCell);
    int cy = (int)(a->pos.y * invCell);
    cx = clampi(cx, 0, gridW - 1);
    cy = clampi(cy, 0, gridH - 1);

    int found = 0;
    int scanned = 0;

    for (int oy = -radCells; oy <= radCells; oy++) {
        int ny = cy + oy;
        if (ny < 0 || ny >= gridH) continue;

        for (int ox = -radCells; ox <= radCells; ox++) {
            int nx = cx + ox;
            if (nx < 0 || nx >= gridW) continue;

            int cell = ny * gridW + nx;
            int start = cellStart[cell];
            int end   = cellStart[cell + 1];

            for (int t = start; t < end; t++) {
                int j = cellAgents[t];
                if (j == self) continue;

                if (++scanned > AVOID_MAX_SCAN) return avoidance;

                Vector2 toSelf = vsub(a->pos, agents[j].pos);
                float dsq = vlen2(toSelf);
                if (dsq <= 1e-10f || dsq >= r2) continue;

                float invDist = 1.0f / sqrtf(dsq);
                float dist = dsq * invDist;

                float u = 1.0f - dist * invR;
                float strength = u * u;

                avoidance.x += toSelf.x * (strength * invDist);
                avoidance.y += toSelf.y * (strength * invDist);

                if (++found >= AVOID_MAX_NEIGHBORS) return avoidance;
            }
        }
    }

    return avoidance;
}

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;

    InitWindow(screenW, screenH, "Raylib crowd: separation + bucket grid (fast)");
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

        Vector2 to = vsub(agents[i].goal, agents[i].pos);
        agents[i].lastDist = sqrtf(vlen2(to));

        agents[i].avoidCache = (Vector2){0,0};
    }

    const float invCell = 1.0f / CELL_SIZE;
    const int gridW = (int)ceilf(WORLD_W / CELL_SIZE);
    const int gridH = (int)ceilf(WORLD_H / CELL_SIZE);
    const int cellCount = gridW * gridH;

    int* cellCountArr = (int*)MemAlloc((size_t)cellCount * sizeof(int));
    int* cellStart    = (int*)MemAlloc((size_t)(cellCount + 1) * sizeof(int));
    int* cellWrite    = (int*)MemAlloc((size_t)cellCount * sizeof(int));
    int* agentCell    = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));
    int* cellAgents   = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));

    int radCells = (int)ceilf(AVOID_RADIUS / CELL_SIZE);
    if (radCells < 1) radCells = 1;

    bool avoidanceOn = false;
    uint32_t frame = 0;

    const float arriveEps2 = ARRIVE_EPS * ARRIVE_EPS;

    while (!WindowShouldClose()) {
        frame++;

        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        if (dt > 0.05f) dt = 0.05f;

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
        // IMPORTANT: Only build grid if avoidance is ON
        if (avoidanceOn) {
            BuildGridBuckets(agents, AGENT_COUNT, gridW, gridH, cellCount,
                             invCell,
                             cellCountArr, cellStart, cellWrite,
                             agentCell, cellAgents);
        }

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
                    Vector2 to2 = vsub(a->goal, a->pos);
                    a->lastDist = sqrtf(vlen2(to2));
                }
                a->vel = (Vector2){0,0};
                a->avoidCache = (Vector2){0,0};
                continue;
            }

            // arrival check
            Vector2 to = vsub(a->goal, a->pos);
            float dist2 = vlen2(to);
            if (dist2 <= arriveEps2) {
                a->pos = a->goal;
                a->wait = WAIT_SECONDS;
                a->vel = (Vector2){0,0};
                a->stuck = 0.0f;
                a->wiggleDir = (Vector2){0,0};
                a->lastDist = 0.0f;
                a->avoidCache = (Vector2){0,0};
                continue;
            }

            // TRUE baseline path when avoidance is OFF:
            if (!avoidanceOn) {
                float invDist = 1.0f / sqrtf(dist2);
                float dist = dist2 * invDist;
                Vector2 dir = vmul(to, invDist);

                float step = a->speed * dt;
                if (step > dist) step = dist;

                a->pos = vadd(a->pos, vmul(dir, step));
                a->pos.x = fminf(fmaxf(a->pos.x, 0.0f), WORLD_W);
                a->pos.y = fminf(fmaxf(a->pos.y, 0.0f), WORLD_H);
                a->vel = (Vector2){0,0};
                continue;
            }

            // avoidance ON path:
            float invDist = 1.0f / sqrtf(dist2);
            float dist = dist2 * invDist;
            Vector2 desiredDir = vmul(to, invDist);

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

            bool allowAvoid = (a->stuck <= STUCK_THRESHOLD);

            Vector2 avoidance = (Vector2){0,0};
            if (allowAvoid) {
                if (((frame + (uint32_t)i) % AVOID_PERIOD) == 0) {
                    a->avoidCache = ComputeAvoidanceBucket(
                        agents, i, gridW, gridH, cellStart, cellAgents, invCell, radCells, true
                    );
                }
                avoidance = a->avoidCache;
            } else {
                a->avoidCache = (Vector2){0,0};
            }

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

            a->pos = vadd(a->pos, vmul(a->vel, dt));
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

             Vector2 p = agents[i].pos;
            Color col = (Color){ 200, 220, 255, 255 };
            if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
            else if (avoidanceOn && a->stuck > STUCK_THRESHOLD) col = (Color){ 255, 90, 90, 255 };

               Rectangle r = { p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f, AGENT_SIZE, AGENT_SIZE };
            DrawRectangleRec(r, (Color){ 200, 220, 255, 255 });
           // DrawCircleV(a->pos, AGENT_RADIUS, col);
        }
        EndMode2D();

        DrawText("WASD/Arrows: pan | Wheel: zoom | V: toggle avoidance", 12, 12, 18, RAYWHITE);
        DrawText(avoidanceOn ? "Avoidance: ON (bucket grid)" : "Avoidance: OFF (true baseline)",
                 12, 36, 18, avoidanceOn ? GREEN : RED);
        DrawFPS(12, 60);

        EndDrawing();
    }

    MemFree(cellCountArr);
    MemFree(cellStart);
    MemFree(cellWrite);
    MemFree(agentCell);
    MemFree(cellAgents);
    MemFree(agents);

    CloseWindow();
    return 0;
}
