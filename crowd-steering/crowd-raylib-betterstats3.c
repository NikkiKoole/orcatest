// crowd_raylib.c
// Open world + boids-style separation using a FAST bucketed spatial grid (prefix sums).
// Press V to toggle avoidance. When OFF, we also SKIP grid building + stuck/wiggle logic
// so it matches the true "no avoidance" baseline workload.
//
// Improved profiler:
//  - Measures FULL frame time including EndDrawing() (swap/present).
//  - Shows swap time explicitly.
//  - Shows max scanned/found and cap-hit counters to verify caps are active.
//  - Optionally culls drawing to camera view (C key).
//
// CACHE OPTIMIZATIONS:
//  - Agents sorted by grid cell for spatial coherency
//  - Avoidance always enabled (no death spiral)

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define AGENT_COUNT 300000  // try 50000 later

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

#define STUCK_THRESHOLD      1.5f    // Increased from 1.0
#define WIGGLE_STRENGTH      40.0f
#define PROGRESS_TOLERANCE   0.15f   // Relaxed from 0.30
#define STUCK_RECOVERY_RATE  1.0f    // Increased from 0.50

#define MAX_SPEED_MULTIPLIER 1.30f

// ---- perf knobs ----
#define CELL_SIZE        (AVOID_RADIUS * 0.5f) // smaller cells => fewer agents per cell
#define AVOID_MAX_SCAN   48                    // cap total candidates scanned
#define AVOID_PERIOD     3                     // compute avoidance every N frames per agent

// ---- profiler knobs ----
#define PROF_SMOOTH_ALPHA 0.10 // EMA smoothing for ms numbers (0.05..0.20)

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
    // Count agents per cell
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

    // Prefix sum into start offsets
    cellStart[0] = 0;
    for (int c = 0; c < cellCount; c++) {
        cellStart[c + 1] = cellStart[c] + cellCountArr[c];
    }

    // Copy starts into write cursors
    for (int c = 0; c < cellCount; c++) cellWrite[c] = cellStart[c];

    // Scatter agent indices into the big contiguous list
    for (int i = 0; i < N; i++) {
        int idx = agentCell[i];
        cellAgents[cellWrite[idx]++] = i;
    }
}

// ---------------- CACHE OPTIMIZATION: Sort agents by grid cell ----------------
static void SortAgentsByCell(Agent* agents, int N,
                             int gridW, int gridH, int cellCount,
                             int* agentCell, int* cellAgents,
                             int* cellStart)
{
    // Allocate temporary storage
    Agent* tempAgents = (Agent*)MemAlloc((size_t)N * sizeof(Agent));
    int* oldToNew = (int*)MemAlloc((size_t)N * sizeof(int));

    // Reorder agents by cell, so agents in same cell are contiguous
    int writePos = 0;
    for (int cell = 0; cell < cellCount; cell++) {
        int start = cellStart[cell];
        int end = cellStart[cell + 1];

        for (int t = start; t < end; t++) {
            int oldIdx = cellAgents[t];
            tempAgents[writePos] = agents[oldIdx];
            oldToNew[oldIdx] = writePos;
            writePos++;
        }
    }

    // Copy sorted agents back
    for (int i = 0; i < N; i++) {
        agents[i] = tempAgents[i];
    }

    // Update cellAgents with new indices
    for (int cell = 0; cell < cellCount; cell++) {
        int start = cellStart[cell];
        int end = cellStart[cell + 1];

        for (int t = start; t < end; t++) {
            int oldIdx = cellAgents[t];
            cellAgents[t] = oldToNew[oldIdx];
        }
    }

    // Update agentCell array
    for (int i = 0; i < N; i++) {
        int cx = (int)(agents[i].pos.x * CELL_SIZE);
        int cy = (int)(agents[i].pos.y * CELL_SIZE);
        cx = clampi(cx, 0, gridW - 1);
        cy = clampi(cy, 0, gridH - 1);
        agentCell[i] = cy * gridW + cx;
    }

    MemFree(tempAgents);
    MemFree(oldToNew);
}

static Vector2 ComputeAvoidanceBucket(const Agent* agents, int self,
                                     int gridW, int gridH,
                                     const int* cellStart, const int* cellAgents,
                                     float invCell,
                                     int radCells,
                                     bool enabled,
                                     int* outScanned,
                                     int* outFound,
                                     bool* outHitScanCap,
                                     bool* outHitNeighborCap)
{
    if (!enabled) {
        if (outScanned) *outScanned = 0;
        if (outFound) *outFound = 0;
        if (outHitScanCap) *outHitScanCap = false;
        if (outHitNeighborCap) *outHitNeighborCap = false;
        return (Vector2){0,0};
    }

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
    bool hitScanCap = false;
    bool hitNeighborCap = false;

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

                // Hard cap on raw candidates scanned (regardless of distance)
                scanned++;
                if (scanned >= AVOID_MAX_SCAN) {
                    hitScanCap = true;
                    if (outScanned) *outScanned = scanned;
                    if (outFound) *outFound = found;
                    if (outHitScanCap) *outHitScanCap = hitScanCap;
                    if (outHitNeighborCap) *outHitNeighborCap = hitNeighborCap;
                    return avoidance;
                }

                Vector2 toSelf = vsub(a->pos, agents[j].pos);
                float dsq = vlen2(toSelf);
                if (dsq <= 1e-10f || dsq >= r2) continue;

                // normalized(toSelf) * ((1 - dist/r)^2)
                float invDist = 1.0f / sqrtf(dsq);
                float dist = dsq * invDist;

                float u = 1.0f - dist * invR;
                float strength = u * u;

                float k = strength * invDist;
                avoidance.x += toSelf.x * k;
                avoidance.y += toSelf.y * k;

                found++;
                if (found >= AVOID_MAX_NEIGHBORS) {
                    hitNeighborCap = true;
                    if (outScanned) *outScanned = scanned;
                    if (outFound) *outFound = found;
                    if (outHitScanCap) *outHitScanCap = hitScanCap;
                    if (outHitNeighborCap) *outHitNeighborCap = hitNeighborCap;
                    return avoidance;
                }
            }
        }
    }

    if (outScanned) *outScanned = scanned;
    if (outFound) *outFound = found;
    if (outHitScanCap) *outHitScanCap = hitScanCap;
    if (outHitNeighborCap) *outHitNeighborCap = hitNeighborCap;
    return avoidance;
}

// ---------- tiny profiler helpers ----------
static inline double Ms(double seconds) { return seconds * 1000.0; }
static inline double EMA(double prev, double cur, double alpha) {
    return (prev == 0.0) ? cur : (prev + alpha * (cur - prev));
}
static void DrawTextShadow(const char *text, int x, int y, int size, Color col) {
    DrawText(text, x + 1, y + 1, size, BLACK); // shadow
    DrawText(text, x, y, size, col);           // main
}

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;


    char title[256];
    snprintf(title, sizeof(title),
             "Raylib crowd: separation + bucket grid + accurate profiler + CACHE OPT | agents: %d",
             AGENT_COUNT);

    InitWindow(screenW, screenH, title);
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
        agents[i].speed = 100.0f;//Randf(40.0f, 120.0f);
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
    bool drawAgents = true;
    bool cullDraw = true;
    uint32_t frame = 0;

    const float arriveEps2 = ARRIVE_EPS * ARRIVE_EPS;

    // last frame raw ms (includes EndDrawing)
    double last_ms_frame=0, last_ms_grid=0, last_ms_update=0, last_ms_draw=0, last_ms_swap=0, last_ms_avoidKernel_est=0;

    // profiler EMAs (ms)
    double ema_frame=0, ema_grid=0, ema_update=0, ema_draw=0, ema_swap=0, ema_avoidKernel=0;

    // last frame counters
    uint32_t last_avoidCalls = 0;
    double   last_avgScan = 0.0;
    double   last_avgFound = 0.0;
    uint32_t last_maxScanned = 0;
    uint32_t last_maxFound = 0;
    uint32_t last_scanCapHits = 0;
    uint32_t last_neighborCapHits = 0;

    while (!WindowShouldClose()) {
        frame++;

        double tFrame0 = GetTime();

        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        if (dt > 0.05f) dt = 0.05f;

        if (IsKeyPressed(KEY_V)) avoidanceOn = !avoidanceOn;
        if (IsKeyPressed(KEY_R)) drawAgents = !drawAgents;
        if (IsKeyPressed(KEY_C)) cullDraw = !cullDraw;

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

        // Per-frame counters (this frame)
        uint32_t avoidCalls = 0;
        uint64_t avoidScannedTotal = 0;
        uint64_t avoidFoundTotal = 0;
        uint32_t maxScanned = 0;
        uint32_t maxFound = 0;
        uint32_t scanCapHits = 0;
        uint32_t neighborCapHits = 0;

        // Kernel timing sample
        uint32_t avoidKernelTimedCalls = 0;
        double avoidKernelSeconds = 0.0;

        // Grid build (only if avoidance ON)
        double tGrid0 = GetTime();
        if (avoidanceOn) {
            BuildGridBuckets(agents, AGENT_COUNT, gridW, gridH, cellCount,
                             invCell,
                             cellCountArr, cellStart, cellWrite,
                             agentCell, cellAgents);

            // CACHE OPTIMIZATION: Sort agents every second for spatial coherency
            if ((frame % 60) == 0) {
                SortAgentsByCell(agents, AGENT_COUNT, gridW, gridH, cellCount,
                                agentCell, cellAgents, cellStart);
            }
        }
        double tGrid1 = GetTime();

        // Update agents
        double tUpd0 = GetTime();

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

            // ALWAYS compute avoidance (no death spiral!)
            bool computeNewAvoidance = (((frame + (uint32_t)i) % AVOID_PERIOD) == 0);

            Vector2 avoidance = (Vector2){0,0};
            if (computeNewAvoidance) {
                int scanned = 0, found = 0;
                bool hitScan = false, hitFound = false;

                // Time 1/64th of avoidance calls
                bool timeThis = (((frame + (uint32_t)i) & 63u) == 0u);
                double tk0 = 0.0;
                if (timeThis) tk0 = GetTime();

                a->avoidCache = ComputeAvoidanceBucket(
                    agents, i, gridW, gridH, cellStart, cellAgents, invCell, radCells, true,
                    &scanned, &found, &hitScan, &hitFound
                );

                if (timeThis) {
                    avoidKernelSeconds += (GetTime() - tk0);
                    avoidKernelTimedCalls++;
                }

                avoidCalls++;
                avoidScannedTotal += (uint64_t)scanned;
                avoidFoundTotal += (uint64_t)found;

                if ((uint32_t)scanned > maxScanned) maxScanned = (uint32_t)scanned;
                if ((uint32_t)found > maxFound) maxFound = (uint32_t)found;
                if (hitScan) scanCapHits++;
                if (hitFound) neighborCapHits++;
            }
            avoidance = a->avoidCache;

            // Add wiggle when stuck, but don't replace avoidance
            Vector2 wiggle = (Vector2){0,0};
            if (a->stuck > STUCK_THRESHOLD) {
                if (vlen2(a->wiggleDir) < 1e-6f) a->wiggleDir = RandDir();
                // Scale wiggle by how stuck we are (more stuck = more wiggle)
                float wiggleMult = fminf((a->stuck - STUCK_THRESHOLD) * 0.5f, 1.0f);
                wiggle = vmul(a->wiggleDir, WIGGLE_STRENGTH * wiggleMult);
            }

            // Combine ALL forces: goal + avoidance + wiggle
            Vector2 vel = (Vector2){0,0};
            vel = vadd(vel, vmul(desiredDir, a->speed));
            vel = vadd(vel, vmul(avoidance, AVOID_STRENGTH));
            vel = vadd(vel, wiggle);  // Add wiggle, don't replace

            // Let stuck agents move faster to escape
            float maxSpeedMult = (a->stuck > STUCK_THRESHOLD) ? 1.6f : MAX_SPEED_MULTIPLIER;
            vel = vclamp_len(vel, a->speed * maxSpeedMult);
            a->vel = vel;

            a->pos = vadd(a->pos, vmul(a->vel, dt));
            a->pos.x = fminf(fmaxf(a->pos.x, 0.0f), WORLD_W);
            a->pos.y = fminf(fmaxf(a->pos.y, 0.0f), WORLD_H);
        }

        double tUpd1 = GetTime();

        // Estimate total kernel time from sampled calls:
        double ms_avoidKernel_est = 0.0;
        if (avoidKernelTimedCalls > 0 && avoidCalls > 0) {
            // Better estimator: timed_total * (avoidCalls / timedCalls)
            ms_avoidKernel_est = Ms(avoidKernelSeconds) * ((double)avoidCalls / (double)avoidKernelTimedCalls);
        }

        // Draw bounds for culling
        Vector2 tl = GetScreenToWorld2D((Vector2){0, 0}, cam);
        Vector2 br = GetScreenToWorld2D((Vector2){(float)screenW, (float)screenH}, cam);
        float margin = 8.0f;
        float minX = fminf(tl.x, br.x) - margin;
        float maxX = fmaxf(tl.x, br.x) + margin;
        float minY = fminf(tl.y, br.y) - margin;
        float maxY = fmaxf(tl.y, br.y) + margin;

        // Draw (we will display LAST frame's measured numbers)
        double tDraw0 = GetTime();

        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);
        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        if (drawAgents) {
            for (int i = 0; i < AGENT_COUNT; i++) {
                const Agent* a = &agents[i];
                Vector2 p = a->pos;

                if (cullDraw) {
                    if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY) continue;
                }

                Color col = (Color){ 200, 220, 255, 255 };
                if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
                else if (avoidanceOn && a->stuck > STUCK_THRESHOLD) col = (Color){ 255, 90, 90, 255 };

                Rectangle r = { p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f, AGENT_SIZE, AGENT_SIZE };
                DrawRectangleRec(r, col);
            }
        }

        EndMode2D();

        // Overlay text (previous frame)
        DrawTextShadow("WASD/Arrows: pan | Wheel: zoom | V: avoidance | R: draw | C: cull", 12, 12, 18, RAYWHITE);
        DrawTextShadow(avoidanceOn ? "Avoidance: ON (bucket grid + CACHE OPT)" : "Avoidance: OFF (true baseline)",
                 12, 36, 18, avoidanceOn ? GREEN : RED);
        DrawTextShadow(drawAgents ? "Draw: ON" : "Draw: OFF", 12, 60, 18, drawAgents ? GREEN : RED);
        DrawTextShadow(cullDraw ? "Cull: ON" : "Cull: OFF", 12, 84, 18, cullDraw ? GREEN : RED);

        DrawFPS(12, 108);

        int y = 136;

        DrawTextShadow(TextFormat("ms/frame: %.2f (avg %.2f)", last_ms_frame, ema_frame), 12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("grid:     %.2f (avg %.2f)", last_ms_grid,  ema_grid),  12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("update:   %.2f (avg %.2f)", last_ms_update,ema_update),12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("draw+swap:%.2f (avg %.2f)", last_ms_draw,  ema_draw),  12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("swap only:%.2f (avg %.2f)", last_ms_swap,  ema_swap),  12, y, 18, RAYWHITE); y += 22;

        DrawTextShadow(TextFormat("CELL_SIZE=%.1f radCells=%d  scanCap=%d neighCap=%d period=%d",
                                  (float)CELL_SIZE, radCells, (int)AVOID_MAX_SCAN, (int)AVOID_MAX_NEIGHBORS, (int)AVOID_PERIOD),
                       12, y, 18, RAYWHITE); y += 22;

        if (avoidanceOn) {
            DrawTextShadow(TextFormat("avoid calls: %u  avg scanned: %.1f  avg found: %.1f",
                                      last_avoidCalls, last_avgScan, last_avgFound),
                           12, y, 18, RAYWHITE); y += 22;

            DrawTextShadow(TextFormat("max scanned: %u  max found: %u  scanCapHits: %u  neighCapHits: %u",
                                      last_maxScanned, last_maxFound, last_scanCapHits, last_neighborCapHits),
                           12, y, 18, RAYWHITE); y += 22;

            DrawTextShadow(TextFormat("avoid kernel est: %.2f ms (avg %.2f)",
                                      last_ms_avoidKernel_est, ema_avoidKernel),
                           12, y, 18, RAYWHITE); y += 22;
        }

        // Measure swap/present accurately: time EndDrawing()
        double tBeforeSwap = GetTime();
        EndDrawing();
        double tFrame1 = GetTime();

        // Compute true per-frame ms INCLUDING EndDrawing()
        double ms_frame = Ms(tFrame1 - tFrame0);
        double ms_grid  = Ms(tGrid1 - tGrid0);
        double ms_upd   = Ms(tUpd1 - tUpd0);
        double ms_draw  = Ms(tFrame1 - tDraw0);        // draw + swap
        double ms_swap  = Ms(tFrame1 - tBeforeSwap);   // swap/present

        // Store "last" values for next frame overlay
        last_ms_frame = ms_frame;
        last_ms_grid  = ms_grid;
        last_ms_update= ms_upd;
        last_ms_draw  = ms_draw;
        last_ms_swap  = ms_swap;
        last_ms_avoidKernel_est = ms_avoidKernel_est;

        last_avoidCalls = avoidCalls;
        last_avgScan  = (avoidCalls > 0) ? ((double)avoidScannedTotal / (double)avoidCalls) : 0.0;
        last_avgFound = (avoidCalls > 0) ? ((double)avoidFoundTotal / (double)avoidCalls) : 0.0;
        last_maxScanned = maxScanned;
        last_maxFound = maxFound;
        last_scanCapHits = scanCapHits;
        last_neighborCapHits = neighborCapHits;

        // Smooth EMAs
        ema_frame = EMA(ema_frame, ms_frame, PROF_SMOOTH_ALPHA);
        ema_grid  = EMA(ema_grid,  ms_grid,  PROF_SMOOTH_ALPHA);
        ema_update= EMA(ema_update,ms_upd,   PROF_SMOOTH_ALPHA);
        ema_draw  = EMA(ema_draw,  ms_draw,  PROF_SMOOTH_ALPHA);
        ema_swap  = EMA(ema_swap,  ms_swap,  PROF_SMOOTH_ALPHA);
        ema_avoidKernel = EMA(ema_avoidKernel, ms_avoidKernel_est, PROF_SMOOTH_ALPHA);
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
