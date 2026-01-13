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
//  - Speed-relative avoidance (fixes slow agent stuck problem)
//
// Y-SORTING:
//  - Visible agents sorted by Y for correct 2.5D draw order
//  - Uses insertion sort (fast for mostly-sorted data frame-to-frame)

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define AGENT_COUNT 1000

#define WORLD_W 4000.0f
#define WORLD_H 4000.0f

#define AGENT_SIZE   4.0f
#define AGENT_RADIUS (AGENT_SIZE * 0.5f)

#define ARRIVE_EPS    2.0f
#define WAIT_SECONDS  1.0f

// ---- npc.gd-ish tuning ----
#define AVOID_RADIUS         40.0f
#define AVOID_STRENGTH_SCALE 0.5f
#define AVOID_MAX_NEIGHBORS  10

#define STUCK_THRESHOLD      1.5f
#define WIGGLE_STRENGTH      40.0f
#define PROGRESS_TOLERANCE   0.15f
#define STUCK_RECOVERY_RATE  1.0f

#define MAX_SPEED_MULTIPLIER 1.30f

// ---- perf knobs ----
#define CELL_SIZE        (AVOID_RADIUS * 0.5f)
#define AVOID_MAX_SCAN   48
#define AVOID_PERIOD     3

// ---- profiler knobs ----
#define PROF_SMOOTH_ALPHA 0.10

typedef struct Agent {
    Vector2 pos;
    Vector2 vel;

    Vector2 goal;
    float speed;
    float wait;

    float stuck;
    float lastDist;
    Vector2 wiggleDir;

    Vector2 avoidCache;
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

// ---------------- CACHE OPTIMIZATION: Sort agents by grid cell ----------------
static void SortAgentsByCell(Agent* agents, int N,
                             int gridW, int gridH, int cellCount,
                             int* agentCell, int* cellAgents,
                             int* cellStart)
{
    Agent* tempAgents = (Agent*)MemAlloc((size_t)N * sizeof(Agent));
    int* oldToNew = (int*)MemAlloc((size_t)N * sizeof(int));

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

    for (int i = 0; i < N; i++) {
        agents[i] = tempAgents[i];
    }

    for (int cell = 0; cell < cellCount; cell++) {
        int start = cellStart[cell];
        int end = cellStart[cell + 1];

        for (int t = start; t < end; t++) {
            int oldIdx = cellAgents[t];
            cellAgents[t] = oldToNew[oldIdx];
        }
    }

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

// ---------------- Y-SORT for 2.5D rendering ----------------
// Row-bucket approach: O(n) bucketing + tiny sorts per row
#define YBAND_HEIGHT 32.0f  // pixels per band - tune this

static void SortDrawOrderByYBands(const Agent* agents, int* drawOrder, int count,
                                   float minY, float maxY,
                                   int* bandCounts, int* bandStarts, int* tempOrder) {
    float range = maxY - minY;
    if (range < 1.0f) range = 1.0f;

    int numBands = (int)ceilf(range / YBAND_HEIGHT);
    if (numBands < 1) numBands = 1;
    if (numBands > 4096) numBands = 4096;  // sanity cap

    float invBandH = (float)numBands / range;

    // Count agents per band
    for (int b = 0; b < numBands; b++) bandCounts[b] = 0;

    for (int i = 0; i < count; i++) {
        int idx = drawOrder[i];
        float y = agents[idx].pos.y;
        int band = (int)((y - minY) * invBandH);
        if (band < 0) band = 0;
        if (band >= numBands) band = numBands - 1;
        bandCounts[band]++;
    }

    // Prefix sum for band starts
    bandStarts[0] = 0;
    for (int b = 0; b < numBands; b++) {
        bandStarts[b + 1] = bandStarts[b] + bandCounts[b];
    }

    // Reset counts as write cursors
    for (int b = 0; b < numBands; b++) bandCounts[b] = bandStarts[b];

    // Scatter into temp array by band
    for (int i = 0; i < count; i++) {
        int idx = drawOrder[i];
        float y = agents[idx].pos.y;
        int band = (int)((y - minY) * invBandH);
        if (band < 0) band = 0;
        if (band >= numBands) band = numBands - 1;
        tempOrder[bandCounts[band]++] = idx;
    }

    // Copy back - agents now grouped by band, drawn top to bottom
    for (int i = 0; i < count; i++) {
        drawOrder[i] = tempOrder[i];
    }

    // Optional: insertion sort within each band (bands are small!)
    for (int b = 0; b < numBands; b++) {
        int start = bandStarts[b];
        int end = bandStarts[b + 1];
        // Insertion sort this small range
        for (int i = start + 1; i < end; i++) {
            int key = drawOrder[i];
            float keyY = agents[key].pos.y;
            int j = i - 1;
            while (j >= start && agents[drawOrder[j]].pos.y > keyY) {
                drawOrder[j + 1] = drawOrder[j];
                j--;
            }
            drawOrder[j + 1] = key;
        }
    }
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
    DrawText(text, x + 1, y + 1, size, BLACK);
    DrawText(text, x, y, size, col);
}

int main(void) {
    const int screenW = 1280;
    const int screenH = 720;

    char title[256];
    snprintf(title, sizeof(title),
             "Raylib crowd: separation + bucket grid + Y-sort | agents: %d",
             AGENT_COUNT);

    InitWindow(screenW, screenH, title);
     Texture2D agentTex = LoadTexture("agent.png");
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

    // Y-sort draw order array (persists across frames for incremental sorting)
    int* drawOrder = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));
    int* tempOrder = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));  // for band sort
    int* bandCounts = (int*)MemAlloc(4097 * sizeof(int));  // max bands + 1
    int* bandStarts = (int*)MemAlloc(4097 * sizeof(int));
    int visibleCount = 0;

    int radCells = (int)ceilf(AVOID_RADIUS / CELL_SIZE);
    if (radCells < 1) radCells = 1;

    bool avoidanceOn = false;
    bool drawAgents = true;
    bool cullDraw = true;
    bool ySortOn = true;  // Toggle Y-sorting with Y key
    uint32_t frame = 0;

    const float arriveEps2 = ARRIVE_EPS * ARRIVE_EPS;

    double last_ms_frame=0, last_ms_grid=0, last_ms_update=0, last_ms_draw=0, last_ms_swap=0, last_ms_avoidKernel_est=0;
    double last_ms_ysort = 0;

    double ema_frame=0, ema_grid=0, ema_update=0, ema_draw=0, ema_swap=0, ema_avoidKernel=0;
    double ema_ysort = 0;

    uint32_t last_avoidCalls = 0;
    double   last_avgScan = 0.0;
    double   last_avgFound = 0.0;
    uint32_t last_maxScanned = 0;
    uint32_t last_maxFound = 0;
    uint32_t last_scanCapHits = 0;
    uint32_t last_neighborCapHits = 0;
    int last_visibleCount = 0;

    while (!WindowShouldClose()) {
        frame++;

        double tFrame0 = GetTime();

        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;
        if (dt > 0.05f) dt = 0.05f;

        if (IsKeyPressed(KEY_V)) avoidanceOn = !avoidanceOn;
        if (IsKeyPressed(KEY_R)) drawAgents = !drawAgents;
        if (IsKeyPressed(KEY_C)) cullDraw = !cullDraw;
        if (IsKeyPressed(KEY_Y)) ySortOn = !ySortOn;

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

        uint32_t avoidCalls = 0;
        uint64_t avoidScannedTotal = 0;
        uint64_t avoidFoundTotal = 0;
        uint32_t maxScanned = 0;
        uint32_t maxFound = 0;
        uint32_t scanCapHits = 0;
        uint32_t neighborCapHits = 0;

        uint32_t avoidKernelTimedCalls = 0;
        double avoidKernelSeconds = 0.0;

        double tGrid0 = GetTime();
        if (avoidanceOn) {
            BuildGridBuckets(agents, AGENT_COUNT, gridW, gridH, cellCount,
                             invCell,
                             cellCountArr, cellStart, cellWrite,
                             agentCell, cellAgents);

            if ((frame % 60) == 0) {
                SortAgentsByCell(agents, AGENT_COUNT, gridW, gridH, cellCount,
                                agentCell, cellAgents, cellStart);
            }
        }
        double tGrid1 = GetTime();

        double tUpd0 = GetTime();

        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent* a = &agents[i];

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

            float invDist = 1.0f / sqrtf(dist2);
            float dist = dist2 * invDist;
            Vector2 desiredDir = vmul(to, invDist);

            float progress = a->lastDist - dist;
            float expected = a->speed * dt * PROGRESS_TOLERANCE;

            if (progress < expected) {
                a->stuck += dt;
            } else {
                a->stuck = fmaxf(0.0f, a->stuck - dt * STUCK_RECOVERY_RATE);
                if (a->stuck <= 0.0f) a->wiggleDir = (Vector2){0,0};
            }
            a->lastDist = dist;

            bool computeNewAvoidance = (((frame + (uint32_t)i) % AVOID_PERIOD) == 0);

            Vector2 avoidance = (Vector2){0,0};
            if (computeNewAvoidance) {
                int scanned = 0, found = 0;
                bool hitScan = false, hitFound = false;

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

            Vector2 wiggle = (Vector2){0,0};
            if (a->stuck > STUCK_THRESHOLD) {
                if (vlen2(a->wiggleDir) < 1e-6f) a->wiggleDir = RandDir();
                float wiggleMult = fminf((a->stuck - STUCK_THRESHOLD) * 0.5f, 1.0f);
                wiggle = vmul(a->wiggleDir, WIGGLE_STRENGTH * wiggleMult);
            }

            Vector2 vel = (Vector2){0,0};
            vel = vadd(vel, vmul(desiredDir, a->speed));

            float avoidScale = a->speed * AVOID_STRENGTH_SCALE;
            vel = vadd(vel, vmul(avoidance, avoidScale));

            vel = vadd(vel, wiggle);

            float maxSpeedMult = (a->stuck > STUCK_THRESHOLD) ? 1.6f : MAX_SPEED_MULTIPLIER;
            vel = vclamp_len(vel, a->speed * maxSpeedMult);
            a->vel = vel;

            a->pos = vadd(a->pos, vmul(a->vel, dt));
            a->pos.x = fminf(fmaxf(a->pos.x, 0.0f), WORLD_W);
            a->pos.y = fminf(fmaxf(a->pos.y, 0.0f), WORLD_H);
        }

        double tUpd1 = GetTime();

        double ms_avoidKernel_est = 0.0;
        if (avoidKernelTimedCalls > 0 && avoidCalls > 0) {
            ms_avoidKernel_est = Ms(avoidKernelSeconds) * ((double)avoidCalls / (double)avoidKernelTimedCalls);
        }

        // Compute visible bounds for culling
        Vector2 tl = GetScreenToWorld2D((Vector2){0, 0}, cam);
        Vector2 br = GetScreenToWorld2D((Vector2){(float)screenW, (float)screenH}, cam);
        // Margin accounts for sprite height so agents smoothly enter from top
        float margin = (float)agentTex.height + 16.0f;
        float minX = fminf(tl.x, br.x) - margin;
        float maxX = fmaxf(tl.x, br.x) + margin;
        float minY = fminf(tl.y, br.y) - margin;
        float maxY = fmaxf(tl.y, br.y) + margin;

        // Build draw order: collect visible agents
        double tYSort0 = GetTime();
        visibleCount = 0;

        if (drawAgents) {
            if (cullDraw) {
                for (int i = 0; i < AGENT_COUNT; i++) {
                    Vector2 p = agents[i].pos;
                    if (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY) {
                        drawOrder[visibleCount++] = i;
                    }
                }
            } else {
                // No culling: all agents visible
                for (int i = 0; i < AGENT_COUNT; i++) {
                    drawOrder[visibleCount++] = i;
                }
            }

            // Sort visible agents by Y for correct 2.5D draw order
            if (ySortOn && visibleCount > 0) {
                SortDrawOrderByYBands(agents, drawOrder, visibleCount,
                                      minY, maxY, bandCounts, bandStarts, tempOrder);
            }
        }
        double tYSort1 = GetTime();

        // Draw
        double tDraw0 = GetTime();

        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);
        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        if (drawAgents) {
            // Draw in Y-sorted order (lower Y = further back = drawn first)
            for (int v = 0; v < visibleCount; v++) {
                int i = drawOrder[v];
                const Agent* a = &agents[i];
                Vector2 p = a->pos;

                Color col = (Color){ 200, 220, 255, 255 };
                if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
                else if (avoidanceOn && a->stuck > STUCK_THRESHOLD) col = (Color){ 255, 90, 90, 255 };

                //Rectangle r = { p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f, AGENT_SIZE, AGENT_SIZE };
                //DrawRectangleRec(r, col);

                 Rectangle src = { 0, 0, agentTex.width, agentTex.height };
                 Rectangle dst = { p.x, p.y, agentTex.width, agentTex.height };
                 Vector2 origin = { agentTex.width* 0.5f, agentTex.height };
                 DrawTexturePro(agentTex, src, dst, origin, 0.0f, col);
            }
        }

        EndMode2D();

        // Overlay text
        DrawTextShadow("WASD/Arrows: pan | Wheel: zoom | V: avoidance | R: draw | C: cull | Y: y-sort", 12, 12, 18, RAYWHITE);
        DrawTextShadow(avoidanceOn ? "Avoidance: ON (speed-relative + cache opt)" : "Avoidance: OFF (true baseline)",
                 12, 36, 18, avoidanceOn ? GREEN : RED);
        DrawTextShadow(drawAgents ? "Draw: ON" : "Draw: OFF", 12, 60, 18, drawAgents ? GREEN : RED);
        DrawTextShadow(cullDraw ? "Cull: ON" : "Cull: OFF", 120, 60, 18, cullDraw ? GREEN : RED);
        DrawTextShadow(ySortOn ? "Y-Sort: ON" : "Y-Sort: OFF", 230, 60, 18, ySortOn ? GREEN : RED);

        DrawFPS(12, 84);

        int y = 112;

        DrawTextShadow(TextFormat("ms/frame: %.2f (avg %.2f)", last_ms_frame, ema_frame), 12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("grid:     %.2f (avg %.2f)", last_ms_grid,  ema_grid),  12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("update:   %.2f (avg %.2f)", last_ms_update,ema_update),12, y, 18, RAYWHITE); y += 22;
        DrawTextShadow(TextFormat("y-sort:   %.2f (avg %.2f)  visible: %d", last_ms_ysort, ema_ysort, last_visibleCount), 12, y, 18, RAYWHITE); y += 22;
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

        double tBeforeSwap = GetTime();
        EndDrawing();
        double tFrame1 = GetTime();

        double ms_frame = Ms(tFrame1 - tFrame0);
        double ms_grid  = Ms(tGrid1 - tGrid0);
        double ms_upd   = Ms(tUpd1 - tUpd0);
        double ms_ysort = Ms(tYSort1 - tYSort0);
        double ms_draw  = Ms(tFrame1 - tDraw0);
        double ms_swap  = Ms(tFrame1 - tBeforeSwap);

        last_ms_frame = ms_frame;
        last_ms_grid  = ms_grid;
        last_ms_update= ms_upd;
        last_ms_ysort = ms_ysort;
        last_ms_draw  = ms_draw;
        last_ms_swap  = ms_swap;
        last_ms_avoidKernel_est = ms_avoidKernel_est;
        last_visibleCount = visibleCount;

        last_avoidCalls = avoidCalls;
        last_avgScan  = (avoidCalls > 0) ? ((double)avoidScannedTotal / (double)avoidCalls) : 0.0;
        last_avgFound = (avoidCalls > 0) ? ((double)avoidFoundTotal / (double)avoidCalls) : 0.0;
        last_maxScanned = maxScanned;
        last_maxFound = maxFound;
        last_scanCapHits = scanCapHits;
        last_neighborCapHits = neighborCapHits;

        ema_frame = EMA(ema_frame, ms_frame, PROF_SMOOTH_ALPHA);
        ema_grid  = EMA(ema_grid,  ms_grid,  PROF_SMOOTH_ALPHA);
        ema_update= EMA(ema_update,ms_upd,   PROF_SMOOTH_ALPHA);
        ema_ysort = EMA(ema_ysort, ms_ysort, PROF_SMOOTH_ALPHA);
        ema_draw  = EMA(ema_draw,  ms_draw,  PROF_SMOOTH_ALPHA);
        ema_swap  = EMA(ema_swap,  ms_swap,  PROF_SMOOTH_ALPHA);
        ema_avoidKernel = EMA(ema_avoidKernel, ms_avoidKernel_est, PROF_SMOOTH_ALPHA);
    }

    MemFree(drawOrder);
    MemFree(tempOrder);
    MemFree(bandCounts);
    MemFree(bandStarts);
    MemFree(cellCountArr);
    MemFree(cellStart);
    MemFree(cellWrite);
    MemFree(agentCell);
    MemFree(cellAgents);
    MemFree(agents);
      UnloadTexture(agentTex);
    CloseWindow();
    return 0;
}
