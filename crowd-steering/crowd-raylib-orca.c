// crowd-raylib.c
// 2D top-down open world with 5000 agents + ORCA local avoidance.
// Improvements for dense crowds:
//  - side preference (keep-left/keep-right bias)
//  - stuck detection + flip side preference
//  - adaptive time horizon when crowded
//  - slight radius fudge (crowd compression)
//  - acceleration limit (reduces jitter / "butterflies attached")

#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>

#define AGENT_COUNT 500

#define WORLD_W 1000.0f
#define WORLD_H 1000.0f

#define AGENT_SIZE 4.0f
#define AGENT_RADIUS (AGENT_SIZE * 0.5f)

#define ARRIVE_EPS 2.0f
#define WAIT_SECONDS 1.0f

// ---- ORCA tuning ----
#define NEIGHBOR_DIST 70.0f      // neighbor consideration distance
#define MAX_NEIGHBORS 24         // cap neighbors per agent
#define CELL_SIZE NEIGHBOR_DIST  // spatial hash cell size

// Dense behavior tuning
#define TIME_HORIZON_BASE 2.5f   // seconds
#define TIME_HORIZON_CROWDED 1.2f
#define TIME_HORIZON_VERY_CROWDED 0.9f

#define SIDE_BIAS_FRAC 0.12f     // 0..0.25 (stronger => more "laning" tendency)
#define RADIUS_FUDGE   1.15f     // <1 => less conservative, fewer freezes (too low => overlaps)

#define STUCK_SPEED    4.0f      // below this speed => "stuck"
#define STUCK_TIME     0.6f      // seconds before flipping side

// Velocity smoothing / stability
#define MAX_ACCEL      900.0f    // units/sec^2 (limits how fast velocity can change)
#define DAMPING        1.0f     // mild damping per frame (helps settle jitter)

typedef struct Agent {
    Vector2 pos;
    Vector2 vel;        // current velocity
    Vector2 goal;
    float maxSpeed;     // max speed
    float wait;         // waiting time remaining

    int side;           // +1 or -1 (right/left preference)
    float stuck;        // seconds stuck
    float share;        // reciprocity share (slight randomness helps break symmetry)
} Agent;

typedef struct Line {
    Vector2 point;      // point on line
    Vector2 direction;  // direction of line (should be normalized-ish)
} Line;

// ---------- vector helpers ----------
static inline Vector2 vadd(Vector2 a, Vector2 b) { return (Vector2){a.x+b.x, a.y+b.y}; }
static inline Vector2 vsub(Vector2 a, Vector2 b) { return (Vector2){a.x-b.x, a.y-b.y}; }
static inline Vector2 vmul(Vector2 a, float s)   { return (Vector2){a.x*s, a.y*s}; }
static inline float   vdot(Vector2 a, Vector2 b) { return a.x*b.x + a.y*b.y; }
static inline float   vdet(Vector2 a, Vector2 b) { return a.x*b.y - a.y*b.x; }
static inline float   vlen2(Vector2 a)           { return vdot(a,a); }
static inline float   vlen(Vector2 a)            { return sqrtf(vlen2(a)); }
static inline Vector2 vperp(Vector2 a)           { return (Vector2){a.y, -a.x}; }

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

// ---------- random helpers ----------
static float Randf(float minv, float maxv) {
    int r = GetRandomValue(0, 10000);
    float t = (float)r / 10000.0f;
    return minv + (maxv - minv) * t;
}

static Vector2 RandGoal(void) {
    return (Vector2){ Randf(0.0f, WORLD_W), Randf(0.0f, WORLD_H) };
}

// ---------- ORCA linear programs (simplified RVO2-style) ----------
static int linearProgram1(const Line* lines, int lineNo, float radius, Vector2 optVelocity,
                          int directionOpt, Vector2* result)
{
    const Line* line = &lines[lineNo];

    float dotp = vdot(line->point, line->direction);
    float discriminant = dotp*dotp + radius*radius - vlen2(line->point);

    if (discriminant < 0.0f) return 0;

    float sqrtDisc = sqrtf(discriminant);
    float tLeft  = -dotp - sqrtDisc;
    float tRight = -dotp + sqrtDisc;

    for (int i = 0; i < lineNo; i++) {
        float denom = vdet(line->direction, lines[i].direction);
        float numer = vdet(lines[i].direction, vsub(line->point, lines[i].point));

        if (fabsf(denom) < 1e-6f) {
            if (numer < 0.0f) return 0;
            continue;
        }

        float t = numer / denom;
        if (denom >= 0.0f) {
            if (t > tLeft) tLeft = t;
        } else {
            if (t < tRight) tRight = t;
        }
        if (tLeft > tRight) return 0;
    }

    if (directionOpt) {
        if (vdot(optVelocity, line->direction) > 0.0f) {
            *result = vadd(line->point, vmul(line->direction, tRight));
        } else {
            *result = vadd(line->point, vmul(line->direction, tLeft));
        }
    } else {
        float t = vdot(line->direction, vsub(optVelocity, line->point));
        if (t < tLeft) t = tLeft;
        else if (t > tRight) t = tRight;
        *result = vadd(line->point, vmul(line->direction, t));
    }

    return 1;
}

static int linearProgram2(const Line* lines, int numLines, float radius, Vector2 optVelocity,
                          int directionOpt, Vector2* result)
{
    if (directionOpt) {
        *result = vmul(vnormalize(optVelocity), radius);
    } else {
        if (vlen2(optVelocity) > radius*radius) {
            *result = vmul(vnormalize(optVelocity), radius);
        } else {
            *result = optVelocity;
        }
    }

    for (int i = 0; i < numLines; i++) {
        if (vdet(lines[i].direction, vsub(lines[i].point, *result)) > 0.0f) {
            Vector2 temp = *result;
            if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                *result = temp;
                return i;
            }
        }
    }
    return numLines;
}

static void linearProgram3(const Line* lines, int numLines, int beginLine, float radius, Vector2* result)
{
    float distance = 0.0f;

    for (int i = beginLine; i < numLines; i++) {
        float violation = vdet(lines[i].direction, vsub(lines[i].point, *result));
        if (violation > distance) {
            Line projLines[MAX_NEIGHBORS];
            int projCount = 0;

            for (int j = 0; j < i; j++) {
                Line line;

                float detv = vdet(lines[i].direction, lines[j].direction);
                if (fabsf(detv) < 1e-6f) {
                    if (vdot(lines[i].direction, lines[j].direction) > 0.0f) {
                        continue;
                    } else {
                        line.point = vmul(vadd(lines[i].point, lines[j].point), 0.5f);
                    }
                } else {
                    Vector2 diff = vsub(lines[j].point, lines[i].point);
                    float t = vdet(lines[j].direction, diff) / detv;
                    line.point = vadd(lines[i].point, vmul(lines[i].direction, t));
                }

                Vector2 dir = vsub(lines[j].direction, lines[i].direction);
                line.direction = vnormalize(dir);
                if (vlen2(line.direction) < 1e-8f) continue;
                projLines[projCount++] = line;
            }

            Vector2 temp = *result;
            Vector2 optDir = vperp(lines[i].direction); // direction-only optimization
            int lpFail = linearProgram2(projLines, projCount, radius, optDir, 1, result);
            if (lpFail < projCount) {
                *result = temp;
            }

            distance = vdet(lines[i].direction, vsub(lines[i].point, *result));
        }
    }
}

static void ResolveOverlaps(Agent* agents, int agentCount, int gridW, int gridH, int* head, int* next)
{
    // rebuild grid using UPDATED positions
    int cellCount = gridW * gridH;
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

    const float minDist = (AGENT_RADIUS + AGENT_RADIUS);   // use TRUE radius here
    const float minDistSq = minDist * minDist;

    // One or two iterations is enough
    for (int iter = 0; iter < 2; iter++) {
        for (int i = 0; i < agentCount; i++) {
            Agent* a = &agents[i];

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
                        if (j <= i) continue; // each pair once

                        Agent* b = &agents[j];
                        Vector2 d = vsub(b->pos, a->pos);
                        float dsq = vlen2(d);
                        if (dsq >= minDistSq) continue;

                        float dist = sqrtf(fmaxf(dsq, 1e-8f));
                        Vector2 n = vmul(d, 1.0f / dist);
                        float pen = (minDist - dist);

                        // push both away equally
                        Vector2 push = vmul(n, 0.5f * pen);
                        a->pos = vsub(a->pos, push);
                        b->pos = vadd(b->pos, push);

                        // --- velocity correction so they don't immediately "re-collide" ---
                        Vector2 relVel = vsub(b->vel, a->vel);
                        float closing = vdot(relVel, n);          // positive means separating, negative means closing
                        if (closing < 0.0f) {
                            // remove half of the closing speed from each agent
                            Vector2 corr = vmul(n, 0.5f * (-closing));
                            a->vel = vsub(a->vel, corr);
                            b->vel = vadd(b->vel, corr);
                        }

                    }
                }
            }
        }
    }
}

// Compute ORCA-corrected velocity for one agent given neighbor indices
static Vector2 orca_compute_velocity(const Agent* agents, int self, const int* neighbors, int nCount, float dt)
{
    const Agent* a = &agents[self];

    // adaptive horizon: less conservative when crowded (reduces "freeze")
    float timeH = TIME_HORIZON_BASE;
    if (nCount >= (MAX_NEIGHBORS / 2)) timeH = TIME_HORIZON_CROWDED;
    if (nCount >= (MAX_NEIGHBORS - 2)) timeH = TIME_HORIZON_VERY_CROWDED;
    const float invTimeHorizon = 1.0f / timeH;

    Line lines[MAX_NEIGHBORS];
    int lineCount = 0;

    for (int k = 0; k < nCount; k++) {
        int j = neighbors[k];
        const Agent* b = &agents[j];

        Vector2 relPos = vsub(b->pos, a->pos);
        Vector2 relVel = vsub(a->vel, b->vel);

        float distSq = vlen2(relPos);
        float combinedRadius = (AGENT_RADIUS + AGENT_RADIUS) * RADIUS_FUDGE;
        float combinedRadiusSq = combinedRadius * combinedRadius;

        Line line;
        Vector2 u;

        if (distSq > combinedRadiusSq) {
            // No collision (within radius): build constraint for time horizon
            Vector2 w = vsub(relVel, vmul(relPos, invTimeHorizon));
            float wLenSq = vlen2(w);
            float dot1 = vdot(w, relPos);

            if (dot1 < 0.0f && dot1*dot1 > combinedRadiusSq * wLenSq) {
                // Project on cut-off circle
                float wLen = sqrtf(wLenSq);
                Vector2 unitW = (wLen > 1e-6f) ? vmul(w, 1.0f / wLen) : (Vector2){0,0};

                line.direction = vperp(unitW);
                u = vmul(unitW, (combinedRadius * invTimeHorizon - wLen));
            } else {
                // Project on legs
                float leg = sqrtf(fmaxf(0.0f, distSq - combinedRadiusSq));

                if (vdet(relPos, w) > 0.0f) {
                    // left leg
                    line.direction = (Vector2){
                        (relPos.x * leg - relPos.y * combinedRadius) / distSq,
                        (relPos.x * combinedRadius + relPos.y * leg) / distSq
                    };
                } else {
                    // right leg
                    line.direction = (Vector2){
                        -(relPos.x * leg + relPos.y * combinedRadius) / distSq,
                        (relPos.x * combinedRadius - relPos.y * leg) / distSq
                    };
                }

                line.direction = vnormalize(line.direction);

                float dot2 = vdot(relVel, line.direction);
                u = vsub(vmul(line.direction, dot2), relVel);
            }
        } else {
            // Collision now: use current timestep
            float invTimeStep = (dt > 1e-6f) ? (1.0f / dt) : 60.0f;
            Vector2 w = vsub(relVel, vmul(relPos, invTimeStep));
            float wLen = vlen(w);

            Vector2 unitW = (wLen > 1e-6f) ? vmul(w, 1.0f / wLen) : vnormalize(relPos);

            line.direction = vperp(unitW);
            u = vmul(unitW, (combinedRadius * invTimeStep - wLen));
        }

        // Slightly asymmetric reciprocity share helps break "paired jitter"
        float share = a->share; // ~0.45..0.55
        line.point = vadd(a->vel, vmul(u, share));

        line.direction = vnormalize(line.direction);
        if (vlen2(line.direction) < 1e-8f) continue;

        lines[lineCount++] = line;
        if (lineCount >= MAX_NEIGHBORS) break;
    }

    // Preferred velocity toward goal + side bias to break symmetry
    Vector2 toGoal = vsub(a->goal, a->pos);
    float d = vlen(toGoal);

    Vector2 vPref = (Vector2){0,0};
    if (d > 1e-6f) {
        Vector2 dir = vmul(toGoal, 1.0f / d);
        vPref = vmul(dir, a->maxSpeed);

        // Overshoot prevention: if close, aim to arrive in one step
        float maxStep = a->maxSpeed * dt;
        if (d < maxStep && dt > 1e-6f) vPref = vmul(toGoal, 1.0f / dt);

        // side preference (keep-left/keep-right)
        Vector2 side = vperp(dir);
        vPref = vadd(vPref, vmul(side, (float)a->side * a->maxSpeed * SIDE_BIAS_FRAC));
        vPref = vclamp_len(vPref, a->maxSpeed);
    }

    // Solve: closest feasible velocity to vPref within max speed circle
    Vector2 newVel;
    int lpFail = linearProgram2(lines, lineCount, a->maxSpeed, vPref, 0, &newVel);
    if (lpFail < lineCount) {
        linearProgram3(lines, lineCount, lpFail, a->maxSpeed, &newVel);
    }

    return newVel;
}

int main(void)
{
    const int screenW = 1280;
    const int screenH = 720;

    InitWindow(screenW, screenH, "Raylib crowd: 5000 agents (ORCA + anti-deadlock)");
    SetTargetFPS(60);

    srand((unsigned)time(NULL));

    Camera2D cam = {0};
    cam.offset = (Vector2){ screenW * 0.5f, screenH * 0.5f };
    cam.target = (Vector2){ WORLD_W * 0.5f, WORLD_H * 0.5f };
    cam.zoom = 1.0f;

    Agent agents[AGENT_COUNT];
    Vector2 desiredVel[AGENT_COUNT];

    for (int i = 0; i < AGENT_COUNT; i++) {
        agents[i].pos = RandGoal();
        agents[i].goal = RandGoal();
        agents[i].vel = (Vector2){0,0};
        agents[i].maxSpeed = Randf(10.0f, 30.0f);
        agents[i].wait = 0.0f;

        agents[i].side = (GetRandomValue(0, 1) == 0) ? -1 : +1;
        agents[i].stuck = 0.0f;
        agents[i].share = Randf(0.45f, 0.55f); // tiny asymmetry to reduce pair jitter

        desiredVel[i] = (Vector2){0,0};
    }

    // Spatial hash
    const int gridW = (int)ceilf(WORLD_W / CELL_SIZE);
    const int gridH = (int)ceilf(WORLD_H / CELL_SIZE);
    const int cellCount = gridW * gridH;

    int* head = (int*)MemAlloc((size_t)cellCount * sizeof(int));
    int* next = (int*)MemAlloc((size_t)AGENT_COUNT * sizeof(int));

    bool useOrca = true;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;

        if (IsKeyPressed(KEY_O)) useOrca = !useOrca;

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

        // Clamp camera target
        if (cam.target.x < 0) cam.target.x = 0;
        if (cam.target.y < 0) cam.target.y = 0;
        if (cam.target.x > WORLD_W) cam.target.x = WORLD_W;
        if (cam.target.y > WORLD_H) cam.target.y = WORLD_H;

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

        const float neighborDistSq = NEIGHBOR_DIST * NEIGHBOR_DIST;

        // Compute desired velocities from a snapshot (ORCA uses current positions+vels)
        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent* a = &agents[i];

            // waiting
            if (a->wait > 0.0f) {
                a->wait -= dt;
                if (a->wait <= 0.0f) {
                    a->goal = RandGoal();
                    a->wait = 0.0f;
                }
                desiredVel[i] = (Vector2){0,0};
                continue;
            }

            // arrival
            Vector2 to = vsub(a->goal, a->pos);
            if (vlen2(to) <= ARRIVE_EPS * ARRIVE_EPS) {
                a->pos = a->goal;
                a->wait = WAIT_SECONDS;
                a->stuck = 0.0f;
                desiredVel[i] = (Vector2){0,0};
                continue;
            }

            if (!useOrca) {
                // baseline straight-to-goal
                float d = vlen(to);
                Vector2 dir = (d > 1e-6f) ? vmul(to, 1.0f/d) : (Vector2){0,0};
                Vector2 vPref = vmul(dir, a->maxSpeed);

                float maxStep = a->maxSpeed * dt;
                if (d < maxStep && dt > 1e-6f) vPref = vmul(to, 1.0f / dt);

                desiredVel[i] = vclamp_len(vPref, a->maxSpeed);
                continue;
            }

            // gather neighbors
            int neighbors[MAX_NEIGHBORS];
            int nCount = 0;

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
                        if (j == i) continue;

                        Vector2 dpos = vsub(agents[j].pos, a->pos);
                        if (vlen2(dpos) > neighborDistSq) continue;

                        neighbors[nCount++] = j;
                        if (nCount >= MAX_NEIGHBORS) break;
                    }
                    if (nCount >= MAX_NEIGHBORS) break;
                }
                if (nCount >= MAX_NEIGHBORS) break;
            }

            desiredVel[i] = orca_compute_velocity(agents, i, neighbors, nCount, dt);
        }

        // Apply velocities with acceleration limit + integrate
        for (int i = 0; i < AGENT_COUNT; i++) {
            Agent* a = &agents[i];

            Vector2 vTarget = desiredVel[i];

            // acceleration clamp: a->vel moves toward vTarget
            Vector2 dv = vsub(vTarget, a->vel);
            float maxDv = MAX_ACCEL * dt;
            dv = vclamp_len(dv, maxDv);
            a->vel = vadd(a->vel, dv);

            // mild damping to reduce jitter in dense packs
            a->vel = vmul(a->vel, DAMPING);

            // stuck detection: if barely moving and not near goal, flip side
            if (a->wait <= 0.0f) {
                float sp = vlen(a->vel);
                float gd2 = vlen2(vsub(a->goal, a->pos));
                if (gd2 > (ARRIVE_EPS*ARRIVE_EPS) * 4.0f && sp < STUCK_SPEED) {
                    a->stuck += dt;
                    if (a->stuck > STUCK_TIME) {
                        a->side = -a->side;
                        a->share = Randf(0.45f, 0.55f); // also refresh asymmetry
                        a->stuck = 0.0f;
                    }
                } else {
                    a->stuck = 0.0f;
                }
            }

            // integrate
            a->pos = vadd(a->pos, vmul(a->vel, dt));

            // keep inside world
            if (a->pos.x < 0) a->pos.x = 0;
            if (a->pos.y < 0) a->pos.y = 0;
            if (a->pos.x > WORLD_W) a->pos.x = WORLD_W;
            if (a->pos.y > WORLD_H) a->pos.y = WORLD_H;

            // arrival after move
            if (a->wait <= 0.0f) {
                Vector2 to = vsub(a->goal, a->pos);
                if (vlen2(to) <= ARRIVE_EPS * ARRIVE_EPS) {
                    a->pos = a->goal;
                    a->wait = WAIT_SECONDS;
                    a->vel = (Vector2){0,0};
                    a->stuck = 0.0f;
                }
            }
        }

        ResolveOverlaps(agents, AGENT_COUNT, gridW, gridH, head, next);

        // Draw
        BeginDrawing();
        ClearBackground((Color){ 20, 20, 25, 255 });

        BeginMode2D(cam);

        DrawRectangleLines(0, 0, (int)WORLD_W, (int)WORLD_H, (Color){ 80, 80, 90, 255 });

        for (int i = 0; i < AGENT_COUNT; i++) {
            const Agent* a = &agents[i];
            Vector2 p = a->pos;
            //Rectangle r = { p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f, AGENT_SIZE, AGENT_SIZE };

            // debug coloring: stuck-ish agents show red
            Color col = (Color){ 200, 220, 255, 255 };
            if (a->wait > 0.0f) col = (Color){ 240, 220, 120, 255 };
            else if (a->stuck > 0.2f) col = (Color){ 255, 90, 90, 255 };


            DrawCircleV(p, AGENT_RADIUS, col);
            // DrawCircle(p.x - AGENT_SIZE * 0.5f, p.y - AGENT_SIZE * 0.5f,  AGENT_SIZE * 0.5f, col);
            //DrawRectangleRec(r, col);
        }

        EndMode2D();

        DrawText("WASD/Arrows: pan | Mouse wheel: zoom | O: toggle ORCA", 12, 12, 18, RAYWHITE);
        DrawText(useOrca ? "ORCA: ON" : "ORCA: OFF", 12, 36, 18, useOrca ? GREEN : RED);
        DrawFPS(12, 60);

        EndDrawing();
    }

    MemFree(head);
    MemFree(next);

    CloseWindow();
    return 0;
}
