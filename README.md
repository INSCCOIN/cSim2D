# cSim2D

2.5-D sim engine for the SharkDeck. Physics live on an **XZ floor** with **Y up**. The painter is a perspective camera, a Mode-7 ground, and depth-sorted spheres / boxes. Games do not touch `phys.c` or `draw.c`.

Rebuild **on the device**. Do not copy `.o` files from a PC.

```bash
cd /home/working/cSim2D
rm -f *.o cSim2D
make
./cSim2D
```

`demo.c` is the witness scene (ball + crate stack), not the engine.

## Demo keys

| Key | |
|-----|--|
| W A S D / arrows | kick the ball in camera XZ |
| Q E | yaw |
| Space | hop |
| R | reset scene |
| X | quit |

## Units and axes

SI. Position metres, mass kg, force through `sim_kick` as Δp (impulse-ish: `v += F * inv_mass`).

```
        +Y up
         |
         |
         +——— +X right
        /
       /
     +Z forward (into the world, away from default camera)
```

Floor is `Y = 0`. A sphere of radius `r` should be spawned at `y = r` or it will pop up on the first step. Gravity default is `(0, -22, 0)`.

Cap: **48 bodies**.

## Files

| File | Role |
|------|------|
| `sim.h` | public API |
| `phys.c` | integrate, contacts, floor |
| `draw.c` | camera + floor + solids |
| `fb.c` / `fb.h` | `/dev/fb0` back buffer |
| `demo.c` | example game |

Link a game with `phys.o draw.o fb.o -lm`.

## API

```c
void sim_init(Sim *s);
int  sim_add_sphere(Sim *s, float x, float y, float z, float r, float mass);
int  sim_add_box(Sim *s, float x, float y, float z,
                 float sx, float sy, float sz, float mass);
void sim_static(Sim *s, int id);
void sim_kick(Sim *s, int id, float fx, float fy, float fz);
void sim_step(Sim *s, float dt);          /* call at 1/60 */
void sim_cam_follow(Sim *s, int id);      /* optional */
void sim_draw(const Sim *s);              /* includes fb_flip */
```

Add functions return a body **id**, or `-1` if the world is full.

`sim_static(id)` pins a body (infinite mass). Use it for walls and platforms.

`sim_kick` adds velocity, not a lasting force. For a thruster, kick every frame.

Camera fields you may set after `sim_init` / each frame:

```c
s.camx; s.camy; s.camz;   /* metres */
s.yaw;                    /* rad, 0 looks +Z */
s.foc;                    /* ~210 px, larger = zoom */
s.gy;                     /* default -22 */
s.ge; s.gmu;              /* ground bounce / friction */
s.b[id].e; s.b[id].mu;    /* body restitution / friction */
s.b[id].yaw;              /* box heading (rad) */
```

Read a body: `s.b[id].x`, `.y`, `.z`, `.vx`, `.vy`, `.vz`. Write pose if you must (teleport), then let `sim_step` run.

## Get started — smallest game

1. Copy the tree (or just `sim.h phys.c draw.c fb.c fb.h`).
2. Write `game.c` instead of `demo.c`.
3. Point the Makefile `OBJS` at `game.o` instead of `demo.o`.

```c
#include "sim.h"
#include "fb.h"
#include <unistd.h>

int main(void)
{
    Sim s;
    int ball, crate;

    if (fb_open() < 0)
        return 1;
    sim_init(&s);

    ball  = sim_add_sphere(&s, 0, 0.5f, 0, 0.5f, 2.f);
    crate = sim_add_box(&s, 2.f, 0.4f, 3.f, 0.8f, 0.8f, 0.8f, 1.5f);
    (void)crate;

    for (;;) {
        /* read keys yourself; then: */
        sim_kick(&s, ball, 0, 0, 12.f);   /* example: shove +Z */
        sim_step(&s, 1.f / 60.f);
        sim_cam_follow(&s, ball);
        sim_draw(&s);
        usleep(16000);
    }
}
```

Makefile pattern:

```make
game: game.o phys.o draw.o fb.o
	$(CC) -O2 -o game game.o phys.o draw.o fb.o -lm
```

Needs `/dev/fb0`. Run on the deck, not in PuTTY.

## Typical loop

```
input  →  sim_kick / set yaw
sim_step(1/60)
sim_cam_follow(player)   or aim cam yourself
sim_draw
sleep ~16 ms
```

Do **not** call `sim_draw` more than once per painted frame. `sim_step` can be called twice if you ever run at 30 Hz paint / 60 Hz physics (`sim_step(1/60)` × 2).

## Scene recipes

**Floor is free** — you do not add a ground box. `phys.c` already resolves `y - hy < 0`.

**Wall:**

```c
int w = sim_add_box(&s, 6.f, 1.f, 0, 0.4f, 2.f, 12.f, 1.f);
sim_static(&s, w);
```

**Stack:** spawn boxes with `y = hy + i * (2*hy)` so they start barely touching.

**Moving platform:** keep it `sim_static`, write `s.b[id].x` each frame yourself. Static bodies do not integrate, so you are the motor.

**Win / lose:** read `s.b[player].y < -2` (fell off) or distance to a goal sphere.

## What the engine will not do (yet)

No hinges, no scene files, no mesh import, no textures beyond the floor checker, no sleeping islands. 48 dynamic/static bodies. If a frame hitches, cut boxes before you add features.

## Colour

`s.b[id].col` is an int hashed into a small palette. Set it after add if you want teams:

```c
s.b[ball].col = 1;   /* bluish */
s.b[crate].col = 3;  /* yellow */
```
