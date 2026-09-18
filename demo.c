#include "sim.h"
#include "fb.h"
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static struct termios oldt;
static int raw, player;

static void io_open(void)
{
    struct termios t;
    tcgetattr(0, &oldt);
    t = oldt;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &t);
    raw = 1;
}
static void io_close(void) { if (raw) tcsetattr(0, TCSANOW, &oldt); }

static void scene(Sim *s)
{
    int i, j, post, sign;
    sim_init(s);
    player = sim_add_sphere(s, 0, 0.5f, 0, 0.45f, 2.f);
    s->b[player].e = 0.55f;
    s->b[player].mu = 0.35f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            int id = sim_add_box(s, 1.5f + (j % 2) * 0.04f, 0.35f + i * 0.72f,
                                 1.3f + j * 0.04f, 0.7f, 0.7f, 0.7f, 1.4f);
            s->b[id].e = 0.12f;
        }
    sim_add_sphere(s, 2.8f, 0.35f, 2.2f, 0.35f, 0.7f);
    post = sim_add_box(s, -2.4f, 1.2f, 2.5f, 0.25f, 2.4f, 0.25f, 8.f);
    sim_static(s, post);
    sign = sim_add_box(s, -1.4f, 1.6f, 2.5f, 0.7f, 0.15f, 0.5f, 0.6f);
    sim_joint_hinge(s, post, sign, -2.0f, 1.6f, 2.5f);
    /* hanging ball on a distance joint */
    {
        int hook = sim_add_sphere(s, 3.5f, 2.8f, 0.5f, 0.12f, 1.f);
        int bob = sim_add_sphere(s, 3.5f, 1.2f, 0.5f, 0.28f, 0.5f);
        sim_static(s, hook);
        sim_joint_dist(s, hook, bob, 1.6f);
        s->b[bob].e = 0.7f;
    }
}

int main(void)
{
    Sim s;
    int run = 1;
    if (fb_open() < 0) {
        fprintf(stderr, "cSim2D needs /dev/fb0\n");
        return 1;
    }
    scene(&s);
    io_open();
    while (run) {
        unsigned char b[16];
        int n = (int)read(0, b, sizeof b), i;
        float fx = 0, fz = 0, fy = 0, cy = cosf(s.yaw), sy = sinf(s.yaw);
        for (i = 0; i < n; i++) {
            unsigned char c = b[i];
            if (c == 'q' || c == 'Q') s.yaw -= 0.12f;
            else if (c == 'e' || c == 'E') s.yaw += 0.12f;
            else if (c == 'a' || c == 'A' || (c == 0x1b && i + 2 < n && b[i + 2] == 'D')) fx -= 1;
            else if (c == 'd' || c == 'D' || (c == 0x1b && i + 2 < n && b[i + 2] == 'C')) fx += 1;
            else if (c == 'w' || c == 'W' || (c == 0x1b && i + 2 < n && b[i + 2] == 'A')) fz += 1;
            else if (c == 's' || c == 'S' || (c == 0x1b && i + 2 < n && b[i + 2] == 'B')) fz -= 1;
            else if (c == ' ') fy = 1;
            else if (c == '+' || c == '=') { s.foc *= 1.12f; if (s.foc > 420) s.foc = 420; }
            else if (c == '-' || c == '_') { s.foc /= 1.12f; if (s.foc < 90) s.foc = 90; }
            else if (c == 'r' || c == 'R') scene(&s);
            else if (c == 'x' || c == 'X') run = 0;
            if (c == 0x1b && i + 2 < n) i += 2;
        }
        if (fx || fz || fy) {
            float kx = (fx * cy + fz * sy) * 18.f;
            float kz = (-fx * sy + fz * cy) * 18.f;
            sim_kick(&s, player, kx, fy * 28.f, kz);
        }
        sim_step(&s, 1.f / 60.f);
        sim_cam_follow(&s, player);
        sim_draw(&s);
        usleep(16000);
    }
    io_close();
    fb_close();
    return 0;
}
