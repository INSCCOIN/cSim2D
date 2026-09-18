#include "sim.h"
#include "fb.h"
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

static void io_close(void)
{
    if (raw)
        tcsetattr(0, TCSANOW, &oldt);
}

static void scene(Sim *s)
{
    int i, id;
    sim_init(s);
    id = sim_add_box(s, 0, -0.4f, 28.f, 0.8f, 1000.f);
    sim_static(s, id);
    player = sim_add_circle(s, -4.f, 1.2f, 0.45f, 2.f);
    for (i = 0; i < 6; i++)
        sim_add_box(s, 1.2f + (i % 2) * 0.15f, 0.5f + i * 0.55f, 0.7f, 0.5f, 1.2f);
    sim_add_circle(s, 3.5f, 1.0f, 0.35f, 0.8f);
    sim_add_circle(s, 4.2f, 1.4f, 0.28f, 0.5f);
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
        for (i = 0; i < n; i++) {
            unsigned char c = b[i];
            if (c == 'q' || c == 'Q')
                run = 0;
            else if (c == '+' || c == '=')
                s.zoom *= 1.1f;
            else if (c == '-')
                s.zoom /= 1.1f;
            else if (c == 'r' || c == 'R')
                scene(&s);
            else if (c == 0x1b && i + 2 < n && b[i + 1] == '[') {
                if (b[i + 2] == 'C')
                    sim_kick(&s, player, 14.f, 2.f);
                if (b[i + 2] == 'D')
                    sim_kick(&s, player, -14.f, 2.f);
                if (b[i + 2] == 'A')
                    sim_kick(&s, player, 0, 22.f);
                if (b[i + 2] == 'B')
                    sim_kick(&s, player, 0, -8.f);
                i += 2;
            } else if (c == 'a' || c == 'A')
                sim_kick(&s, player, -14.f, 2.f);
            else if (c == 'd' || c == 'D')
                sim_kick(&s, player, 14.f, 2.f);
            else if (c == 'w' || c == 'W')
                sim_kick(&s, player, 0, 22.f);
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
