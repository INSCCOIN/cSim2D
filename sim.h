#ifndef CSIM2D_H
#define CSIM2D_H

enum { SIM_MAX = 96, SIM_JMAX = 32, SIM_SPHERE = 0, SIM_BOX = 1 };
enum { J_DIST = 0, J_HINGE = 1, J_SLIDE = 2 };

typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float wy, wp, wr;
    float yaw, pitch, roll;
    float px, py, pz;
    float m, im, I, iI;
    float r, hx, hy, hz;
    float e, mu;
    unsigned layer, mask;
    int shape, static_, col, sleep, awake;
} SimBody;

typedef struct {
    int type, a, b;
    float rest;
    float lax, laz, lbx, lbz;
    float ax, ay, az;
} SimJoint;

typedef struct {
    float gx, gy, gz;
    float ge, gmu;
    float camx, camy, camz, yaw, foc;
    SimBody b[SIM_MAX];
    SimJoint j[SIM_JMAX];
    int n, nj;
    int debug, ncontact;
    float fps;
} Sim;

void sim_init(Sim *s);
int  sim_add_sphere(Sim *s, float x, float y, float z, float r, float m);
int  sim_add_box(Sim *s, float x, float y, float z, float sx, float sy, float sz, float m);
void sim_static(Sim *s, int id);
void sim_kick(Sim *s, int id, float fx, float fy, float fz);
int  sim_joint_dist(Sim *s, int a, int b, float rest);
int  sim_joint_hinge(Sim *s, int a, int b, float wx, float wy, float wz);
int  sim_joint_slide(Sim *s, int a, int b, float ax, float ay, float az);
void sim_step(Sim *s, float dt);
void sim_draw(const Sim *s);
void sim_cam_follow(Sim *s, int id);
int  sim_ray(const Sim *s, float ox, float oy, float oz,
             float dx, float dy, float dz, float maxd, float *hit);
int  sim_query(const Sim *s, float x, float y, float z, float r, int *out, int max);
int  sim_load(Sim *s, const char *path);

#endif
