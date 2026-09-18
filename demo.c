#include "sim.h"
#include "fb.h"
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
static struct termios oldt;
static int raw, player;
static void io_open(void){struct termios t;tcgetattr(0,&oldt);t=oldt;t.c_lflag&=~(ICANON|ECHO);t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;tcsetattr(0,TCSANOW,&t);raw=1;}
static void io_close(void){if(raw)tcsetattr(0,TCSANOW,&oldt);}
static void scene(Sim *s){
  int i,j; sim_init(s);
  player=sim_add_sphere(s,0,0.5f,0,0.45f,2.f);
  for(i=0;i<3;i++) for(j=0;j<3;j++)
    sim_add_box(s,1.4f+(j%2)*0.05f,0.35f+i*0.72f,1.2f+j*0.05f,0.7f,0.7f,0.7f,1.4f);
  sim_add_sphere(s,2.8f,0.35f,2.0f,0.35f,0.7f);
  sim_add_box(s,-2.f,0.4f,3.f,1.2f,0.8f,0.8f,3.f);
}
int main(void){
  Sim s; int run=1;
  if(fb_open()<0){fprintf(stderr,"cSim2D needs /dev/fb0\\n");return 1;}
  scene(&s); io_open();
  while(run){
    unsigned char b[16]; int n=(int)read(0,b,sizeof b),i;
    float fx=0,fz=0,fy=0, cy=cosf(s.yaw), sy=sinf(s.yaw);
    for(i=0;i<n;i++){
      unsigned char c=b[i];
      if(c=="q"[0]||c=="Q"[0]) s.yaw-=0.12f;
      else if(c=="e"[0]||c=="E"[0]) s.yaw+=0.12f;
      else if(c=="a"[0]||c=="A"[0]||(c==0x1b&&i+2<n&&b[i+2]=="D"[0])) fx-=1;
      else if(c=="d"[0]||c=="D"[0]||(c==0x1b&&i+2<n&&b[i+2]=="C"[0])) fx+=1;
      else if(c=="w"[0]||c=="W"[0]||(c==0x1b&&i+2<n&&b[i+2]=="A"[0])) fz+=1;
      else if(c=="s"[0]||c=="S"[0]||(c==0x1b&&i+2<n&&b[i+2]=="B"[0])) fz-=1;
      else if(c==32) fy=1;
      else if(c=="r"[0]||c=="R"[0]) scene(&s);
      else if(c=="x"[0]||c=="X"[0]) run=0;
      if(c==0x1b&&i+2<n) i+=2;
    }
    if(fx||fz||fy){
      float kx=(fx*cy+fz*sy)*18.f, kz=(-fx*sy+fz*cy)*18.f;
      sim_kick(&s,player,kx,fy*28.f,kz);
    }
    sim_step(&s,1.f/60.f); sim_cam_follow(&s,player); sim_draw(&s); usleep(16000);
  }
  io_close(); fb_close(); return 0;
}
