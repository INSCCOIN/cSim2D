CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra
OBJS = demo.o phys.o draw.o fb.o scene.o

cSim2D: $(OBJS)
	$(CC) $(CFLAGS) -o cSim2D $(OBJS) -lm

clean:
	rm -f cSim2D $(OBJS)
