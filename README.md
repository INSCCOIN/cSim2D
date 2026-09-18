# cSim2D

2-D rigid body engine with a 2.5-D painter. Games should `#include "sim.h"` and link `phys.o draw.o fb.o`. This tree ships a crate-stack witness in `demo.c`.

SI units. Semi-implicit step, circle/AABB contacts, friction. Camera is oblique: X across, Y up, a little depth squash.

```bash
rm -f *.o cSim2D
make
./cSim2D
```

| | |
|--|--|
| ← → / A D | kick player |
| ↑ / W | hop |
| + - | zoom |
| R | reset scene |
| Q | quit |

Not a product renderer. 64 bodies is the cap.
