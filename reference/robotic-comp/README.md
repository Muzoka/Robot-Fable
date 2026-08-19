# Vendored reference: Muzoka/Robotic-comp

Copies of three files fetched from the public repository
https://github.com/Muzoka/Robotic-comp — a prior firmware + simulator for
what appears to be this same competition (305 mm corridor grid, three maps,
open 4-way crossing, striped start gate). Vendored here for reference while
designing the Robot-Fable firmware; see `docs/RESEARCH.md` §0 for what
transfers and what does not (that robot had a gyro + wheel encoders; ours
does not).

- `config.h` — all tunable constants, with measured tuning-sweep notes
- `nav_core.cpp` — state machine / navigation core
- `wokwi_sketch.ino` — self-contained Wokwi simulation sketch
