# Track Notes — the three maps

Photos: `images/map-3pt.png`, `images/map-5pt.png`, `images/map-9pt.png`.
All tracks share: black-painted smooth wooden walls, white floor inside the
track, **30.5 cm wall-to-wall corridors**, 90° corners, striped ("zebra")
pads at track entries/exits, a hand-drawn arrow on the floor marking the
start direction, thin black lines across the floor marking segment
boundaries (presumably the scoring "points": 3 / 5 / 9 segments).

Common derived numbers:

| Quantity | Value |
|---|---|
| Corridor width | 30.5 cm |
| Robot width | 15.7 cm → 7.4 cm nominal clearance/side |
| Centered readings | LEFT ≈ 8.65 cm, RIGHT ≈ 9.50 cm (offset 0.85 cm) |
| "Wall present" classification | side reading < ~14 cm |
| "Opening" classification | side reading > ~20 cm or echo timeout |
| Corner box (junction area) | 30.5 × 30.5 cm; centered robot has ~15.25 cm to each wall — pivot sweep must stay under that (need axle position to confirm) |

## CONFIRMED layouts (organisers' spreadsheet, 2026-08-19)

The team supplied the official map drawings
(`Robotics_Competition_Maps_v2_5x5_grid.xlsx`, vendored in this folder):
5×5 grid of 1 ft (30.5 cm) cells, wall thickness 0.75 in, wall height
7.5 in. `sim/track.py` now builds all three maps cell-by-cell from these
drawings. The photo-based reconstructions below are superseded.

- **Map 1 — 3 sectors**: U ring. Up the west column (5 cells), east along
  the top row, down the east column. Entrance and exit on the south wall.
  Script: `[F:R, F:R]` — two right corners.
- **Map 2 — 5 sectors**: top row east → down the east leg **straight
  through the 4-way crossing** → around the lower loop (3 right corners)
  → back into the crossing **straight again**, exiting through its east
  arm. Script: `[F:R, X, F:R, F:R, F:R, X]`. This matches the team's
  "go forward at the intersection" requirement exactly — the robot never
  pivots inside the crossing.
- **Map 3 — 9 sectors**: a pure serpentine — nine straight segments joined
  by eight plain 90° corners, no branches, no T-junctions. Script:
  `[F:R, F:R, F:L, F:L, F:L, F:L, F:R, F:R]`.

**Design consequence (excellent news):** no map requires a side-opening
('O') turn — the least robust maneuver class. Every junction in the whole
competition is either a plain blocked-front corner or map 2's crossing,
which are the two maneuvers the controller executes most reliably.

Historical photo-based notes (superseded, kept for reference):

## Map 1 — "3 points" (`images/map-3pt.png`)

What the photo shows: an outer ∩-shaped wall, a large solid block in the
middle, track = the gap between them. Both corridor mouths open toward the
photographer (blue floor); zebra pads at both mouths; the start arrow is in
the **left** corridor pointing into the track.

**Route hypothesis (confirm):** enter left corridor → 90° **right** at the
top-left corner → traverse top corridor → 90° **right** at top-right →
exit down the right corridor over the finish pad.
Script: `[R, R]`. Simplest map: two decision points, no intersections.

## Map 2 — "5 points" (`images/map-5pt.png`)

What the photo shows: roughly square outer wall; start at bottom-left
(arrow up, zebra pad); a corridor branch exits through the **top** wall to
a zebra pad (likely the finish); a U/L-shaped inner wall structure creating
a nested corridor on the left/center; a small square block obstacle
center-right; segment lines dividing the floor into ~5 segments. This is
the map the team flagged: it contains **the intersection where the robot
must go straight through**.

**Route hypothesis (confirm & correct on the photo):** start bottom-left
heading up → follow the corridor system around the inner structure
(covering all 5 segments) → at the 4-way/T intersection under the top
branch, **go straight** per the team's instruction on the pass(es) that
must continue, and take the branch **up to the top zebra pad** at the end.
I can't pin the exact turn sequence from one photo — this is question #1
for the team; the firmware just takes a script like `[R, S, L, L, S, R]`
so any confirmed sequence drops straight in.

Supporting evidence: the prior firmware for this competition
(`Muzoka/Robotic-comp`, `docs/RESEARCH.md` §0) describes its map 2 — the
one with the open 4-way crossing — as "straight down the middle to the
bottom row, round the lower loop, back along the middle and out", crossing
the open junction **twice, straight through both times, never pivoting in
it**. If our 5-point map is the same layout, that is the route and
"straight-first" handles the intersection with no special casing.

## Map 3 — "9 points" (`images/map-9pt.png`)

What the photo shows: the most complex map, an H-like layout: two vertical
channels on the left (each ending in its own zebra pad — one top-left, one
bottom-left where the start arrow is), a horizontal middle corridor
connecting them, and on the right a large rectangular ring corridor around
an inner block. ~9 floor segments.

**Route hypothesis (confirm & correct on the photo):** start at the
bottom-left pad heading up → into the middle/right system → all the way
around the right-hand ring → back along the middle → up the second left
channel → finish on the top-left pad. Again, the exact sequence is a
script constant once the team confirms it.

## Finish detection (no floor sensors on this robot)

The zebra pads are visual floor markings — our sensor suite can't see them.
Finish will be executed as: after the **last scripted decision point**,
drive straight while corridor walls remain, and when the walls open
(track mouth) continue a fixed distance (~1.5 robot lengths, timed at
calibrated cruise speed) to fully clear the track, then stop. Clean,
observable, and no extra hardware.

## Segment boundary lines

The thin black floor lines are ignored by the robot (no floor sensor) and
are assumed to be for human scoring only. If the rules require a *stop* on
any of them, tell me — that changes the route script format (we'd add
timed checkpoint stops).

## To make the simulator exact (team, when convenient)

1. Confirm/redraw the three routes on the photos (any annotation app, or
   describe turn-by-turn: "up, right, right, …").
2. One reference length per map (e.g. inner length of the 3-point map's top
   corridor) — everything else scales from the 30.5 cm module.
3. Wall height (for checking the front sensor's cone never sees over a wall).
