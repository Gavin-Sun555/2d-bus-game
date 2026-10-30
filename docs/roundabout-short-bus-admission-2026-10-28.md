# Short-bus entry alongside an articulated ring reservation

Read-only slot 1 backup: `dist/ring-entry-gap-2026-10-28/slot1-original.json`,
saved 2026-10-28 10:37:48 -07:00. The player is departing the west side of the
central roundabout. AI 14 waits at the east entrance despite every normal
controller speed limit allowing movement.

The blocker was `RoundaboutAdmissionHold`: the long-vehicle reservation held
all entrances until the owner's last section left the entire roundabout. The
original behavior is retained for cars, further articulated entrants, unknown
or cross-lane entry trajectories, and before the owner is admitted/draining.

Once the owner is clearly driving outward through its exit, an ordinary/double-decker bus can use an entrance
whose full local join is separated from both owner sections. The new check
classifies the actual aligned incoming lane, checks the current body and 49
poses along its complete join, using 10px length/8px width margins on the
entrant and 4px margins on both owner sections. The owner must be moving above
5px/s, with its powered section within one body width of the outer edge (or
beyond) and pointing outward with radial alignment >0.5. Circulating and
stationary owners retain the original protection. A trailer still across this
mouth retains the hold. Invalid geometry retains the hold. Removing this
administrative hold does not bypass any ordinary dynamic priority, following,
steering, boundary or swept collision check. The ring remains reserved for
cars and further long entrants until its original release condition.

Validation evidence in `dist/ring-entry-gap-2026-10-28/`:

- Original replay: AI 14 entered at 3.40s, only after the owner fully departed.
- Updated replay, seeds 5/17/29: AI 14 starts on the first tick, travels 20px by
  0.40s while the owner still occupies the ring, enters by 2.03s and exits by
  10.88s. Each replay continues for 30s with no damage, body intersection or
  teleportation. Entry time improved by 1.37s (~40%).
- Four approach headings, three lane widths, ordinary/double-decker models.
- Dedicated trailer-only obstruction (front alone is clear), occupied mouth,
  unknown heading, another articulated entrant, and not-yet-admitted owner.
- Existing owner-selection, draining, complete-tail reservation, car hold,
  following and entry-priority tests retained.

No save-format changes or manual state repairs. The user's save is read-only;
all replays use the backup. No Git commit or push is included.

The first experiment allowed this release while the owner was still circulating.
Its optional metropolis soak encountered a car contact after 90.05s on an
ordinary junction, while the matching pre-change copy passed. That broader
experiment is discarded (`soak.log`, `baseline-soak-map0.log`). The final exit-
only handoff keeps the saved-scene improvement and passes both 120s mixed-fleet
soaks: metropolis and custom map, each initially 864 cars, player and every AI
bus health 100, continuous progress and save/load at 60s. The custom map ends
with 489 active cars as boundary traffic drains; it is not a constant 864-car
occupancy test. Evidence: `soak-final.log`.

Final full suite: 10256 passed, 26 skipped, no failures.
Focused race checks and go vet passed. Rebuilt executables share SHA-256
`375afb0fda750136d2bbe7555151ceabf6ac571f141629646ad830e2067227c5`.
