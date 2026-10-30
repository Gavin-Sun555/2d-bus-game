# Block 279 Northbound queue recovery

Read-only source: slot 1, saved 2026-10-28 10:44:53 -07:00. Backup and evidence:
`dist/block279-yield-2026-10-28/`. Display name Block 279 Northbound is internal
stop ID 278. The initially affected AI IDs are 1, 2, 6 and 7, all articulated.

The front bus retained a waiting curve ending less than 2px ahead while still
being yawed. A free front berth offered another 45px of straightening room,
but the recovery required more than half a powered-section length (48px).
It therefore never reconsidered that berth. Following buses also treated the
leader's complete combined, rotated envelope as an oversized berth reservation.
Projecting that envelope again onto the station axis invented additional
longitudinal reach, so even physically separated followers could block the lead.

Changes:

- Replace the fixed recovery-runway threshold with an explicit steering and
  full swept-clearance proof of the available front-berth alignment curve.
- For yawed articulated arrivals in an actual bus-bay queue, use a steerable
  alignment curve instead of retaining an unsteerable short cubic. If there is
  no feasible candidate, discard it; the existing valid-bay service fallback
  can then open the doors at an already safe platform pose. Moving free-bay
  arrivals retain their calibrated taper. Nearby passing cars do not turn a
  free arrival into a bus-queue recovery.
- Classify behind/ahead berth occupancy using the min/max projections of each
  real vehicle section, retaining a 2px longitudinal gap. Full articulation,
  actual collisions, dynamic traffic, boundaries and signal checks remain.

Development checks exposed two incomplete variants: freeing the first bus
alone left AI 2 stuck; applying the new alignment to moving free-bay arrivals
broke the shelter regression. The final variant specifically handles actual
bus queues/stopped corrections and retains free-bay behavior.

Validation:

- Embedded minimal four-bus scene, both fleet iteration orders: all four serve
  stop 278 and make progress, with no physical overlap, boundary contact or
  position jumps across 120 simulated seconds.
- Short-runway recovery and four-direction complete-section projection tests,
  including a true overlapping follower which must retain its reservation.
- Original free-bay/shelter arrival tests, plus moving arrivals beside an
  adjacent car in ordinary and berserk modes.
- Whole saved scene, seed 5: at 120s all four originally blocked AI buses and
  the player remain undamaged. AI 1 completes service at 4.08s, AI 2 at 13.20s,
  AI 7 at 72.67s, AI 6 at 104.88s. Later delays include real red-light and
  passenger-platform queues; the station no longer remains permanently locked.
- The same unrestricted queue-recovery trajectory was also checked with seeds
  17 and 29 while refining the moving free-bay guard; final evidence uses the
  final guarded source, not those intermediate runs.

No vehicles are removed or relocated. No save schema/path-revision change is
needed: existing waiting plans are reconsidered on load. The real save is
read-only; all tests use the copied file or temporary test fixtures. No commit
or push is included.

Final extended replay, final guarded source (`replay-station-qualified.log`):
180s completed, all four original buses both served stop 278 and then traveled
at least another 300px. Total travel: AI1 12,276.9px, AI2 6,643.8px, AI6
1,502.4px, AI7 4,398.6px. The original station zone remains undamaged.
The extended test strictly rejects contacts within 700px of the original stop
but records later off-site incidents separately. One player/car contact occurred
at 120.98s, position (9964.53,-48.52), roughly 6,000px from the platform; player
health 100->97. This later-road issue is recorded, not claimed fixed by this
station change. The initial 120s saved replay remained fully undamaged.

Final station regression matrix, focused race tests and go vet passed. Both
120s optional mixed-fleet soaks passed (metropolis and custom map), each with
864 initial cars, save/load at60s and all bus health100 (`soak-v7.log`). Normal
traffic/signal queues in the soak remain; passing the soak is not a claim that
all possible traffic jams have been eliminated. Executable and app binary
SHA-256: `b5fdbfd97d5b126d5a34e608c7a9acc3060efb2b618dfb04f1ccf0ff64cc6ba8`.

Final all-package short run: 9,928 test cases passed, 33 skipped, no failures
(`tests-final-short.jsonl`). The earlier non-short intermediate run's only
failure was the new saved-queue reproduction; final queue, station, shelter and
passing-car cases were revalidated after the guard was corrected. The final
result is not labeled a fresh full non-short release qualification.
