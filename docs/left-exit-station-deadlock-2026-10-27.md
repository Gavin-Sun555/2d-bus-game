# Slot 1 left-turn exit / station queue deadlock (2026-10-27)

## Reproduction

Original slot 1: saved 2026-10-27 17:09:35 -07:00; SHA-256
`e58c8bfdc76a1ee8da78181fba0b057e4857034a7b456d39bd27b5738c79bbc1`.
Read-only backup and diagnostic outputs are under ignored
`dist/left-exit-jam-2026-10-27/`. The user save is never overwritten.

The scene contains 16 AI buses and 793 cars. AI 2, 4, 9 and 11 are queued
at the arrival to stop 278 after the left turn. In the released code they
travel zero pixels in a 60-second replay, including with every car removed.
This is a bus/controller deadlock rather than congestion caused by cars.

## Causes and local changes

- The road-lane return overlaps the long station arrival taper. Their join
  contains a 94 px backwards segment. Join the overlapping maneuvers on a
  sufficiently long forward curve outside the junction instead. Preserve
  the turn and its separated exit lane. Old saved paths receive the same
  correction only when the changed section is still ahead of the vehicle.
- A bus already occupying a valid, separated platform position can reject
  its final tiny local alignment curve and brake forever. Permit service at
  that actual position when stopped, inside the real bay, with a free berth
  and clear full bodies. Do not move the bus or skip passenger service.
- A stopped, angled follower at the rear entrance can reserve the lead
  bus’s whole platform. Treat it as a follower only when heading and complete
  longitudinal envelopes permit; actual berth collision checks still apply.
- For large articulated lateral arrivals, check the powered section’s
  curvature as well as tail articulation. If its short local cubic is too
  tight, try a full-run curve matching the starting nose direction and final
  road direction. Preserve the existing small berth-alignment corrections.

Collision sweeps, road boundaries, signals and ordinary traffic remain active.
No vehicles are deleted, teleported or given a collision exemption.

## Validation

- `go test -json ./... -count=1 -timeout=30m`: **10,156 passed, 24 skipped**.
- Focused `-race` suite: **36 passed**.
- Original full-save replay: **120 simulated seconds**, all 793 cars retained.
  AI 2/4/9/11 each served stop 278 and resumed travel; distances were
  9,891.6 / 4,705.0 / 10,322.8 / 4,691.6 px respectively. All 16 AI buses
  moved at least 618.4 px. No player or AI bus durability decreased.
- `go vet ./...` and `git diff --check`: passed.
- Long mixed-traffic matrix: **21/21 scenarios passed** across six maps,
  normal/berserk modes and mixed articulated fleets, totaling 42 simulated
  minutes. Safety, progress and mid-run save/load assertions remain enabled.
- `./build.sh app`: passed; root executable and app-bundle executable match
  SHA-256 `b2da657aec18ec8648ec8c5b918db24cfac08439f9e31497643046943c077b82`.

The full-save replay runs the real game coordinator and physics at a fixed
simulation tick; it is a traffic/safety check, not a native FPS benchmark.

The focused regression includes the four captured buses, both vehicle update
orders, a second save/restore during recovery, actual actuator integration,
passenger service and continued travel. It rejects body collisions, position
jumps and hard boundaries. Additional cases exercise four headings, lane
widths 42/52/78, stopped/moving/crossing/overlapping rear traffic, blocked
front space, roadside poses and the previous free-bay shelter regression.

## Delivery

Local fix only. The existing v0.0.7 GitHub release and tag are unchanged.
The rebuilt native application is `2D Bus Simulator.app` in the repository.
