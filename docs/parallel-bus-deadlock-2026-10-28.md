# Slot 1: side-by-side bus deadlock

Read-only reproduction: slot 1 saved 2026-10-28 10:09:26 -07:00,
SHA-256 `0402bb539332c7630100e5e8e161290ca093fc2b3cb8f043ea4fd4168f9fb675`.
Backup and logs: `dist/saved-bus-deadlock-2026-10-28/`.

The articulated player and AI 15 stopped side by side while heading west.
The player was slightly angled toward AI 15 and needed to wait for it to pass.
AI 15's true straight sweep cleared both sections of the player, but the two
extra 1px prediction skins touched at the player's yawed nose. AI 15 therefore
also stopped, preventing the player from completing its return to the lane.
The unmodified read-only replay remained immobile for 60 seconds and failed.

`controlPaddingRelease` now permits the leading bus to clear at no more than
35 px/s (~6.3 km/h) when the only issue is its extra sweep padding beside a
stationary rear bus. Eligibility requires same direction, a powered-section
lead of at least 8px, lateral separation, near-straight steering and low speed.
Every actuator prediction is checked again against the actual two-section
geometry with a 0.25px skin on both vehicles, starting at the current pose.
Real intersections of bodies still stop the bus. Other obstacles, headway,
signals, road boundaries, junction checks and articulation checks are retained.
The rear bus does not receive the same release. The 45px/s eligibility ceiling
allows a small control overshoot around the 35px/s cap without stop/start
oscillation; this is not a high-speed passing exemption.

Validation:

- Four headings, all three lead models, ordinary/articulated rear models.
- Continuing low-speed clearance, rear exclusion, true side contact, another
  front blocker, cars, moving neighbors, opposing traffic, high speed, steering.
- Existing control-sweep and prediction-equivalence tests passed.
- Focused race tests passed.
- Read-only saved replay: seeds 5/17/29 each ran 60 seconds. AI 15 traveled its
  first 20px by 0.68s, the player by 2.68s; original target served by 5.50s.
  Neither vehicle lost durability or teleported. Player continued >2000px,
  AI 15 >500px. Separate 90-second replay passed with player health still 100.

No vehicles are deleted or moved to artificial positions. Save schema and
route geometry are unchanged. The user's original save remains byte-identical
to the backup. No Git commit or GitHub push is part of this change.

Final checks: full suite 10,227 cases passed, 25 skipped; `go vet` and
`git diff --check` passed. The three-seed replay was repeated with per-tick
physical body overlap assertions (including the trailer): all passed.
Rebuilt `busgame` and `2D Bus Simulator.app/Contents/MacOS/busgame` both have
SHA-256 `861f1532ceba62b806869d66b34f91a73a5b6ac295d30cbc0cb6b69ffbda047e`.
