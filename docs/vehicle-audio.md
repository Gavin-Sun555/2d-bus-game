# Vehicle sound

Driving: hold **H** to sound the horn continuously; release H to stop. H retains
its hide-other-routes action in planning. **F8** toggles sound for the current
session. This preference survives loading a save or changing maps. Sound is
silent in menus, planning, paused panels, and while the game lacks focus.

The player's bus has synthesized diesel idle, throttle/RPM response, virtual
gear changes and rolling noise. All three current bus models default to the
same diesel sound. Nearby AI buses and cars do not create additional voices.
The horn has no interaction with collision detection, priority or traffic AI.
These are original synthesized effects, not diesel field recordings.

## Adding a model's sound

After `g.EnableAudio()`, call:

```go
g.SetVehicleSoundProfile(sim.BusArticulated, sound.Profile{
    EngineLoop: engineSamples,
    Horn: hornSamples,
    IdleRPM: 600,
    MaxRPM: 2350,
})
```

Import `internal/sound`. Samples are immutable mono float32 PCM in [-1,1], at
`sound.SampleRate` (24 kHz). `EngineLoop` must be seamless at `IdleRPM`; `Horn`
is a seamless loop. The mixer applies a 15ms attack and 40ms release so long
holds do not have periodic gaps and releasing does not click. Both must contain at least two
samples; RPMs must be finite, positive, and ordered. Registration returns false
for invalid banks. Keep slices immutable after registration. Future recorded
banks can be decoded/resampled at loading time; the mixer does not require a
new physics or save field. Unassigned models use `sound.Diesel()`.

## Runtime and verification

Only the native entry point enables audio. Replays and regular tests never
open a device. The simulation copies speed/throttle/active flags into a mixer;
the audio callback holds no vehicle pointers and uses its own PRNG. One stereo
float32 stream mixes three bounded voices. PCM output is independent of reader
chunk sizes and does not allocate. Native buffer size is 60 ms. Menu/mute gain
fades in/out rather than cutting a waveform. The current in-flight horn is
cancelled on pause; sound banks and session mute state are preserved on load.

Tests cover sustained holds across loop boundaries, release, rapid re-press,
per-tick update invariance, pause/resume silence, finite
bounded stereo PCM, chunk invariance, invalid/future profiles, throttle response,
concurrent update/read under the race detector, zero streaming allocations and
save/map ownership. A native smoke test and an eight-second preview are in
`dist/vehicle-audio-2026-10-28/` (ignored build evidence).

2026-10-28 validation: full suite 10,194 test cases passed, 24 skipped;
focused game/audio race tests and `go vet ./game ./internal/sound` passed.
After the final pause-fade adjustment, sound race tests were repeated and native
playback advanced to 1.92 s in the two-second hardware smoke. Streaming a 1/60 s
block measured 2,858 ns/op, 0 B/op, 0 allocs/op on Apple M4 (mixing only, not
backend/device overhead). Both rebuilt executables have SHA-256
`a97449b3b917cdb149e0ec608e2e8f0a2c685be76566c3c2690c2f532eed7653`.

2026-10-28 follow-up: H now uses held-key input and a seamless sustained horn
loop with mixer-controlled attack/release. Focused audio/save tests passed under
`-race`; `go vet` and native playback smoke passed. Continuous horn mixing is
4,073 ns per 1/60 s block on M4, with zero streaming allocations. The app was
rebuilt. Evidence: `dist/horn-hold-2026-10-28/`.

## Bus announcements (2026-10-28)

On macOS, Settings → Bus announcements cycles between game-language speech, Off, and Chinese + English. The preference is included in progress saves; older saves default to game-language speech. Only the player's selected bus speaks. F8 mutes announcements along with the vehicle sounds. Planning, menus, an unfocused window, loading and changing maps cancel speech.

Next-stop announcements now use a speed-dependent lookahead: 12 seconds of current-speed travel for one language, 20 seconds for bilingual speech, with minimum distances of 1800/2600 world pixels (90/130 game meters) and a maximum of 10000 pixels. Autopilot uses remaining distance along its current target leg, allowing announcements before a turn; manual driving still requires a matching road and heading. Stale path progress is rejected. Opening doors for passenger exchange announces arrival followed by the line welcome. Closing doors and moving above 12 px/s triggers the handrail reminder. Repeated simulation frames do not repeat a message. In bilingual mode each event speaks Chinese then English. Every new event cancels the entire previous event, including unspoken language segments.

Chinese and English station names are read independently from local custom names, falling back to the original language-specific map names. Route numbers use the selected player line. The system uses installed macOS voices through `/usr/bin/say` (prefers Tingting and Samantha, with same-language local fallbacks); there is no network service or downloaded recording. Voice playback runs outside the simulation thread, text is supplied on stdin, and no shell interprets station names. Other operating systems currently report the local speech backend as unavailable; existing engine/horn audio remains available.

Validation: all-package short suite, focused event/persistence tests, race checks, and vet passed. Native Chinese/English synthesis produced valid audio; an opt-in real `say` smoke test verified both language processes stop within the 500 ms cancellation check. The native test is `BUSGAME_NATIVE_SPEECH_TEST=1 go test ./internal/sound -run TestNativeAnnouncementSpeech -count=1`. Settings layouts were rendered in both languages at 800/1280 logical width. Evidence is in ignored `dist/announcements-2026-10-28/`. The macOS application was rebuilt.

## Voice selection and running-noise tuning (2026-10-28)

Settings now offers four paired voice presets: Female 1 (Tingting/Samantha), Male 1 (Reed), Female 2 (Shelley), and Male 2 (Eddy). Each uses the matching installed Mandarin and US English voice. Qualified macOS voice names are matched by their base name and language; fallback preferences stay within the selected voice family. Voice choices are saved with progress. Switching voice cancels speech using the old voice.

Departure text is exactly:

- Chinese: 车辆起步，请抓好扶手，注意安全。
- English: Bus starts, please hold on to the handrails and mind your step.

The native game plays both engine and road sound. The diesel bank uses broad low-frequency firing harmonics instead of narrow combustion spikes and metallic high-frequency texture. Engine RPM changes are smoothed, and two low-pass stages soften the running bed; road noise is quieter and separately smoothed. The held H horn bypasses these running-audio filters and retains its original timbre. F8 still mutes vehicle sound and announcements.

Validation: voice selection, bilingual routing, save/load, manual horn hold/release and race tests pass. An opt-in native test exercises all four presets in both languages and verifies cancellation. Updated settings screenshots and logs are in ignored `dist/audio-voices-2026-10-28/`.


### Earlier next-stop reports

Departure reminders are evaluated before next-stop reports. A new next-stop report clears a pending delayed departure reminder, so beginning to accelerate after closing the doors cannot overwrite that report. Arrival still immediately replaces ongoing speech. Regressions cover early bilingual reports, deduplication, wrong-way rejection, remaining-path checks before turns and delayed departure priority. Evidence: `dist/early-announcements-2026-10-28/`.


### Restored, softened engine and road sound

The running bed remains audible at idle and while driving; it is not removed or permanently muted. Idle/acceleration/cruise comparison WAVs and frequency analysis are in ignored `dist/soft-running-audio-2026-10-28/`. In the identical-input cruise comparison, the fraction of energy above 800 Hz falls from about 0.80% to 0.00063%, while RMS remains nonzero (0.0666 → 0.0453). This measures a tonal change in addition to the moderate level reduction; it does not replace listening evaluation on the user's speakers.

Audio regressions verify nonzero, bounded running levels at 0/40/80/120 km/h, gentle adjacent-sample changes, unchanged manual-horn signal, hold/release, pause/mute, profile switching, zero-allocation streaming and audio/game-thread race safety. A 12-second preview contains idle, acceleration, cruise and deceleration.

### Independent volumes and repeated horn taps

Settings offers separate engine/road, announcement, and horn volumes, from 0% to 100% in 10% steps. Zero mutes that channel. Preferences are saved with progress; missing fields in older saves retain the original full-volume mix. F8 still mutes all three. Speech gain uses the macOS embedded volume command, without changing system volume; station names cannot insert their own embedded commands.

The mixer retains each rising horn-key edge even when several game updates arrive before an audio callback. Each tap has at least 80 ms of tone, a 40 ms release, and a short silent gap before another queued tap. Holding H sustains a single voice; menus, loss of focus, and mute cancel queued taps.

Validation includes PCM tests for one/two/three buffered taps, existing hold/release and pause checks, independent channel gain/mute, save/load with an intentionally muted engine, legacy settings defaults, and control layout at 1280×720. Native Mandarin and English synthesis verified silence at 0%, reduced level at 25%, and full level at 100%. Evidence and settings previews: `dist/audio-volumes-2026-10-28/`.

Maximum-level tuning: engine/road gain is now 1.15 times the previous mix; announcement volume is capped at 0.9 of the native speech volume scale. The 0–100% controls and horn level remain unchanged.
