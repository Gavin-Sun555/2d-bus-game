# Local station names

In route planning, right-click a station (or Shift+left-click) to edit it, including stations outside the selected route. Chinese and English names have independent fields: 12 Unicode characters for Chinese; 30 printable English characters including spaces for English. Empty fields retain their default language-specific name. Save applies immediately; Cancel discards edits; Restore Default removes both overrides.

Map labels, route summaries, next-stop displays and stop-related notifications use the customized names. Renaming does not change station IDs, routes, vehicle positions, passengers or traffic logic.

Native startup loads `station_names.json` in the application's user configuration directory. On macOS this is `~/Library/Application Support/2d-bus-game/station_names.json`. Names are independent of the ten progress save slots and survive restarting, changing maps, and loading an older progress save. The key combines map theme, station ID, road-center position and direction to avoid applying a name to an unrelated custom map; changing lane width preserves names. Writes use a temporary file and atomic replacement. Invalid existing preference files are reported and preserved rather than overwritten.

The editor uses Ebitengine's native text composition API. Mac Cmd+V reads the clipboard only when explicitly pressed in the focused editor. Tab changes fields, Enter saves, Escape cancels, and Cmd/Ctrl+A selects the field contents.

Validation (2026-10-28):

- `go test -short ./...`: all packages passed.
- Focused station-name persistence/editor tests and save/load regression passed.
- Race checks for name persistence, failed writes, route cache invalidation, editor cancellation, and bilingual names passed.
- `go vet ./game ./internal/sim ./internal/ui ./internal/view` passed.
- Native Mac QA verified English key input, Chinese paste, field switching, counters and Enter save. It used temporary preferences and did not open the user's progress save. Interactive Pinyin candidate selection was not automated.
- Rendered and inspected Chinese/English dialogs at logical widths 800 and 1280.
- `./build.sh app` passed.

Evidence: ignored `dist/station-names-2026-10-28/`. Optional UI preview: set `BUSGAME_STATION_NAME_PREVIEW` to an output directory when running `go test ./game -run '^$'`; add `BUSGAME_STATION_NAME_INTERACTIVE=1` for the temporary native editor session.
