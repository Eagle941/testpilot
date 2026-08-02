#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/.."
package="${1:?usage: scripts/install.sh A32NX_PACKAGE_PATH}"
aircraft="$package/SimObjects/AirPlanes/FlyByWire_A320_NEO"
panel="$aircraft/panel"

sh scripts/build-wasm.sh
cp -f target/wasm32-wasip1/release/testpilot-msfs.wasm "$panel/testpilot.wasm"
cp -f example/replayer_config.toml "$aircraft/replayer_config.toml"
cp -f example/scenario.csv "$aircraft/scenario.csv"

python - "$package" <<'PY'
import json
import sys
from pathlib import Path

package = Path(sys.argv[1])
aircraft_relative = "SimObjects/AirPlanes/FlyByWire_A320_NEO"
config_relative = f"{aircraft_relative}/replayer_config.toml"
scenario_relative = f"{aircraft_relative}/scenario.csv"
panel_relative = f"{aircraft_relative}/panel/panel.cfg"
wasm_relative = f"{aircraft_relative}/panel/testpilot.wasm"
panel = package / panel_relative
layout_file = package / "layout.json"
gauge = (
    "htmlgauge04 = WasmInstrument/WasmInstrument.html?"
    "wasm_module=testpilot.wasm&wasm_gauge=testpilot,0,0,1,1"
)

text = panel.read_text()
start = text.index("[VCockpit17]")
end = text.find("\n[", start + 1)
end = len(text) if end < 0 else end + 1
section = text[start:end]
if gauge not in section.splitlines():
    section = section.rstrip() + "\n\n" + gauge + "\n\n"
text = text[:start] + section + text[end:]
panel.write_text(text)

layout = json.loads(layout_file.read_text(encoding="utf-8-sig"))
for relative in (config_relative, scenario_relative, panel_relative, wasm_relative):
    file = package / relative
    stat = file.stat()
    metadata = {
        "path": relative,
        "size": stat.st_size,
        "date": 116_444_736_000_000_000 + stat.st_mtime_ns // 100,
    }
    entry = next((item for item in layout["content"] if item["path"] == relative), None)
    if entry is None:
        layout["content"].append(metadata)
    else:
        entry.update(metadata)

layout_file.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
PY
