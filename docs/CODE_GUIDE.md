# Code Guide & Publication Audit

## How to explore this historical repository

- **Embedded entry point:** `sketch_sep14a/sketch_sep14a.ino` (ESP32, six servo outputs, framed serial input).
- **Laser-target upper computer:** `调用反解DLL库/shangweiji11.py`, with `vision2.py`.
- **Black-rectangle / square trajectory:** `调用反解DLL库/shangweijiblack4.py`, with `visonblack.py`.
- **Kinematic interface:** `调用反解DLL库/InK6RSSV1.h`; the binary solver is `RSS6RBT_InverseDLL.dll`.
- **History / exploration:** other `shangweiji*.py` scripts and `vision.py` reflect iterations. Their finality, test coverage and relationships have not been established.

## Publication audit: items requiring review

| Item | Observation | Recommendation |
|---|---|---|
| `调用反解DLL库/test.txt` | ~25 MB committed artifact | Determine content, provenance and whether it is needed. Prefer documented, anonymized samples over bulk logs. |
| `调用反解DLL库/扫描数据.xlsx` | ~18 MB spreadsheet | Check for personally identifying information, raw sensor captures or confidential results before retaining it publicly. |
| `调用反解DLL库/__pycache__/` | Compiled Python caches | Exclude from future commits; remove in a separately reviewed cleanup. |
| `调用反解DLL库/.idea/` and `__Previews/` | IDE/preview files | Exclude in a future source-only release unless project-specific configuration is genuinely required. |
| `调用反解DLL库/RSS6RBT_InverseDLL.dll` | Compiled native binary, source and usage permission not proven | Identify the owner/vendor and redistribution terms; document platform compatibility; do not relicense by assumption. |
| `调用反解DLL库/main.py` | Visible incomplete `ZP =` assignment | Do not present as a working entry point without a syntax repair and test. |
| Other controller versions | Similar names without changelog | Preserve as history; avoid calling a file “final” based on a suffix alone. |

**Review status:** file names, sizes and selected source content were inspected. The large data-file contents, spreadsheet cells, full git history, all Python files, malware status and real hardware operation were **not** exhaustively audited. No assertion of privacy clearance is made.

A future `.gitignore` can exclude `__pycache__/`, `*.pyc`, `.idea/`, temporary images/logs and local recordings. **Note:** ignoring or deleting a previously committed file does not remove it from Git history. If sensitive information is found, use an appropriate history-removal and credential-revocation process after reviewing downstream implications.

## Current publication strategy

This documentation update is **additive**: the existing historical engineering files have not been deleted or rewritten. The next cleanup should happen only after source/data ownership is established and hardware demonstrations are collected.

## Demo checklist

Record a short overview of the complete platform, then the perception UI, robot response, and one recognizable controlled-motion sequence. Make clear which of the two visual-processing/controller variants the video actually uses. Put a thumbnail or short GIF in the README and link the full original video under a separately documented public media location. Do not upload unreviewed raw camera recordings, faces, workspace details, device serial numbers or copyrighted materials.
