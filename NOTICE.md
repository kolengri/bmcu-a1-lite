# Attribution and licensing status

Firmware derives from [jarczakpawel/BMCU-C-PJARCZAK](https://github.com/jarczakpawel/BMCU-C-PJARCZAK),
pinned at `5b8e235576f9d6c30403e737451ef60f97857dac` (V11-era source).
Existing source comments and notices are retained. Protocol comparisons also used
[zfkun/ams-lite-tiny](https://github.com/zfkun/ams-lite-tiny/tree/06573b58b8123a219478e14cbe7807e720d82cd8)
and the public [Bambu-Bus research](https://github.com/Bambu-Research-Group/Bambu-Bus).
Tiny is a comparison source, not an exact oracle for the owner's A1 firmware.

The pinned BMCU upstream tree contains no repository-level LICENSE, COPYING or
NOTICE file; GitHub reports no license. This repository does not invent an
upstream license grant or claim that inherited code is MIT/GPL licensed. Public
availability does not by itself establish permission for every downstream use.
Any third-party notices embedded in source retain their own meaning.

The isolated ISP companion reuses the pinned BMCU Flasher1.4 Python backend.
Its MIT license is preserved at `tools/bmcu-flasher-mcp/vendor/LICENSE`; that
license applies to the backend, not to the inherited BMCU firmware as a whole.

The compiler, PlatformIO platform and WCH SDK are external dependencies with their
own terms. They are not redistributed here. Historical serializer test oracles
are selected source from earlier stages of this project, with exact file hashes
in `validation/firmware/tests/oracles/manifest.json`; they are not stock-printer
firmware or captured printer traces.

