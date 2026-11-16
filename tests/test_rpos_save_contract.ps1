# Exercise RPOS/SAVE behavior rather than matching its source-code spelling.
# Requires Python 3 and the same ZIG_EXE setup as tests/safety.
$ErrorActionPreference = 'Stop'
$pythonExe = if ($env:DECAP_TEST_PYTHON) { $env:DECAP_TEST_PYTHON } else { 'python' }
& $pythonExe (Join-Path $PSScriptRoot 'safety/test_r_teaching.py')
if ($LASTEXITCODE -ne 0) {
    throw 'RPOS/SAVE behavioral regression failed.'
}
