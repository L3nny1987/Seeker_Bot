param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
& python (Join-Path $PSScriptRoot 'run_host_tests.py') --compiler $Compiler
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
