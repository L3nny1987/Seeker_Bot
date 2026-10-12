param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$originalPath = $env:PATH
Push-Location $projectRoot
try {
    $compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
    $env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
    $out = Join-Path $projectRoot '.pio\host-tests'
    New-Item -ItemType Directory -Force -Path $out | Out-Null
    $cases = @(
        @('navigation_test', 'src/navigation.c'),
        @('navigation_escape_test', 'src/navigation.c'),
        @('navigation_feedback_test', 'src/navigation.c'),
        @('motion_settings_test', ''),
        @('pose_feedback_test', ''),
        @('ld20_scan_test', 'components/ld20_lidar/ld20_scan_builder.c'),
        @('slam_test', 'src/slam.c')
    )
    foreach ($case in $cases) {
        $exe = Join-Path $out ($case[0] + '.exe')
        $compilerArgs = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-I', 'src',
            '-I', 'components/ld20_lidar/include', ('test/' + $case[0] + '.c'))
        if ($case[1]) { $compilerArgs += $case[1] }
        $compilerArgs += @('-lm', '-o', $exe)
        & $compilerPath @compilerArgs
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case[0])" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case[0])" }
    }
    Get-ChildItem test -Filter '*_test.js' | Sort-Object Name | ForEach-Object {
        & node $_.FullName
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $($_.Name)" }
    }
} finally {
    $env:PATH = $originalPath
    Pop-Location
}
