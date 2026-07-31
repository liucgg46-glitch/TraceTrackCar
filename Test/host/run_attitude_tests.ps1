$ErrorActionPreference = "Stop"

$repositoryRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot "..\..")
)
$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"

if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "Visual Studio Installer was not found."
}

$visualStudioRoot = & $vswhere `
    -latest `
    -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -property installationPath

if ([string]::IsNullOrWhiteSpace($visualStudioRoot)) {
    throw "Visual Studio C++ x64 build tools are not installed."
}

$vcvars = Join-Path $visualStudioRoot "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars)) {
    throw "vcvars64.bat was not found: $vcvars"
}

$buildDirectory = Join-Path $env:TEMP "TraceTrackCarHostTests"
$testExecutable = Join-Path $buildDirectory "attitude_estimator_tests.exe"
$testObject = Join-Path $buildDirectory "test_attitude_estimator.obj"
$algorithmObject = Join-Path $buildDirectory "attitude_estimator.obj"
$stubInclude = Join-Path $repositoryRoot "Test\host\stubs"
$commonInclude = Join-Path $repositoryRoot "Common"
$algorithmInclude = Join-Path $repositoryRoot "Algorithm"
$testSource = Join-Path $repositoryRoot "Test\host\test_attitude_estimator.c"
$criticalSource = Join-Path $repositoryRoot "Test\host\stubs\project_critical.c"
$algorithmSource = Join-Path $repositoryRoot "Algorithm\attitude_estimator.c"
$criticalObject = Join-Path $buildDirectory "project_critical.obj"
$odometerTestExecutable = Join-Path $buildDirectory "odometer_tests.exe"
$odometerTestObject = Join-Path $buildDirectory "test_odometer.obj"
$odometerObject = Join-Path $buildDirectory "odometer.obj"
$odometerTestSource = Join-Path $repositoryRoot "Test\host\test_odometer.c"
$odometerSource = Join-Path $repositoryRoot "Algorithm\odometer.c"
$ballTestExecutable = Join-Path $buildDirectory "ball_balance_tests.exe"
$ballTestObject = Join-Path $buildDirectory "test_ball_balance.obj"
$ballEstimatorObject = Join-Path $buildDirectory "ball_state_estimator.obj"
$ballReferenceObject = Join-Path $buildDirectory "ball_reference_generator.obj"
$ballMapObject = Join-Path $buildDirectory "ball_equilibrium_map.obj"
$ballControlObject = Join-Path $buildDirectory "ball_balance_control.obj"
$ballTestSource = Join-Path $repositoryRoot "Test\host\test_ball_balance.c"
$ballEstimatorSource = Join-Path $repositoryRoot "Algorithm\ball_state_estimator.c"
$ballReferenceSource = Join-Path $repositoryRoot "Algorithm\ball_reference_generator.c"
$ballMapSource = Join-Path $repositoryRoot "Algorithm\ball_equilibrium_map.c"
$ballControlSource = Join-Path $repositoryRoot "Algorithm\ball_balance_control.c"
$taskFsmTestExecutable = Join-Path $buildDirectory "task_fsm_tests.exe"
$taskFsmTestObject = Join-Path $buildDirectory "test_task_fsm.obj"
$taskFsmObject = Join-Path $buildDirectory "task_fsm.obj"
$taskFsmTestSource = Join-Path $repositoryRoot "Test\host\test_task_fsm.c"
$taskFsmSource = Join-Path $repositoryRoot "APP\task_fsm.c"

[void](New-Item -ItemType Directory -Force -Path $buildDirectory)

$compileAndRun = @"
call "$vcvars" >nul && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$commonInclude" /I"$algorithmInclude" /c "$testSource" /Fo"$testObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$commonInclude" /I"$algorithmInclude" /c "$algorithmSource" /Fo"$algorithmObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$commonInclude" /c "$criticalSource" /Fo"$criticalObject" && cl /nologo "$testObject" "$algorithmObject" "$criticalObject" /Fe"$testExecutable" && "$testExecutable"
"@

& $env:ComSpec /d /s /c $compileAndRun
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$compileAndRunBall = @"
call "$vcvars" >nul && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$ballTestSource" /Fo"$ballTestObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$ballEstimatorSource" /Fo"$ballEstimatorObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$ballReferenceSource" /Fo"$ballReferenceObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$ballMapSource" /Fo"$ballMapObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$ballControlSource" /Fo"$ballControlObject" && cl /nologo "$ballTestObject" "$ballEstimatorObject" "$ballReferenceObject" "$ballMapObject" "$ballControlObject" /Fe"$ballTestExecutable" && "$ballTestExecutable"
"@

& $env:ComSpec /d /s /c $compileAndRunBall
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$compileAndRunTaskFsm = @"
call "$vcvars" >nul && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$commonInclude" /I"$algorithmInclude" /I"$repositoryRoot\APP" /c "$taskFsmTestSource" /Fo"$taskFsmTestObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$commonInclude" /I"$algorithmInclude" /I"$repositoryRoot\APP" /c "$taskFsmSource" /Fo"$taskFsmObject" && cl /nologo "$taskFsmTestObject" "$taskFsmObject" /Fe"$taskFsmTestExecutable" && "$taskFsmTestExecutable"
"@

& $env:ComSpec /d /s /c $compileAndRunTaskFsm
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$compileAndRunOdometer = @"
call "$vcvars" >nul && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$odometerTestSource" /Fo"$odometerTestObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$commonInclude" /I"$algorithmInclude" /c "$odometerSource" /Fo"$odometerObject" && cl /nologo "$odometerTestObject" "$odometerObject" /Fe"$odometerTestExecutable" && "$odometerTestExecutable"
"@

& $env:ComSpec /d /s /c $compileAndRunOdometer
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
