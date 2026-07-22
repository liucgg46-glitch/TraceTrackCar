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
$algorithmInclude = Join-Path $repositoryRoot "Algorithm"
$testSource = Join-Path $repositoryRoot "Test\host\test_attitude_estimator.c"
$algorithmSource = Join-Path $repositoryRoot "Algorithm\attitude_estimator.c"

[void](New-Item -ItemType Directory -Force -Path $buildDirectory)

$compileAndRun = @"
call "$vcvars" >nul && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$algorithmInclude" /c "$testSource" /Fo"$testObject" && cl /nologo /TC /std:c11 /utf-8 /W4 /WX /I"$stubInclude" /I"$algorithmInclude" /c "$algorithmSource" /Fo"$algorithmObject" && cl /nologo "$testObject" "$algorithmObject" /Fe"$testExecutable" && "$testExecutable"
"@

& $env:ComSpec /d /s /c $compileAndRun
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
