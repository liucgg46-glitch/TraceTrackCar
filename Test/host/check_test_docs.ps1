param()

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$testHeaderPath = Join-Path $projectRoot "Test\test.h"
$testSourcePaths = Get-ChildItem -LiteralPath (Join-Path $projectRoot "Test") `
    -Filter "test*.c" -File
$appTaskConfigPath = Join-Path $projectRoot "APP\app_task_config.h"
$docRoot = Join-Path $projectRoot "Doc"
$testDocName = (-join @(
    [char]0x6D4B, [char]0x8BD5, [char]0x4EFB, [char]0x52A1,
    [char]0x5B8C, [char]0x6574, [char]0x624B, [char]0x518C,
    ".md"
))
$testDocPath = Join-Path $docRoot $testDocName

$testHeader = Get-Content -LiteralPath $testHeaderPath -Raw -Encoding UTF8
$testSource = @(
    $testSourcePaths |
        ForEach-Object {
            Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8
        }
) -join "`n"
$appTaskConfig = Get-Content -LiteralPath $appTaskConfigPath -Raw -Encoding UTF8
$testDoc = Get-Content -LiteralPath $testDocPath -Raw -Encoding UTF8

$publicFunctions = @(
    [regex]::Matches($testHeader, "\bvoid\s+(Test_[A-Za-z0-9_]+)\s*\(") |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)

$definedFunctions = @(
    [regex]::Matches($testSource, "(?m)^\s*(?:static\s+)?void\s+(Test_[A-Za-z0-9_]+)\s*\(") |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)

$missingDefinitions = @($publicFunctions | Where-Object { $_ -notin $definedFunctions })
if ($missingDefinitions.Count -ne 0) {
    throw "Public Test functions without definitions: $($missingDefinitions -join ', ')"
}

$missingDocumentation = @(
    $publicFunctions |
        Where-Object { $testDoc -notmatch ("\b" + [regex]::Escape($_) + "\b") }
)
if ($missingDocumentation.Count -ne 0) {
    throw "Public Test functions missing from the test task document: $($missingDocumentation -join ', ')"
}

$registeredFunctions = New-Object System.Collections.Generic.List[string]
$allRegisteredFunctions = New-Object System.Collections.Generic.List[string]
$taskTableFormatErrors = New-Object System.Collections.Generic.List[string]
Get-ChildItem -LiteralPath $docRoot -Filter "*.md" -File | ForEach-Object {
    $markdownName = $_.Name
    $markdown = Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8
    [regex]::Matches($markdown, "\{\s*(Test_[A-Za-z0-9_]+)\s*,") | ForEach-Object {
        $registeredFunctions.Add($_.Groups[1].Value)
    }
    [regex]::Matches(
        $markdown,
        "\{\s*([A-Za-z_][A-Za-z0-9_]*)\s*,\s*\d+U\s*,\s*0U\s*\}"
    ) | ForEach-Object {
        $allRegisteredFunctions.Add($_.Groups[1].Value)
    }

    $inTaskTable = $false
    $lineNumber = 0
    Get-Content -LiteralPath $_.FullName -Encoding UTF8 | ForEach-Object {
        $lineNumber++
        $line = $_
        if ($line -match "^\s*Task_t\s+task_list\[\]\s*=\s*\{") {
            $inTaskTable = $true
            if ($line -match "\\\s*$") {
                $taskTableFormatErrors.Add("${markdownName}:$lineNumber legacy task table continuation")
            }
        } elseif ($inTaskTable -and ($line -match "^\s*\};\s*$")) {
            $inTaskTable = $false
        } elseif ($inTaskTable -and
                  ($line -match "^\s*\{\s*[A-Za-z_][A-Za-z0-9_]*\s*,")) {
            if ($line -match "\\\s*$") {
                $taskTableFormatErrors.Add("${markdownName}:$lineNumber legacy task entry continuation")
            }
            if (($line -match "//") -or
                (($line -match "/\*") -and ($line -notmatch "\*/\s*$"))) {
                $taskTableFormatErrors.Add("${markdownName}:$lineNumber task comment")
            }
        }
    }
}

$invalidRegistrations = @(
    $registeredFunctions |
        Sort-Object -Unique |
        Where-Object { $_ -notin $publicFunctions }
)
if ($invalidRegistrations.Count -ne 0) {
    throw "Invalid Test functions registered in Markdown task tables: $($invalidRegistrations -join ', ')"
}

if ($taskTableFormatErrors.Count -ne 0) {
    throw "Invalid Markdown task table format: $($taskTableFormatErrors -join ', ')"
}

$legacyTaskMacros = @(
    Get-ChildItem -LiteralPath $docRoot -Filter "*.md" -File |
        Where-Object {
            (Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8) -match
                "(?:APP|TEST)_SCHEDULER_TASK_LIST_DEFINE"
        } |
        ForEach-Object { $_.Name }
)
if ($legacyTaskMacros.Count -ne 0) {
    throw "Legacy scheduler task macros remain in Markdown: $($legacyTaskMacros -join ', ')"
}

$firmwareText = @(
    "APP", "Algorithm", "BSP", "Common", "Driver", "Route", "Test", "user"
) | ForEach-Object {
    Get-ChildItem -LiteralPath (Join-Path $projectRoot $_) -Recurse -File |
        Where-Object { $_.Extension -in @(".c", ".h") } |
        ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8 }
}
$firmwareText = $firmwareText -join "`n"
$missingTaskFunctions = @(
    $allRegisteredFunctions |
        Sort-Object -Unique |
        Where-Object {
            $firmwareText -notmatch ("\b" + [regex]::Escape($_) + "\s*\(")
        }
)
if ($missingTaskFunctions.Count -ne 0) {
    throw "Task functions in Markdown without source declarations: $($missingTaskFunctions -join ', ')"
}

$publicHeaderText = @(
    "APP", "Algorithm", "BSP", "Common", "Driver", "Route", "Test", "user"
) | ForEach-Object {
    Get-ChildItem -LiteralPath (Join-Path $projectRoot $_) -Recurse -Filter "*.h" -File |
        ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8 }
}
$publicHeaderText = $publicHeaderText -join "`n"
$publicSchedulableFunctions = @(
    [regex]::Matches(
        $publicHeaderText,
        "\bvoid\s+([A-Za-z_][A-Za-z0-9_]*(?:Update|Task|Run|Toggle|Ramp|Scan|Background))\s*\(\s*void\s*\)\s*;"
    ) |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)
$undocumentedSchedulableFunctions = @(
    $publicSchedulableFunctions |
        Where-Object { $testDoc -notmatch ("\b" + [regex]::Escape($_) + "\b") }
)
if ($undocumentedSchedulableFunctions.Count -ne 0) {
    throw "Public schedulable functions missing from the test task document: $($undocumentedSchedulableFunctions -join ', ')"
}

$appRegisteredTests = @(
    [regex]::Matches(
        $appTaskConfig,
        "\{\s*(Test_[A-Za-z0-9_]+)\s*,"
    ) |
        ForEach-Object { $_.Groups[1].Value } |
        Sort-Object -Unique
)
if ($appTaskConfig -notmatch '#include\s+"test\.h"') {
    throw "APP task config registers tests but does not include test.h"
}
$invalidAppTests = @(
    $appRegisteredTests |
        Where-Object { $_ -notin $publicFunctions }
)
if ($invalidAppTests.Count -ne 0) {
    throw "APP task config contains invalid Test functions: $($invalidAppTests -join ', ')"
}

Write-Host ("Test API documentation check passed: {0} public Test functions, {1} public schedulable functions, {2} valid task functions." -f
            $publicFunctions.Count,
            $publicSchedulableFunctions.Count,
            @($allRegisteredFunctions | Sort-Object -Unique).Count)
