param()

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$testHeaderPath = Join-Path $projectRoot "Test\test.h"
$testSourcePath = Join-Path $projectRoot "Test\test.c"
$testConfigPath = Join-Path $projectRoot "Test\test_config.h"
$taskConfigPath = Join-Path $projectRoot "APP\app_task_config.h"
$docRoot = Join-Path $projectRoot "Doc"
$testDocName = (-join @(
    [char]0x6D4B, [char]0x8BD5, [char]0x4EFB, [char]0x52A1,
    [char]0x6CE8, [char]0x518C, [char]0x51FD, [char]0x6570,
    [char]0x4F7F, [char]0x7528, [char]0x65B9, [char]0x6CD5,
    ".md"
))
$testDocPath = Join-Path $docRoot $testDocName

$testHeader = Get-Content -LiteralPath $testHeaderPath -Raw -Encoding UTF8
$testSource = Get-Content -LiteralPath $testSourcePath -Raw -Encoding UTF8
$testConfig = Get-Content -LiteralPath $testConfigPath -Raw -Encoding UTF8
$taskConfig = Get-Content -LiteralPath $taskConfigPath -Raw -Encoding UTF8
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
            if ($line -notmatch "\\\s*$") {
                $taskTableFormatErrors.Add("${markdownName}:$lineNumber task table start")
            }
        } elseif ($inTaskTable -and ($line -match "^\s*\};\s*$")) {
            $inTaskTable = $false
        } elseif ($inTaskTable -and
                  ($line -match "^\s*\{\s*[A-Za-z_][A-Za-z0-9_]*\s*,")) {
            if ($line -notmatch "\\\s*$") {
                $taskTableFormatErrors.Add("${markdownName}:$lineNumber task entry")
            }
            if (($line -match "//") -or
                (($line -match "/\*") -and ($line -notmatch "\*/\s*\\\s*$"))) {
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

$taskConfigUsesTests = (($taskConfig -match '#include\s+"test\.h"') -or
                        ($taskConfig -match "\{\s*Test_[A-Za-z0-9_]+\s*,"))
if ($taskConfigUsesTests) {
    if ($testConfig -notmatch "#define\s+PROJECT_TEST_TASKS_ENABLE\s+1U") {
        throw "APP task table uses Test functions, so PROJECT_TEST_TASKS_ENABLE must be 1U"
    }
} elseif ($testConfig -notmatch "#define\s+PROJECT_TEST_TASKS_ENABLE\s+0U") {
    throw "Formal APP task table requires PROJECT_TEST_TASKS_ENABLE=0U"
}

Write-Host ("Test API documentation check passed: {0} public Test functions, {1} valid task functions." -f
            $publicFunctions.Count,
            @($allRegisteredFunctions | Sort-Object -Unique).Count)
