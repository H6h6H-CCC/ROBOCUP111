[CmdletBinding()]
param(
    [string]$Preset = "Debug",
    [switch]$ConfigureOnly
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Get-NormalizedPath {
    param([string]$PathValue)

    return [System.IO.Path]::GetFullPath($PathValue).TrimEnd("\").ToLowerInvariant()
}

function Test-NeedsFreshConfigure {
    param(
        [string]$CacheFile,
        [string]$RepoRoot
    )

    if (-not (Test-Path $CacheFile)) {
        return $true
    }

    $homeLine = Select-String -Path $CacheFile -Pattern '^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$' -ErrorAction SilentlyContinue |
        Select-Object -First 1

    if (-not $homeLine) {
        return $true
    }

    $cachedRoot = $homeLine.Matches[0].Groups[1].Value
    return (Get-NormalizedPath $cachedRoot) -ne (Get-NormalizedPath $RepoRoot)
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$cacheFile = Join-Path $repoRoot "build\$Preset\CMakeCache.txt"

. (Join-Path $PSScriptRoot "Resolve-Stm32Tools.ps1")
$tools = Resolve-Stm32Tools -Require @("CMake", "Gcc", "Ninja")

$env:PATH = "$($tools.GccDir);$($tools.NinjaDir);$env:PATH"

$configureArgs = @("--preset", $Preset)
if (Test-NeedsFreshConfigure -CacheFile $cacheFile -RepoRoot $repoRoot) {
    $configureArgs += "--fresh"
}

Push-Location $repoRoot
try {
    Write-Host "Using CMake: $($tools.CMake)"
    Write-Host "Using GCC:   $($tools.Gcc)"
    Write-Host "Using Ninja: $($tools.Ninja)"

    & $tools.CMake @configureArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    if (-not $ConfigureOnly) {
        & $tools.CMake --build --preset $Preset --parallel
        if ($LASTEXITCODE -ne 0) {
            exit $LASTEXITCODE
        }
    }
}
finally {
    Pop-Location
}
