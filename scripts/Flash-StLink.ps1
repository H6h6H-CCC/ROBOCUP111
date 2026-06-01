[CmdletBinding()]
param(
    [string]$Preset = "Debug",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $PSScriptRoot
$elfPath = Join-Path $repoRoot "build\$Preset\ROBOCUP.elf"
$hexPath = Join-Path $repoRoot "build\$Preset\ROBOCUP.hex"

if (-not $SkipBuild) {
    & (Join-Path $PSScriptRoot "Build-Stm32.ps1") -Preset $Preset
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if (-not (Test-Path $elfPath)) {
    throw "ELF not found: $elfPath"
}

. (Join-Path $PSScriptRoot "Resolve-Stm32Tools.ps1")
$tools = Resolve-Stm32Tools -Require @("FlashTool")

if ($tools.FlashToolKind -eq "STM32CubeProgrammer") {
    Write-Host "Using STM32CubeProgrammer: $($tools.Programmer)"
    Write-Host "Flashing image: $elfPath"

    & $tools.Programmer `
        -c port=SWD freq=4000 mode=UR reset=HWrst `
        -w $elfPath `
        -v `
        -rst
}
elseif ($tools.FlashToolKind -eq "ST-LINK_CLI") {
    if (-not (Test-Path $hexPath)) {
        throw "HEX not found: $hexPath. Rebuild the project to generate the flash image."
    }

    Write-Host "Using ST-LINK_CLI: $($tools.StLinkCli)"
    Write-Host "Flashing image: $hexPath"

    & $tools.StLinkCli `
        -c SWD UR Freq=4000 `
        -P $hexPath `
        -V after_programming `
        -Rst `
        -Run
}
else {
    throw "No supported flash tool was found."
}

exit $LASTEXITCODE
