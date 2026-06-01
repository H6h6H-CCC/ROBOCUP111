[CmdletBinding()]
param(
    [int]$Port = 61234,
    [int]$SemihostConsolePort = 50000
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot "Resolve-Stm32Tools.ps1")
$tools = Resolve-Stm32Tools -Require @("Programmer", "StLinkGdbServer")

Write-Host "Using ST-LINK GDB Server: $($tools.StLinkGdbServer)"
Write-Host "Using STM32CubeProgrammer: $($tools.ProgrammerDir)"
Write-Host "GDB port: $Port"
Write-Host "Semihost console port: $SemihostConsolePort"

& $tools.StLinkGdbServer `
    -p $Port `
    -cp $tools.ProgrammerDir `
    --swd `
    --halt `
    --semihosting terminal `
    --semihost-console-port $SemihostConsolePort

exit $LASTEXITCODE
