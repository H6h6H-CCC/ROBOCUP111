[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$CMakeArgs
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot "Resolve-Stm32Tools.ps1")
$tools = Resolve-Stm32Tools -Require @("CMake", "Gcc", "Ninja")

$env:PATH = "$($tools.GccDir);$($tools.NinjaDir);$env:PATH"

& $tools.CMake @CMakeArgs
exit $LASTEXITCODE
