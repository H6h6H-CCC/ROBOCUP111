Set-StrictMode -Version Latest

function Get-PreferredFile {
    param(
        [string[]]$Patterns,
        [string]$CommandName
    )

    foreach ($pattern in $Patterns) {
        $matches = Get-ChildItem -Path $pattern -File -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending

        if ($matches) {
            return $matches[0].FullName
        }
    }

    if ($CommandName) {
        $command = Get-Command $CommandName -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($command) {
            return $command.Source
        }
    }

    return $null
}

function Test-ToolRequirement {
    param(
        [string]$ToolName,
        [string]$PathValue
    )

    return -not [string]::IsNullOrWhiteSpace($PathValue)
}

function Resolve-Stm32Tools {
    param(
        [string[]]$Require = @()
    )

    $cmake = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\cmake\*\bin\cmake.exe",
        "D:\C++\CLion *\bin\cmake\win\x64\bin\cmake.exe",
        "C:\Program Files\CMake\bin\cmake.exe",
        "C:\Program Files (x86)\CMake\bin\cmake.exe"
    ) -CommandName "cmake.exe"

    $gcc = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32\*\bin\arm-none-eabi-gcc.exe",
        "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gcc.exe",
        "C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gcc.exe"
    ) -CommandName "arm-none-eabi-gcc.exe"

    $gdb = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\gnu-gdb-for-stm32\*\bin\arm-none-eabi-gdb.exe",
        "$env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32\*\bin\arm-none-eabi-gdb.exe",
        "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gdb.exe",
        "C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gdb.exe"
    ) -CommandName "arm-none-eabi-gdb.exe"

    $ninja = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\ninja\*\bin\ninja.exe",
        "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\Ninja-build.Ninja_*\ninja.exe",
        "C:\Program Files\Ninja\bin\ninja.exe",
        "C:\Program Files (x86)\Ninja\bin\ninja.exe"
    ) -CommandName "ninja.exe"

    $programmer = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\programmer\*\bin\STM32_Programmer_CLI.exe",
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\ST\STM32CubeCLT_*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*\tools\bin\STM32_Programmer_CLI.exe"
    ) -CommandName "STM32_Programmer_CLI.exe"

    $stLinkGdbServer = Get-PreferredFile -Patterns @(
        "$env:LOCALAPPDATA\stm32cube\bundles\stlink-gdbserver\*\bin\ST-LINK_gdbserver.exe",
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeCLT\STLink-gdb-server\bin\ST-LINK_gdbserver.exe",
        "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeCLT\STLink-gdb-server\bin\ST-LINK_gdbserver.exe",
        "C:\ST\STM32CubeCLT_*\STLink-gdb-server\bin\ST-LINK_gdbserver.exe",
        "C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*\tools\bin\ST-LINK_gdbserver.exe"
    ) -CommandName "ST-LINK_gdbserver.exe"

    $stLinkCli = Get-PreferredFile -Patterns @(
        "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
        "C:\Program Files\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
    ) -CommandName "ST-LINK_CLI.exe"

    $flashTool = if ($programmer) { $programmer } else { $stLinkCli }
    $flashToolKind = if ($programmer) {
        "STM32CubeProgrammer"
    }
    elseif ($stLinkCli) {
        "ST-LINK_CLI"
    }
    else {
        $null
    }

    $missing = New-Object System.Collections.Generic.List[string]
    foreach ($requiredTool in $Require) {
        switch ($requiredTool) {
            "CMake" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $cmake)) {
                    $missing.Add("Could not find cmake.exe.")
                }
            }
            "Gcc" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $gcc)) {
                    $missing.Add("Could not find arm-none-eabi-gcc.exe.")
                }
            }
            "Gdb" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $gdb)) {
                    $missing.Add("Could not find arm-none-eabi-gdb.exe.")
                }
            }
            "Ninja" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $ninja)) {
                    $missing.Add("Could not find ninja.exe.")
                }
            }
            "Programmer" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $programmer)) {
                    $missing.Add("Could not find STM32_Programmer_CLI.exe.")
                }
            }
            "StLinkGdbServer" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $stLinkGdbServer)) {
                    $missing.Add("Could not find ST-LINK_gdbserver.exe.")
                }
            }
            "FlashTool" {
                if (-not (Test-ToolRequirement -ToolName $requiredTool -PathValue $flashTool)) {
                    $missing.Add("Could not find a flash tool. Install STM32CubeProgrammer or STM32 ST-LINK Utility.")
                }
            }
            default {
                $missing.Add("Unknown tool requirement: $requiredTool")
            }
        }
    }

    if ($missing.Count -gt 0) {
        throw ($missing -join [Environment]::NewLine)
    }

    [pscustomobject]@{
        CMake           = $cmake
        CMakeDir        = if ($cmake) { Split-Path -Parent $cmake } else { $null }
        Gcc             = $gcc
        GccDir          = if ($gcc) { Split-Path -Parent $gcc } else { $null }
        Gdb             = $gdb
        GdbDir          = if ($gdb) { Split-Path -Parent $gdb } else { $null }
        Ninja           = $ninja
        NinjaDir        = if ($ninja) { Split-Path -Parent $ninja } else { $null }
        Programmer      = $programmer
        ProgrammerDir   = if ($programmer) { Split-Path -Parent $programmer } else { $null }
        StLinkCli       = $stLinkCli
        StLinkCliDir    = if ($stLinkCli) { Split-Path -Parent $stLinkCli } else { $null }
        FlashTool       = $flashTool
        FlashToolDir    = if ($flashTool) { Split-Path -Parent $flashTool } else { $null }
        FlashToolKind   = $flashToolKind
        StLinkGdbServer = $stLinkGdbServer
        StLinkGdbDir    = if ($stLinkGdbServer) { Split-Path -Parent $stLinkGdbServer } else { $null }
    }
}
