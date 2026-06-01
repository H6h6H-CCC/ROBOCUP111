[CmdletBinding()]
param(
    [string]$ServerHost = "127.0.0.1",
    [int]$Port = 50000,
    [int]$RetryDelayMs = 250
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

Write-Host "Waiting for semihost console on ${ServerHost}:$Port"

while ($true) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $client.Connect($ServerHost, $Port)
        break
    }
    catch {
        $client.Dispose()
        Start-Sleep -Milliseconds $RetryDelayMs
    }
}

Write-Host "Connected to semihost console on ${ServerHost}:$Port"

$stream = $client.GetStream()
$buffer = New-Object byte[] 1024
$encoding = [System.Text.Encoding]::ASCII

try {
    while ($true) {
        try {
            $count = $stream.Read($buffer, 0, $buffer.Length)
        }
        catch [System.IO.IOException] {
            break
        }
        if ($count -le 0) {
            break
        }
        [Console]::Out.Write($encoding.GetString($buffer, 0, $count))
    }
}
finally {
    $stream.Dispose()
    $client.Dispose()
}

Write-Host ""
Write-Host "Semihost console disconnected"
