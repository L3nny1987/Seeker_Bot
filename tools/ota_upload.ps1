<#
.SYNOPSIS
    Uploads Seeker_Bot firmware (incl. web interface) over Wi-Fi.

.EXAMPLE
    .\tools\ota_upload.ps1
    .\tools\ota_upload.ps1 192.168.178.91 .\ota\seeker_bot_0.3.0.bin
#>
param(
    [Parameter(Position = 0)]
    [string]$Address = "192.168.178.91",
    [Parameter(Position = 1)]
    [string]$Firmware = (Join-Path $PSScriptRoot "..\ota\seeker_bot_latest.bin")
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $Firmware)) {
    throw "Firmware file not found: $Firmware"
}

$Address = $Address.Trim().TrimEnd("/")
if ($Address -notmatch "^https?://") {
    $Address = "http://$Address"
}
$endpoint = "$Address/api/ota"
$size = (Get-Item -LiteralPath $Firmware).Length

Write-Host "OTA upload: $Firmware ($size bytes)" -ForegroundColor Cyan
Write-Host "Target: $endpoint (robot must be IDLE)" -ForegroundColor Cyan

# Resumable upload in 4 KB blocks: a Wi-Fi dropout only repeats one block.
# Same headers the dashboard sends; the robot rejects cross-site requests.
Add-Type -AssemblyName System.Net.Http
$client = New-Object System.Net.Http.HttpClient
$client.Timeout = [TimeSpan]::FromSeconds(10)
$data = [System.IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Firmware))
$offset = 0
$failures = 0
$started = Get-Date
try {
    while ($true) {
        $count = [Math]::Min(4096, $size - $offset)
        $content = New-Object System.Net.Http.ByteArrayContent -ArgumentList $data, $offset, $count
        $content.Headers.ContentType = "application/octet-stream"
        $request = New-Object System.Net.Http.HttpRequestMessage -ArgumentList ([System.Net.Http.HttpMethod]::Post), "$endpoint`?offset=$offset&total=$size"
        $request.Headers.Add("X-Seeker-Control", "1")
        $request.Headers.Add("Origin", $Address)
        $request.Content = $content
        $reply = $null
        try {
            $response = $client.SendAsync($request).GetAwaiter().GetResult()
            $text = $response.Content.ReadAsStringAsync().GetAwaiter().GetResult()
            try { $reply = $text | ConvertFrom-Json }
            catch { $reply = [pscustomobject]@{ ok = $false; error = "HTTP $([int]$response.StatusCode): $text" } }
        } catch {
            $reply = $null
        }
        if ($reply -and $reply.ok) {
            $failures = 0
            $offset = [int]$reply.written
            Write-Progress -Activity "OTA upload" -Status "$offset / $size bytes" -PercentComplete (100 * $offset / $size)
            if ($reply.done) { break }
            continue
        }
        if ((-not $reply -or $reply.error -eq "receive_failed") -and ++$failures -le 40) {
            Start-Sleep -Milliseconds 500
            continue
        }
        $why = if ($reply) { $reply.error } else { "link lost" }
        throw "OTA upload failed at $offset/$size bytes: $why"
    }
} finally {
    Write-Progress -Activity "OTA upload" -Completed
    $client.Dispose()
}

$seconds = [Math]::Round(((Get-Date) - $started).TotalSeconds, 1)
Write-Host "OTA accepted ($size bytes in $seconds s). The robot restarts with the new firmware." -ForegroundColor Green
