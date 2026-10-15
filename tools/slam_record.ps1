<#
.SYNOPSIS
    Controls the SLAM input recorder and downloads recordings.

.EXAMPLE
    .\tools\slam_record.ps1 start ROBOTER_IP      # robot IDLE; erases the free OTA slot
    .\tools\slam_record.ps1 status ROBOTER_IP
    .\tools\slam_record.ps1 stop ROBOTER_IP
    .\tools\slam_record.ps1 download ROBOTER_IP   # -> recordings\slam_<time>.bin
#>
param(
    [Parameter(Position = 0, Mandatory = $true)]
    [ValidateSet("start", "stop", "status", "download")]
    [string]$Command,
    [Parameter(Position = 1, Mandatory = $true)]
    [string]$Address
)

$ErrorActionPreference = "Stop"
$Address = $Address.Trim().TrimEnd("/")
if ($Address -notmatch "^https?://") { $Address = "http://$Address" }
$endpoint = "$Address/api/record"

Add-Type -AssemblyName System.Net.Http
$client = New-Object System.Net.Http.HttpClient
$client.Timeout = [TimeSpan]::FromSeconds(10)

function Invoke-Retry([scriptblock]$Action) {
    for ($try = 1; ; $try++) {
        try { return & $Action }
        catch { if ($try -ge 40) { throw } ; Start-Sleep -Milliseconds 500 }
    }
}

function Get-Status {
    Invoke-Retry { $client.GetStringAsync($endpoint).GetAwaiter().GetResult() | ConvertFrom-Json }
}

try {
    switch ($Command) {
        { $_ -in "start", "stop" } {
            $request = New-Object System.Net.Http.HttpRequestMessage -ArgumentList ([System.Net.Http.HttpMethod]::Post), "$endpoint`?cmd=$Command"
            $request.Headers.Add("X-Seeker-Control", "1")
            $request.Headers.Add("Origin", $Address)
            $response = $client.SendAsync($request).GetAwaiter().GetResult()
            $text = $response.Content.ReadAsStringAsync().GetAwaiter().GetResult()
            if (-not $response.IsSuccessStatusCode) { throw "HTTP $([int]$response.StatusCode): $text" }
            if ($Command -eq "start") {
                do { Start-Sleep -Milliseconds 500; $s = Get-Status } while ($s.erasing)
                if (-not $s.active) { throw "Recording did not start" }
                $minutes = [Math]::Round($s.capacity / 6 / 60, 1)
                Write-Host "Recording (max $($s.capacity) scans, ~$minutes min)." -ForegroundColor Green
            } else {
                Write-Host "Stopped: $((Get-Status).records) scans." -ForegroundColor Green
            }
        }
        "status" { Get-Status | Format-List }
        "download" {
            $s = Get-Status
            $total = [int64]$s.records * $s.recordSize
            if ($total -eq 0) { throw "No recording on the robot" }
            if ($s.active) { Write-Warning "Recording still active: downloading $($s.records) scans recorded so far" }
            $dir = Join-Path $PSScriptRoot "..\recordings"
            New-Item -ItemType Directory -Force -Path $dir | Out-Null
            $file = Join-Path (Resolve-Path $dir) ("slam_{0:yyyyMMdd_HHmmss}.bin" -f (Get-Date))
            $out = [System.IO.File]::Create($file)
            try {
                for ($offset = 0; $offset -lt $total; $offset += $len) {
                    $len = [Math]::Min(4096, $total - $offset)
                    $bytes = Invoke-Retry {
                        $b = $client.GetByteArrayAsync("$endpoint`?offset=$offset&len=$len").GetAwaiter().GetResult()
                        if ($b.Length -ne $len) { throw "short read" }
                        , $b
                    }
                    $out.Write($bytes, 0, $len)
                    Write-Progress -Activity "Download" -Status "$offset / $total bytes" -PercentComplete (100 * $offset / $total)
                }
            } finally {
                Write-Progress -Activity "Download" -Completed
                $out.Dispose()
            }
            Write-Host "Saved $($s.records) scans to $file" -ForegroundColor Green
        }
    }
} finally {
    $client.Dispose()
}
