[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [ValidateScript({ -not [string]::IsNullOrWhiteSpace($_) })]
    [string]$Server
)
$ErrorActionPreference = 'Stop'
$client = Join-Path $env:LOCALAPPDATA 'Programs/isaacsim-webrtc-streaming-client/Isaac Sim WebRTC Streaming Client.exe'
if (-not (Test-Path -LiteralPath $client)) { throw "Isaac Sim WebRTC Streaming Client not found: $client" }
Write-Host "In the streaming client, connect to $Server (signaling TCP 49100)."
Write-Host 'The simulation must be running with --webrtc and an advertised address reachable by this PC.'
Write-Host 'Media uses UDP; an SSH TCP tunnel alone cannot carry the WebRTC media stream.'
# A visible window is intentional: this is the interactive viewer requested by the user.
Start-Process -FilePath $client
