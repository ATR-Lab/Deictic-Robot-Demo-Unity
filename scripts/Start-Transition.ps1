# Start the transition task or receive-only hardware view; Unity stays under your control.
[CmdletBinding()]
param(
    [ValidateSet('Isaac','Logical','HardwareObservation')][string]$Mode = 'Isaac',
    [ValidatePattern('^[A-Za-z0-9_.-]+@[A-Za-z0-9.-]+$')]
    [string]$Remote = 'marnett5@mlworkstation.atr.cs.kent.edu',
    [string]$RemoteRepo = '~/Developer/deictic-k1-reproduction',
    [ValidateSet('ordinary','consequence')][string]$Policy = 'consequence',
    [ValidateSet('summary','history')][string]$Display = 'summary',
    [ValidateScript({ $parsed = $null; [Net.IPAddress]::TryParse($_, [ref]$parsed) -and $parsed.AddressFamily -eq [Net.Sockets.AddressFamily]::InterNetwork })]
    [string]$PublicIp = '131.123.237.31',
    [switch]$NoWebRTC,
    [switch]$NoViewer,
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'
function Quote-Sh([string]$Value) {
    if ($Value -match '[\r\n\x00\x22]') { throw 'Invalid remote path' }
    return "'" + $Value.Replace("'", "'\''") + "'"
}
$remoteMode = @{Isaac='isaac';Logical='logical';HardwareObservation='hardware-observation'}[$Mode]
$controlMode = if ($Mode -eq 'HardwareObservation') { 'hardware_observation' } else { 'transition_simulation' }
$repo = Split-Path $PSScriptRoot -Parent
if ($Mode -eq 'HardwareObservation' -and (Test-Path -LiteralPath (Join-Path $repo 'transition/.runtime/k1-diagnostics-hold.json'))) {
    throw 'K1 diagnostics are held after an out-of-memory/reset incident. See transition/docs/K1_RESET_INCIDENT.md.'
}
$settings = Join-Path $repo 'Deictic-Robot-Demo/Assets/StreamingAssets/transition-runtime.json'
# Expand only the known leading ~/ form on the remote side, not arbitrary shell text.
if ($RemoteRepo.StartsWith('~/')) { $command = 'cd "$HOME"/' + (Quote-Sh $RemoteRepo.Substring(2)) }
else { $command = 'cd ' + (Quote-Sh $RemoteRepo) }
$command += ' && exec bash transition/scripts/run_stack.sh ' + $remoteMode
if ($Mode -ne 'HardwareObservation') { $command += ' ' + $Policy.ToLowerInvariant() + ' ' + $Display.ToLowerInvariant() }
if ($Mode -eq 'Isaac') { $command += ' ' + $(if ($NoWebRTC) { 'off' } else { Quote-Sh $PublicIp }) }
$ports = @(10000)
if ($Mode -ne 'HardwareObservation') { $ports += 8766 }
$sshArgs = @('-tt','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3')
foreach ($port in $ports) { $sshArgs += @('-L',"127.0.0.1:${port}:127.0.0.1:${port}") }
$sshArgs += @($Remote,$command)
Write-Host "Mode: $Mode. Unity control mode: $controlMode. ROS visualization domain: 174."
Write-Host 'Unity and Play mode remain under your control. Keep this terminal open.'
if ($Mode -eq 'HardwareObservation') {
    Write-Host 'Physical task motion is unavailable; this mode receives measured telemetry only.'
    Write-Host 'The robot camera observer must be running (see transition/README.md).'
} else {
    Write-Host "Simulation policy: $Policy. Return display: $Display."
    Write-Host 'Operator panel: http://127.0.0.1:8766 (available after startup).'
}
if ($Mode -eq 'Isaac' -and -not $NoWebRTC) {
    Write-Host "WebRTC spectator: connect to $PublicIp (TCP 49100 / UDP 47998, directly to the workstation)."
    Write-Host 'Wait for "Transition mode=isaac domain=174", then click Connect in the viewer. Reconnect if an earlier attempt failed.'
}
if ($DryRun) { Write-Host "Dry run: $command"; return }
foreach ($port in $ports) {
    $listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback,$port)
    $listener.Server.ExclusiveAddressUse = $true
    try { $listener.Start() } catch { throw "Local port $port is occupied. Close the intended earlier tunnel before starting; nothing was stopped." }
    finally { $listener.Stop() }
}
New-Item -ItemType Directory -Force -Path (Split-Path $settings) | Out-Null
$cameraTopic = if ($Mode -eq 'HardwareObservation') { '/transition/hardware/head/image_raw/compressed' } else { '/deictic/camera_view/stereo/image_raw/compressed' }
@{schema_version=1;control_mode=$controlMode;ros_host='127.0.0.1';ros_port=10000;
  operator_url='http://127.0.0.1:8766';robot_camera_topic=$cameraTopic;
  robot_camera_stereo=($Mode -ne 'HardwareObservation')} | ConvertTo-Json | Set-Content -Encoding utf8 $settings
if ($Mode -eq 'Isaac' -and -not $NoWebRTC -and -not $NoViewer) {
    if (-not (Get-Process -Name 'Isaac Sim WebRTC Streaming Client' -ErrorAction SilentlyContinue)) {
        try { & (Join-Path $PSScriptRoot 'Start-IsaacStream.ps1') -Server $PublicIp }
        catch { Write-Warning "WebRTC viewer could not open: $_. Continuing with the simulator stack." }
    }
}
& ssh.exe @sshArgs
if ($LASTEXITCODE -notin @(0,130)) { throw "Transition SSH session ended with code $LASTEXITCODE" }
