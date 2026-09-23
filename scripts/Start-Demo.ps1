# Run from Windows. Keep this terminal open; Ctrl+C stops this launch's services.
[CmdletBinding()]
param(
    [ValidatePattern('^[A-Za-z0-9_.-]+@[A-Za-z0-9.-]+$')]
    [string]$Remote = 'marnett5@mlworkstation.atr.cs.kent.edu',
    [ValidateNotNullOrEmpty()]
    [string]$RemoteRepo = '~/Developer/deictic-k1-reproduction',
    [ValidatePattern('^[A-Za-z0-9.-]+$')]
    [string]$PublicIp = '131.123.237.31',
    [ValidateRange(60, 3600)]
    [int]$StartupTimeout = 900,
    [switch]$NoViewer,
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'

# Quote for the remote POSIX shell without literal double quotes. This also
# avoids PowerShell 5.1's legacy native-argument double-quote processing.
function ConvertTo-ShArgument([string]$Value) {
    if ($Value.Contains("`r") -or $Value.Contains("`n") -or $Value.Contains([char]0) -or $Value.Contains([char]34)) {
        throw 'Remote arguments must be a single line without NUL or double-quote characters.'
    }
    return "'" + $Value.Replace("'", "'\''") + "'"
}

$launcherPath = Join-Path $PSScriptRoot 'start_demo_remote.py'
if (-not (Test-Path -LiteralPath $launcherPath)) { throw "Missing launcher: $launcherPath" }
$payload = [Convert]::ToBase64String([IO.File]::ReadAllBytes($launcherPath))
# Ship just the launcher over the authenticated connection. No scp/password
# storage or remote Git mutation is needed; the installed project is reused.
$remoteCommand = 'printf %s ' + $payload + ' | base64 -d | /usr/bin/env -u PYTHONHOME -u PYTHONPATH /usr/bin/python3 -u - --repo ' +
    (ConvertTo-ShArgument $RemoteRepo) + ' --public-ip ' + (ConvertTo-ShArgument $PublicIp) +
    ' --startup-timeout ' + $StartupTimeout
if ($remoteCommand.Length -gt 30000) { throw 'Remote launcher exceeds the Windows command-line size budget.' }
$sshArgs = @('-tt', '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15',
    '-o', 'ServerAliveCountMax=3', '-o', 'ConnectTimeout=20',
    '-L', '127.0.0.1:10000:127.0.0.1:10000', $Remote, $remoteCommand)

Write-Host "Workstation: $Remote"
Write-Host "Installed repository: $RemoteRepo"
Write-Host 'Starts Isaac, trajectory/teleoperation relay, markerless ROS bridge and learned registration.'
Write-Host 'ROS tunnel: Windows 127.0.0.1:10000 -> workstation 127.0.0.1:10000'
Write-Host "WebRTC: $PublicIp (TCP 49100 / UDP 47998; direct connection)"
Write-Host 'Unity and Play mode remain under your control. No robot motion is requested.'
if ($DryRun) {
    Write-Host "Dry run: no connection, services, tunnel or viewer started. Launcher payload: $($payload.Length) characters."
    return
}

$ssh = (Get-Command ssh.exe -ErrorAction Stop).Source
$portProbe = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, 10000)
$portProbe.Server.ExclusiveAddressUse = $true
try {
    $portProbe.Start()
} catch {
    throw 'Windows port 10000 is occupied. Reuse the running demo or close its existing SSH tunnel/mock endpoint before starting this launcher. Nothing was stopped.'
} finally {
    $portProbe.Stop()
}

if (-not $NoViewer) {
    $viewer = Get-Process -Name 'Isaac Sim WebRTC Streaming Client' -ErrorAction SilentlyContinue
    if (-not $viewer) {
        try { & (Join-Path $PSScriptRoot 'Start-IsaacStream.ps1') -Server $PublicIp }
        catch { Write-Warning "WebRTC viewer could not open: $_. Continuing with the ROS/Isaac stack." }
    }
}
Write-Host 'Authenticate to SSH if prompted. Wait for the launcher readiness message before entering Unity Play mode.'
Write-Host 'Keep this terminal open. Ctrl+C stops this launch and closes its tunnel; the viewer window may be closed separately.'
& $ssh @sshArgs
if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
    throw "Demo SSH session ended with code $LASTEXITCODE. Review the preflight/service messages above."
}
