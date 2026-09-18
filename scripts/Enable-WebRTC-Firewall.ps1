# Run from an elevated PowerShell. Rules are limited to the workstation peer.
[CmdletBinding(SupportsShouldProcess)]
param(
    [Parameter(Mandatory)]
    [ValidateNotNullOrEmpty()]
    [ValidateScript({ -not [string]::IsNullOrWhiteSpace($_) })]
    [string[]]$RemoteAddress
)
$ErrorActionPreference = 'Stop'
$principal = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $WhatIfPreference -and -not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Windows requires an elevated PowerShell to change firewall rules. Re-run this script as Administrator.'
}
foreach ($protocol in @('TCP', 'UDP')) {
    $ports = @('47995-48012', '49000-49007')
    if ($protocol -eq 'TCP') { $ports += '49100' }
    foreach ($direction in @('Inbound', 'Outbound')) {
        $ruleName = "Deictic-K1-WebRTC-$protocol-$direction"
        $parameters = @{
            Name = $ruleName; DisplayName = $ruleName; Group = 'Deictic K1 simulation'
            Direction = $direction; Action = 'Allow'; Enabled = 'True'; Profile = 'Any'
            Protocol = $protocol; RemoteAddress = $RemoteAddress
        }
        if ($direction -eq 'Inbound') { $parameters.LocalPort = $ports }
        else { $parameters.RemotePort = $ports }
        if ($PSCmdlet.ShouldProcess("$ruleName ($($RemoteAddress -join ', '))", 'Allow WebRTC peer ports')) {
            if (Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue) {
                $parameters.Remove('Name'); $parameters.Remove('DisplayName'); $parameters.Remove('Group')
                Set-NetFirewallRule -Name $ruleName @parameters | Out-Null
            } else { New-NetFirewallRule @parameters | Out-Null }
        }
    }
}
if ($WhatIfPreference) { Write-Output 'Preview complete; no firewall rules changed.' }
else { Get-NetFirewallRule -Group 'Deictic K1 simulation' | Select-Object Name,Enabled,Direction,Action }
