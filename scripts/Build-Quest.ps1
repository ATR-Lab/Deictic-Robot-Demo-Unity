# Close the editor for this project first, or pass a separate validation copy.
[CmdletBinding()]
param(
    [string]$ProjectPath = (Join-Path $PSScriptRoot '../Deictic-Robot-Demo'),
    [string]$EditorPath = 'C:/Program Files/Unity/Hub/Editor/6000.6.0f1/Editor/Unity.exe',
    [string]$OutputPath = (Join-Path $PSScriptRoot '../output/DeicticK1.apk')
)
$ErrorActionPreference = 'Stop'
$ProjectPath = (Resolve-Path -LiteralPath $ProjectPath).Path
$OutputPath = [IO.Path]::GetFullPath($OutputPath)
$outputDirectory = Split-Path -Parent $OutputPath
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
# This host's packaged-app TEMP path fails Java 17 Unix-domain socket connect.
# Unity strips JAVA_TOOL_OPTIONS from Gradle, so TEMP/TMP must also be scoped to
# this build. A non-redirected directory passed Selector.open() without JVM flags.
$javaSocketDirectory = Join-Path $env:PUBLIC 'DeicticJavaTemp'
New-Item -ItemType Directory -Path $javaSocketDirectory -Force | Out-Null
$previousJavaOptions = $env:JAVA_TOOL_OPTIONS
$previousOutput = $env:DEICTIC_ANDROID_OUTPUT
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
try {
    $env:TEMP = $javaSocketDirectory
    $env:TMP = $javaSocketDirectory
    $env:JAVA_TOOL_OPTIONS = ($previousJavaOptions + ' -Djdk.net.unixdomain.tmpdir=' + $javaSocketDirectory).Trim()
    $env:DEICTIC_ANDROID_OUTPUT = $OutputPath
    $buildLog = Join-Path $outputDirectory 'quest-build.log'
    $arguments = @('-batchmode', '-nographics', '-buildTarget', 'Android', '-projectPath', ('"'+$ProjectPath+'"'),
        '-executeMethod', 'Deictic.Editor.DemoSetup.BuildQuest', '-quit', '-logFile', ('"'+$buildLog+'"'))
    $process = Start-Process -FilePath $EditorPath -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $process.WaitForExit()
    if ($process.ExitCode -ne 0 -or -not (Test-Path -LiteralPath $OutputPath)) { throw "Quest build failed. Inspect $buildLog" }
    Write-Output "Quest development APK: $OutputPath"
} finally {
    $env:JAVA_TOOL_OPTIONS = $previousJavaOptions
    $env:DEICTIC_ANDROID_OUTPUT = $previousOutput
    $env:TEMP = $previousTemp
    $env:TMP = $previousTmp
}
