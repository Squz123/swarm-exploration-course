param([ValidateSet('Fast','Baseline','Online')][string]$Mode='Fast')
$ErrorActionPreference='Stop'
$projectRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$localFile=Join-Path $projectRoot 'local-settings.json'
$settings=$null
if (Test-Path -LiteralPath $localFile) {
    $settings=Get-Content -Raw -LiteralPath $localFile | ConvertFrom-Json
}
$distro=$env:COURSE_WSL_DISTRO
if (-not $distro -and $settings -and $settings.distribution) { $distro=$settings.distribution }
if (-not $distro) { $distro='Ubuntu-20.04' }
if ($settings -and $settings.rosPort) {
    $port=[int]$settings.rosPort
    if ($port -lt 1024 -or $port -gt 65535) { throw 'rosPort must be between 1024 and 65535.' }
    if (-not $env:COURSE_ROS_PORT) { $env:COURSE_ROS_PORT=$port.ToString() }
}
$wsl=Join-Path $env:WINDIR 'System32\wsl.exe'
if (-not (Test-Path -LiteralPath $wsl)) { throw 'WSL is not installed. See the README Windows setup.' }
# Native stderr can contain a harmless WSL proxy notice: check exit codes.
$ErrorActionPreference='Continue'
$converted=@(& $wsl -d $distro --exec wslpath -u $projectRoot)
$convertCode=$LASTEXITCODE
if ($convertCode -ne 0) {
    Write-Error 'Cannot convert the project path. Check the distribution name in local-settings.json.'
    exit $convertCode
}
$linuxRoot=($converted -join "`n").Replace([string][char]0,'').Trim()
if (-not $linuxRoot.StartsWith('/')) { Write-Error 'WSL returned an unexpected path.'; exit 2 }
$script=switch ($Mode) { 'Fast' { 'run_fast.sh' } 'Online' { 'run_online.sh' } default { 'run_trial.sh' } }
Write-Host "Running $Mode in WSL distribution $distro"
$forward=@()
foreach ($name in @('COURSE_ROS_PORT','COURSE_GUI','COURSE_ONLINE_PAUSE_UAV','COURSE_ONLINE_PAUSE_AT','COURSE_ONLINE_PAUSE_FOR','COURSE_ONLINE_DROP_ROUND','COURSE_ONLINE_TIMEOUT')) {
    $value=[Environment]::GetEnvironmentVariable($name)
    if ($value) { $forward+="$name=$value" }
}
if ($Mode -eq 'Baseline') { & $wsl -d $distro --exec env @forward bash "$linuxRoot/setup/ros/$script" B }
else { & $wsl -d $distro --exec env @forward bash "$linuxRoot/setup/ros/$script" }
exit $LASTEXITCODE
