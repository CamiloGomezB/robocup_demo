# Lanza el demo 1 vs nadie compartiendo la red del robot virtual de Booster Studio.
# Uso: powershell -ExecutionPolicy Bypass -File .\sim_docker\start_demo.ps1 [-Mode striker|demo] [-KickMode walk|visual] [-Control topic|remote]
#   -Control remote: como en el robot real (solo se activa con el control remoto LB+RB+Y, ver deploy/README.md)
param(
    [string]$Image = "robocup-demo-sim",
    [string]$Cpus = "",
    [ValidateSet("striker", "demo")][string]$Mode = "striker",
    [ValidateSet("walk", "visual")][string]$KickMode = "walk",
    [ValidateSet("topic", "remote")][string]$Control = "topic"
)

$robot = docker ps --format "{{.Names}}" | Where-Object { $_ -match '-k1$' } | Select-Object -First 1
if (-not $robot) {
    $robot = docker ps --format "{{.Names}} {{.Image}}" | Where-Object { $_ -match 'virtual-robot' } |
        ForEach-Object { ($_ -split ' ')[0] } | Select-Object -First 1
}
if (-not $robot) {
    Write-Error "No hay robot virtual corriendo. Activa el K1 en Booster Studio (escena football_pitch_K1)."
    exit 1
}
Write-Host "Robot virtual: $robot"

docker rm -f robocup-demo 2>$null | Out-Null

$dockerArgs = @("run", "--rm", "-it", "--name", "robocup-demo", "--network", "container:$robot", "-e", "MODE=$Mode", "-e", "KICK_MODE=$KickMode", "-e", "CONTROL=$Control")
$envDomain = docker inspect $robot --format "{{range .Config.Env}}{{println .}}{{end}}" | Where-Object { $_ -match '^ROS_DOMAIN_ID=' }
if ($envDomain) { $dockerArgs += @("-e", $envDomain) }
if ($Cpus) { $dockerArgs += @("--cpus", $Cpus) }
$dockerArgs += $Image

& docker @dockerArgs
