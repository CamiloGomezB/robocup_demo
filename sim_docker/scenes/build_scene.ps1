# Empaqueta una escena de Booster Studio (.bscene) a partir de su carpeta fuente.
# Uso: powershell -ExecutionPolicy Bypass -File .\sim_docker\scenes\build_scene.ps1 [-Scene gray_floor_K1]
# Resultado: sim_docker\scenes\<Scene>.bscene, listo para importar desde Booster Studio.
param(
    [string]$Scene = "gray_floor_K1"
)
$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$src = Join-Path $root $Scene
$out = Join-Path $root "$Scene.bscene"
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) "bscene_$Scene"

if (-not (Test-Path (Join-Path $src "manifest.json"))) { throw "No existe $src\manifest.json" }

# El manifest exige que el SHA-256 del thumbnail coincida
$thumbHash = (Get-FileHash (Join-Path $src "thumbnail.webp") -Algorithm SHA256).Hash.ToLower()
$manifest = Get-Content (Join-Path $src "manifest.json") -Raw | ConvertFrom-Json
if ($manifest.thumbnail.sha256 -ne $thumbHash) {
    throw "El sha256 del thumbnail en manifest.json no coincide. Usa: $thumbHash"
}
if ($manifest.id -ne $Scene) { throw "manifest.id ($($manifest.id)) tiene que ser igual al nombre del archivo ($Scene)" }

if (Test-Path $tmp) { Remove-Item -Recurse -Force $tmp }
New-Item -ItemType Directory -Force $tmp | Out-Null

# 1) model.mjspec.zip: el XML de MuJoCo y sus texturas, en la raiz del zip
Compress-Archive -Path (Join-Path $src "model\*") -DestinationPath (Join-Path $tmp "model.zip")
Rename-Item (Join-Path $tmp "model.zip") "model.mjspec.zip"

# 2) El paquete: manifest, extensiones, modelo y thumbnail en la raiz
Copy-Item (Join-Path $src "manifest.json"), (Join-Path $src "extensions.json"), (Join-Path $src "thumbnail.webp") $tmp
$pkg = Join-Path $tmp "package.zip"
Compress-Archive -Path (Join-Path $tmp "manifest.json"), (Join-Path $tmp "extensions.json"), (Join-Path $tmp "model.mjspec.zip"), (Join-Path $tmp "thumbnail.webp") -DestinationPath $pkg
Move-Item -Force $pkg $out
Remove-Item -Recurse -Force $tmp

Write-Host "Escena lista: $out"
