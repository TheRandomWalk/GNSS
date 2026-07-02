param(
    [string]$BuildDir = "build",
    [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$cmakeArgs = @(
    "-S", ".",
    "-B", $BuildDir,
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=$Config"
)

$cl = Get-Command cl -ErrorAction SilentlyContinue
$vcvars = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
if (-not $cl -and (Test-Path $vcvars)) {
    $cmd = "call `"$vcvars`" && cmake $($cmakeArgs -join ' ') && cmake --build $BuildDir --config $Config && ctest --test-dir $BuildDir --output-on-failure -C $Config"
    cmd /c $cmd
} else {
    cmake @cmakeArgs
    cmake --build $BuildDir --config $Config
    ctest --test-dir $BuildDir --output-on-failure -C $Config
}
