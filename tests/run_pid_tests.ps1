param([string]$Compiler = 'g++')

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    New-Item -ItemType Directory -Force -Path 'build/host-tests' | Out-Null
    $common = @('-std=c++14', '-Wall', '-Wextra', '-pedantic')
    $integrationExe = 'build/host-tests/yuntai_pid_integration_test.exe'
    $gravityExe = 'build/host-tests/gravity_compensation_test.exe'
    $sources = @(
        'tests/yuntai_pid_integration_test.cpp',
        'applications/mission/mission.cpp',
        'applications/data_interfaces/can_task.cpp',
        'sp_middleware/tools/pid/pid.cpp',
        'sp_middleware/tools/math_tools/math_tools.cpp',
        'sp_middleware/motor/rm_motor/rm_motor.cpp'
    )
    # Match the firmware's math constant and supply the host's size_t definition.
    & $Compiler @common '-include' 'cstddef' '-DM_PI=3.14159265358979323846' `
        '-Itests/fakes' '-Iapplications' '-Isp_middleware' @sources '-o' $integrationExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration test compilation failed.' }
    & $integrationExe
    if ($LASTEXITCODE -ne 0) { throw 'Integration tests failed.' }

    & $Compiler @common 'tests/gravity_compensation_test.cpp' '-o' $gravityExe
    if ($LASTEXITCODE -ne 0) { throw 'Gravity test compilation failed.' }
    & $gravityExe
    if ($LASTEXITCODE -ne 0) { throw 'Gravity tests failed.' }
}
finally {
    Pop-Location
}
