param(
    [string]$Compiler = 'C:\msys64\ucrt64\bin\g++.exe'
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$testBinary = Join-Path $PSScriptRoot 'static_identify_fsm_test.exe'
$originalPath = $env:PATH

try {
    $env:PATH = (Split-Path $Compiler) + ';' + $originalPath
    & $Compiler `
        -std=c++17 -Wall -Wextra -Werror `
        "-I$repoRoot\Tests\stubs" `
        "-I$repoRoot\User\Algorithm\Inc" `
        "$repoRoot\Tests\static_identify_fsm_test.cpp" `
        "$repoRoot\User\Algorithm\Src\alg_fsm.cpp" `
        -o $testBinary
    if ($LASTEXITCODE -ne 0) {
        throw "Static-identify test compilation failed with exit code $LASTEXITCODE."
    }

    & $testBinary
    if ($LASTEXITCODE -ne 0) {
        throw "Static-identify tests failed with exit code $LASTEXITCODE."
    }
}
finally {
    $env:PATH = $originalPath
    if (Test-Path -LiteralPath $testBinary) {
        Remove-Item -LiteralPath $testBinary
    }
}
