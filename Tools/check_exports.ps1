# Compares the DLL's ed_fm_* export list with the saved baseline.
param([string]$Dll = "$PSScriptRoot\..\EFM Source Code\bin\F_117_NightHawk.dll",
      [string]$Baseline = "$PSScriptRoot\..\build\refactor-baseline\exports.txt")
$db = (Get-ChildItem "C:\Program Files\Microsoft Visual Studio\2022\*\VC\Tools\MSVC\*\bin\Hostx64\x64\dumpbin.exe" | Select-Object -Last 1).FullName
$now = & $db /exports $Dll | ForEach-Object { if ($_ -match '\b(ed_fm_\w+)') { $Matches[1] } } | Sort-Object -Unique
$diff = Compare-Object (Get-Content $Baseline) $now
if ($diff) { $diff | Format-Table; exit 1 } else { "exports match baseline ($($now.Count))" }
