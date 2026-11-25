# ESP32-C3 烧录脚本
#
# 用 PlatformIO 自己的烧录流程（偏移量/模式都正确），
# 靠 platformio.ini 里的 upload_flags = --no-stub 绕过 C3 的 stub flasher 问题。
#
# 用法：
#   .\flash.ps1
#
# 如果报 "port busy"，脚本会自动清理残留的 esptool 进程

$ErrorActionPreference = "Stop"

$pio  = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
$proj = $PSScriptRoot

Write-Host "[1/2] 清理残留的 esptool 进程..." -ForegroundColor Cyan
Get-CimInstance Win32_Process |
    Where-Object { $_.Name -eq 'esptool.exe' } |
    ForEach-Object {
        Write-Host "  结束 PID $($_.ProcessId)" -ForegroundColor DarkGray
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 800

Write-Host "[2/2] 编译并烧录..." -ForegroundColor Cyan
& $pio run -d $proj -t upload

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "烧录失败。试试：" -ForegroundColor Yellow
    Write-Host "  1. 按住板子的 BOOT 键，点一下 RST，松开 BOOT"
    Write-Host "  2. 重新运行本脚本"
}
