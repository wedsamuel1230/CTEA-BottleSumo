# Connect to BottleSumo_AP WiFi
# Run this as Administrator if needed

Write-Host "Attempting to connect to BottleSumo_AP..." -ForegroundColor Cyan

# Show available networks
Write-Host "`nScanning for WiFi networks..." -ForegroundColor Yellow
netsh wlan show networks

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "To connect manually:" -ForegroundColor Yellow
Write-Host "1. Open WiFi settings (Win + A)" -ForegroundColor White
Write-Host "2. Select 'BottleSumo_AP'" -ForegroundColor White
Write-Host "3. Enter password: sumobot123456" -ForegroundColor White
Write-Host "========================================`n" -ForegroundColor Cyan

# Note: Automatic connection via PowerShell requires a WiFi profile
# Easiest to connect manually via GUI for first-time connection
