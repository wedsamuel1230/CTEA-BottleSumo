# BottleSumo Connection Test Script
# Run this AFTER connecting to BottleSumo_AP WiFi

Write-Host "=== BottleSumo Connection Diagnostics ===" -ForegroundColor Cyan
Write-Host ""

# Step 1: Check WiFi adapter status
Write-Host "[1] Checking WiFi adapter..." -ForegroundColor Yellow
$wifi = Get-NetAdapter | Where-Object {$_.Name -like "*Wi-Fi*" -and $_.Status -eq "Up"}
if ($wifi) {
    Write-Host "    WiFi adapter is UP" -ForegroundColor Green
    $ipconfig = Get-NetIPAddress -InterfaceAlias $wifi.Name -AddressFamily IPv4 -ErrorAction SilentlyContinue
    if ($ipconfig) {
        Write-Host "    Your IP: $($ipconfig.IPAddress)" -ForegroundColor Green
        if ($ipconfig.IPAddress -like "192.168.42.*") {
            Write-Host "    ✓ Connected to BottleSumo_AP network!" -ForegroundColor Green
        } else {
            Write-Host "    ✗ NOT connected to BottleSumo_AP (wrong IP range)" -ForegroundColor Red
            Write-Host "    Expected: 192.168.42.x" -ForegroundColor Yellow
        }
    }
} else {
    Write-Host "    ✗ WiFi adapter not found or disconnected" -ForegroundColor Red
}

Write-Host ""

# Step 2: Ping Pico W
Write-Host "[2] Pinging Pico W (192.168.42.1)..." -ForegroundColor Yellow
$ping = Test-Connection -ComputerName 192.168.42.1 -Count 2 -Quiet
if ($ping) {
    Write-Host "    ✓ Pico W is reachable!" -ForegroundColor Green
} else {
    Write-Host "    ✗ Cannot ping Pico W" -ForegroundColor Red
    Write-Host "    Make sure:" -ForegroundColor Yellow
    Write-Host "      - Pico W is powered on" -ForegroundColor Yellow
    Write-Host "      - Code is uploaded and running" -ForegroundColor Yellow
    Write-Host "      - You're connected to BottleSumo_AP WiFi" -ForegroundColor Yellow
}

Write-Host ""

# Step 3: Test TCP port
Write-Host "[3] Testing TCP port 5000..." -ForegroundColor Yellow
try {
    $tcpClient = New-Object System.Net.Sockets.TcpClient
    $connect = $tcpClient.BeginConnect("192.168.42.1", 5000, $null, $null)
    $wait = $connect.AsyncWaitHandle.WaitOne(3000, $false)
    
    if ($wait -and $tcpClient.Connected) {
        Write-Host "    ✓ TCP port 5000 is OPEN and accepting connections!" -ForegroundColor Green
        $tcpClient.Close()
    } else {
        Write-Host "    ✗ Cannot connect to TCP port 5000" -ForegroundColor Red
        Write-Host "    Check Serial Monitor for server status" -ForegroundColor Yellow
        $tcpClient.Close()
    }
} catch {
    Write-Host "    ✗ Connection failed: $($_.Exception.Message)" -ForegroundColor Red
}

Write-Host ""
Write-Host "=== Diagnostics Complete ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "If all checks pass, run: python test_client_gui.py" -ForegroundColor Green
