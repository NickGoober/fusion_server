# Forward Oracle fusion_server webhooks to the pose viewer on THIS PC.
#
# Prerequisites:
#   1. Pose viewer running locally:  cd vercel_pose_viewer && npm run dev
#   2. On Oracle, VERCEL_WEBHOOK_URL must be http://127.0.0.1:3000/api/gadget
#
# Usage:
#   .\scripts\webhook-tunnel.ps1
#   .\scripts\webhook-tunnel.ps1 -OracleHost 79.72.87.48
#
# Keep this window open. Fusion on Oracle POSTs to its localhost:3000,
# which SSH forwards to your Windows viewer on port 3000.

param(
    [string] $OracleHost = "79.72.87.48",
    [string] $IdentityFile = "C:\Users\carno\Desktop\RepreactStuff\ssh-key-2026-08-03.key"
)

if (-not (Test-Path -LiteralPath $IdentityFile)) {
    Write-Error "SSH key not found: $IdentityFile"
    exit 1
}

Write-Host "Forwarding Oracle localhost:3000 -> this PC localhost:3000"
Write-Host "Host: $OracleHost"
Write-Host "Key:  $IdentityFile"
Write-Host "Oracle webhook URL should be: http://127.0.0.1:3000/api/gadget"
Write-Host "Press Ctrl+C to stop."

ssh -i $IdentityFile -R 3000:127.0.0.1:3000 -N $OracleHost
