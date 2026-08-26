# Forward Oracle fusion_server webhooks to the pose viewer on THIS PC.
#
# Prerequisites:
#   1. Pose viewer running locally:  cd vercel_pose_viewer && npm run dev
#   2. On Oracle, VERCEL_WEBHOOK_URL must be http://127.0.0.1:3000/api/gadget
#
# Usage:
#   .\scripts\webhook-tunnel.ps1 -OracleHost ubuntu@your-oracle-ip
#
# Keep this window open while testing. Fusion on Oracle POSTs to its own
# localhost:3000, which SSH forwards to your Windows viewer on port 3000.

param(
    [Parameter(Mandatory = $true)]
    [string] $OracleHost
)

Write-Host "Forwarding Oracle localhost:3000 -> this PC localhost:3000"
Write-Host "Oracle webhook URL should be: http://127.0.0.1:3000/api/gadget"
Write-Host "Press Ctrl+C to stop."
ssh -R 3000:127.0.0.1:3000 -N $OracleHost
