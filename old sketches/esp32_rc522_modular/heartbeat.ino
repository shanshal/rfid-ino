void publishHeartbeatIfDue() {
  const unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_INTERVAL_MS) {
    return;
  }

  lastHeartbeatMs = now;
  publishStatus("alive");
}
