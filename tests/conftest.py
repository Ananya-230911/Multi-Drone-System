"""Test setup: switch CrewAI's telemetry off before anything imports it (it tries to reach the network and can hang)."""

import os

os.environ.setdefault("CREWAI_DISABLE_TELEMETRY", "true")
os.environ.setdefault("OTEL_SDK_DISABLED", "true")
