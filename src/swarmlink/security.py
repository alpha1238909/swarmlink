"""Optional HMAC packet authentication and bounded replay protection.

HMAC authenticates packets but does not encrypt telemetry. Treat shared keys as
secrets and rotate them outside source code before real deployments.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import secrets

from .core import NodeTelemetry

SIGNED_PROTOCOL_VERSION = "swarmlink.signed-telemetry.v1"


class PacketAuthenticationError(ValueError):
    pass


class ReplayDetectedError(PacketAuthenticationError):
    pass


class PacketAuthenticator:
    """Signs and verifies canonical telemetry envelopes with a shared secret."""

    def __init__(self, secret: bytes | str, *, max_age_s: float = 10.0, max_future_skew_s: float = 1.0, replay_cache_size: int = 4096) -> None:
        if isinstance(secret, str):
            secret = secret.encode("utf-8")
        if len(secret) < 16:
            raise ValueError("use a secret of at least 16 bytes")
        if max_age_s <= 0 or max_future_skew_s < 0 or replay_cache_size <= 0:
            raise ValueError("invalid packet-authentication configuration")
        self._secret = secret
        self.max_age_s = max_age_s
        self.max_future_skew_s = max_future_skew_s
        self.replay_cache_size = replay_cache_size
        self._seen: dict[str, float] = {}

    @staticmethod
    def _canonical(payload: dict[str, object]) -> bytes:
        return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")

    def _signature(self, payload: dict[str, object]) -> str:
        return hmac.new(self._secret, self._canonical(payload), hashlib.sha256).hexdigest()

    def encode(self, telemetry: NodeTelemetry, *, issued_at_s: float, nonce: str | None = None) -> bytes:
        unsigned: dict[str, object] = {
            "type": SIGNED_PROTOCOL_VERSION,
            "issued_at_s": issued_at_s,
            "nonce": nonce or secrets.token_urlsafe(18),
            "telemetry": telemetry.to_dict(),
        }
        return self._canonical({**unsigned, "signature": self._signature(unsigned)})

    def decode(self, packet: bytes | str, *, now_s: float) -> NodeTelemetry:
        if isinstance(packet, bytes):
            packet = packet.decode("utf-8")
        try:
            envelope = json.loads(packet)
        except json.JSONDecodeError as error:
            raise PacketAuthenticationError("packet is not valid JSON") from error
        if not isinstance(envelope, dict) or envelope.get("type") != SIGNED_PROTOCOL_VERSION:
            raise PacketAuthenticationError("unsupported signed packet type")
        signature = envelope.get("signature")
        if not isinstance(signature, str):
            raise PacketAuthenticationError("packet has no signature")
        unsigned = {key: value for key, value in envelope.items() if key != "signature"}
        if not hmac.compare_digest(signature, self._signature(unsigned)):
            raise PacketAuthenticationError("packet signature is invalid")
        try:
            issued_at_s = float(unsigned["issued_at_s"])
            nonce = str(unsigned["nonce"])
            telemetry_payload = unsigned["telemetry"]
        except (KeyError, TypeError, ValueError) as error:
            raise PacketAuthenticationError("packet fields are invalid") from error
        if now_s - issued_at_s > self.max_age_s or issued_at_s - now_s > self.max_future_skew_s:
            raise PacketAuthenticationError("packet is outside accepted time window")
        if nonce in self._seen:
            raise ReplayDetectedError("packet nonce has already been accepted")
        if not isinstance(telemetry_payload, dict):
            raise PacketAuthenticationError("packet telemetry is invalid")
        self._seen[nonce] = issued_at_s
        self._prune(now_s)
        return NodeTelemetry.from_dict(telemetry_payload)

    def _prune(self, now_s: float) -> None:
        self._seen = {nonce: timestamp for nonce, timestamp in self._seen.items() if now_s - timestamp <= self.max_age_s}
        if len(self._seen) > self.replay_cache_size:
            for nonce, _ in sorted(self._seen.items(), key=lambda item: item[1])[: len(self._seen) - self.replay_cache_size]:
                del self._seen[nonce]
