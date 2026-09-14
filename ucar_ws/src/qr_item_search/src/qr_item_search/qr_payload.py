from urllib.parse import urlparse

import requests


class InvalidQrUrl(ValueError):
    pass


class InvalidPayload(ValueError):
    pass


def validate_url(qr_content):
    if not isinstance(qr_content, str):
        raise InvalidQrUrl(str(qr_content))
    normalized = qr_content.strip()
    parsed = urlparse(normalized)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise InvalidQrUrl(normalized)
    return normalized


class ItemResolver:
    def __init__(
        self,
        session=None,
        retries=0,
        connect_timeout=1.0,
        read_timeout=2.0,
    ):
        if retries < 0:
            raise ValueError("retries must be non-negative")
        self.session = session or requests.Session()
        self.retries = retries
        self.timeout = (connect_timeout, read_timeout)

    def resolve(self, qr_content):
        normalized = validate_url(qr_content)

        for attempt in range(self.retries + 1):
            try:
                response = self.session.get(normalized, timeout=self.timeout)
                response.raise_for_status()
                payload = response.json()
            except (requests.RequestException, ValueError):
                if attempt == self.retries:
                    raise
                continue

            if not isinstance(payload, dict):
                raise InvalidPayload("payload must be an object")
            if payload.get("code") != 200:
                raise InvalidPayload("unexpected business code")

            result = payload.get("result")
            if not isinstance(result, str) or not result.strip():
                raise InvalidPayload("result must be a non-empty string")
            return result.strip()
