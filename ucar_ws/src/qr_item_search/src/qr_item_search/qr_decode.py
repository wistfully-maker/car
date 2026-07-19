def pyzbar_backend(image):
    from pyzbar.pyzbar import ZBarSymbol, decode

    results = decode(image, symbols=[ZBarSymbol.QRCODE])
    return [result.data.decode("utf-8").strip() for result in results]


class UniqueQrDecoder:
    """Return each non-empty QR URL once per search."""

    def __init__(self, backend=pyzbar_backend):
        self._backend = backend
        self._seen = set()

    def process(self, image):
        values = self._backend(image)
        new_values = []
        pending = set()
        for value in values:
            if not isinstance(value, str):
                raise TypeError("QR backend values must be strings")
            value = value.strip()
            if value and value not in self._seen and value not in pending:
                new_values.append(value)
                pending.add(value)
        self._seen.update(pending)
        return new_values

    def reset_search(self):
        self._seen.clear()
