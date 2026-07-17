def pyzbar_backend(image):
    from pyzbar.pyzbar import ZBarSymbol, decode

    results = decode(image, symbols=[ZBarSymbol.QRCODE])
    return [result.data.decode("utf-8").strip() for result in results]


class StableQrDecoder:
    def __init__(self, backend=pyzbar_backend, required_frames=2):
        if required_frames < 1:
            raise ValueError("required_frames must be at least 1")
        self._backend = backend
        self._required_frames = required_frames
        self._candidate = None
        self._count = 0
        self._seen = set()

    def process(self, image):
        values = self._backend(image)
        value = next(
            (value for value in values if value and value not in self._seen),
            None,
        )
        if value is None:
            self.reset_wall()
            return None

        if value != self._candidate:
            self._candidate = value
            self._count = 1
        else:
            self._count += 1

        if self._count < self._required_frames:
            return None

        self._seen.add(value)
        self.reset_wall()
        return value

    def reset_wall(self):
        self._candidate = None
        self._count = 0

    def reset_search(self):
        self.reset_wall()
        self._seen.clear()
