#!/usr/bin/env python3
from pathlib import Path

import qrcode
from qrcode.constants import ERROR_CORRECT_H


QR_CODES = {
    "food_banana.png": "http://172.20.10.7:8000/item/food",
    "daily_towel.png": "http://172.20.10.7:8000/item/daily",
    "electronics_phone.png": "http://172.20.10.7:8000/item/electronics",
}


def main():
    output_dir = Path(__file__).resolve().parents[1] / "test_qr_codes"
    output_dir.mkdir(parents=True, exist_ok=True)
    for filename, url in QR_CODES.items():
        qr = qrcode.QRCode(
            version=None,
            error_correction=ERROR_CORRECT_H,
            box_size=16,
            border=4,
        )
        qr.add_data(url)
        qr.make(fit=True)
        image = qr.make_image(fill_color="black", back_color="white")
        output_path = output_dir / filename
        image.save(output_path)
        print("{} -> {}".format(output_path, url))


if __name__ == "__main__":
    main()
