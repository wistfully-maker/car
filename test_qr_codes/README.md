# QR item test fixtures

Start the local JSON service from the repository root:

```powershell
python tools/qr_test_server.py
```

The QR codes use the computer's current car-network address `172.20.10.7`:

| File | URL | JSON item | Category |
| --- | --- | --- | --- |
| `food_banana.png` | `/item/food` | 香蕉 | 食品 |
| `daily_towel.png` | `/item/daily` | 毛巾 | 日用品 |
| `electronics_phone.png` | `/item/electronics` | 手机 | 电子产品 |

If the computer's IP changes, update `tools/generate_test_qr.py` and regenerate
the images before testing.
