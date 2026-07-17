#!/usr/bin/env python3
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


ITEMS = {
    "/item/food": "香蕉",
    "/item/daily": "毛巾",
    "/item/electronics": "手机",
}


class ItemHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        item_name = ITEMS.get(self.path)
        if item_name is None:
            self._send_json(404, {"code": 400, "result": ""})
            return
        self._send_json(200, {"code": 200, "result": item_name})

    def log_message(self, format_string, *args):
        print(
            "{} - {}".format(
                self.address_string(),
                format_string % args,
            )
        )

    def _send_json(self, status_code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    server = ThreadingHTTPServer(("0.0.0.0", 8000), ItemHandler)
    print("QR test server: http://0.0.0.0:8000")
    print("Food:       http://172.20.10.7:8000/item/food")
    print("Daily:      http://172.20.10.7:8000/item/daily")
    print("Electronics:http://172.20.10.7:8000/item/electronics")
    server.serve_forever()


if __name__ == "__main__":
    main()
