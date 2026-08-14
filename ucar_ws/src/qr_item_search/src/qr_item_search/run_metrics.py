"""Structured per-run metrics for stop-and-scan QR searches.

Each completed search appends one JSON line to ``qr_search_runs.jsonl``
inside ``metrics_dir``.  Writes are short and never raise; failures are
reported through the optional warning callback.
"""
import json
import os

METRICS_FILENAME = "qr_search_runs.jsonl"


def append_run_record(metrics_dir, record, warning=None):
    """Append ``record`` as one UTF-8 JSON line; create dirs as needed.

    Directory creation and the append happen inside the function so a
    single call never leaves the metrics file half-written.  Any failure
    is reported through ``warning`` and swallowed.
    """
    warning = warning or (lambda _: None)
    if not isinstance(metrics_dir, str) or not metrics_dir.strip():
        warning("metrics dir must be a non-empty string")
        return
    try:
        os.makedirs(metrics_dir, exist_ok=True)
        line = json.dumps(record, ensure_ascii=False) + "\n"
        path = os.path.join(metrics_dir, METRICS_FILENAME)
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(line)
    except Exception as error:
        warning("metrics write failed: %s" % error)
