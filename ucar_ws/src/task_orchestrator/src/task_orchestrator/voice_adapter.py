"""ROS-independent recognized-speech to task-request adapter logic."""

from task_orchestrator.voice_parser import parse_categories


class VoiceTaskAdapterLogic:
    def __init__(self, clock, id_factory, debounce_seconds):
        if debounce_seconds < 0:
            raise ValueError("debounce_seconds must be non-negative")
        self._clock = clock
        self._id_factory = id_factory
        self._debounce_seconds = float(debounce_seconds)
        self._last_text = None
        self._last_time = None

    def build(self, raw_text):
        if not isinstance(raw_text, str) or not raw_text.strip():
            raise ValueError("voice text must be non-empty")
        text = raw_text.strip()
        now = self._clock()
        if (
            text == self._last_text
            and self._last_time is not None
            and now - self._last_time < self._debounce_seconds
        ):
            return None

        physical, simulation = parse_categories(text)
        payload = {
            "protocol_version": 1,
            "task_id": self._id_factory(),
            "physical_target_category": physical,
            "simulation_target_category": simulation,
            "raw_text": text,
        }
        self._last_text = text
        self._last_time = now
        return payload
