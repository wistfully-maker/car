"""Safe motion mode mapping for task states."""

IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
QR_SEARCH = "QR_SEARCH"


def motion_mode_for_state(state):
    if state in (
        "NAVIGATING_TO_PICKUP",
        "NAVIGATING_TO_WORKSHOP",
        "NAVIGATING_TO_SIM_WORKSHOP",
    ):
        return NAVIGATION
    if state == "WAITING_QR":
        return QR_SEARCH
    return IDLE
