"""Safe motion mode mapping for task states."""

IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
QR_SEARCH = "QR_SEARCH"


def motion_mode_for_state(state):
    if state == "NAVIGATING_TO_PICKUP":
        return NAVIGATION
    if state == "WAITING_QR":
        return QR_SEARCH
    return IDLE
