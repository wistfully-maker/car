"""Safe motion mode mapping for task states."""

IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
QR_SEARCH = "QR_SEARCH"
STOP_NAVIGATION = "STOP_NAVIGATION"


def motion_mode_for_state(state):
    if state in ("NAVIGATING_TO_PICKUP", "NAVIGATING_TO_WORKSHOP"):
        return NAVIGATION
    if state == "NAVIGATING_TO_SIM_WORKSHOP":
        # 仿真阶段由 stop 栈导航，只转发 /cmd_vel/stop 来源。
        return STOP_NAVIGATION
    if state == "WAITING_QR":
        return QR_SEARCH
    return IDLE
