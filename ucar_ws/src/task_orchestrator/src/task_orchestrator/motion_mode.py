"""Safe motion mode mapping for task states."""

IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
QR_SEARCH = "QR_SEARCH"
STOP_NAVIGATION = "STOP_NAVIGATION"
LINE_FOLLOW = "LINE_FOLLOW"


def motion_mode_for_state(state):
    if state == "NAVIGATING_TO_PICKUP":
        return NAVIGATION
    if state in (
        "NAVIGATING_TO_WORKSHOP",
        "NAVIGATING_TO_SIM_WORKSHOP",
        "NAVIGATING_LINE_START",
    ):
        # 仿真与巡线起点阶段由 stop 栈导航，只转发 /cmd_vel/stop 来源。
        return STOP_NAVIGATION
    if state == "WAITING_QR":
        return QR_SEARCH
    if state == "LINE_FOLLOWING":
        return LINE_FOLLOW
    return IDLE
