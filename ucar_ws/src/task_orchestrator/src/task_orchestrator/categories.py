_CATEGORY_CONFIG = {
    "食品": {"label": "食品大类", "workshop": "食品加工车间"},
    "日用品": {"label": "日用品大类", "workshop": "日用品加工车间"},
    "电子产品": {
        "label": "电子产品大类",
        "workshop": "电子产品生产车间",
    },
}


def category_config(category):
    if category not in _CATEGORY_CONFIG:
        raise ValueError("unsupported category: %s" % category)
    return dict(_CATEGORY_CONFIG[category])


def format_result_speech(
    physical_item,
    physical_category,
    simulation_item,
    simulation_category,
):
    physical = category_config(physical_category)
    simulation = category_config(simulation_category)
    return (
        "取得%s属于%s应放置在%s，仿真环境中取得%s属于%s应放置在%s"
        % (
            physical_item,
            physical["label"],
            physical["workshop"],
            simulation_item,
            simulation["label"],
            simulation["workshop"],
        )
    )


def format_delivery_speech(selected_item, workshop):
    return "已将%s放入%s" % (selected_item, workshop)


def format_simulation_delivery_speech(selected_item, workshop):
    return "仿真任务已完成，已将%s放入%s" % (selected_item, workshop)
