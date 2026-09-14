#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Spark X2 ROS adapter for protocol-v1 dual-target classification."""

import json
import os
import re
from concurrent.futures import ThreadPoolExecutor

import requests
import rospy
from std_msgs.msg import String

from llm_spark.protocol import (
    ProtocolError,
    build_error_result,
    build_result_from_items,
    build_selection_prompt,
    parse_request,
)


SYSTEM_PROMPT = """你是智慧工厂机器人分类模块。
产品类别和车间只有：
- 食品 -> 食品加工车间
- 日用品 -> 日用品加工车间
- 电子产品 -> 电子产品生产车间

根据目标母类从候选列表选择唯一物品。只输出要求的 JSON，不要解释或使用
Markdown。"""


def clean_json(text):
    value = text.strip()
    value = re.sub(r"^```(?:json)?\s*", "", value)
    value = re.sub(r"\s*```$", "", value)
    return value.strip()


class SparkLLMNode:
    def __init__(self):
        rospy.init_node("spark_llm_node")
        self.url = rospy.get_param(
            "~url",
            "https://spark-api-open.xf-yun.com/x2/chat/completions",
        )
        self.request_timeout = float(
            rospy.get_param("~request_timeout", 90.0)
        )
        self.api_password = rospy.get_param(
            "~api_password",
            os.environ.get("SPARK_API_PASSWORD", ""),
        ).strip()
        if not self.api_password:
            raise RuntimeError(
                "Spark credential missing: set ~api_password or "
                "SPARK_API_PASSWORD"
            )
        self.result_pub = rospy.Publisher(
            "/llm/classify/result",
            String,
            queue_size=10,
        )
        self.request_sub = rospy.Subscriber(
            "/llm/classify/request",
            String,
            self.callback,
            queue_size=1,
        )
        rospy.loginfo("Spark X2 dual-target LLM node started")

    def callback(self, message):
        request = None
        try:
            request = parse_request(message.data)
            result = self.call_spark(request)
        except Exception as error:
            rospy.logerr("LLM processing failed: %s", error)
            if request is None:
                try:
                    raw = json.loads(message.data)
                except (TypeError, ValueError):
                    return
                request = {
                    "task_id": raw.get("task_id"),
                    "request_id": raw.get("request_id"),
                }
            try:
                result = build_error_result(request, str(error))
            except ProtocolError:
                return
        self.result_pub.publish(String(
            data=json.dumps(result, ensure_ascii=False)
        ))

    def call_spark(self, request):
        with ThreadPoolExecutor(max_workers=2) as executor:
            physical_future = executor.submit(
                self.call_selection,
                request,
                request["physical_target_category"],
            )
            simulation_future = executor.submit(
                self.call_selection,
                request,
                request["simulation_target_category"],
            )
            physical_item = physical_future.result()
            simulation_item = simulation_future.result()
        return build_result_from_items(
            request,
            physical_item,
            simulation_item,
        )

    def call_selection(self, request, target_category):
        response = requests.post(
            self.url,
            headers={
                "Authorization": "Bearer " + self.api_password,
                "Content-Type": "application/json",
            },
            json={
                "model": "spark-x",
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {
                        "role": "user",
                        "content": build_selection_prompt(
                            request,
                            target_category,
                        ),
                    },
                ],
            },
            timeout=self.request_timeout,
        )
        response.raise_for_status()
        response_json = response.json()
        answer = response_json["choices"][0]["message"]["content"]
        model_result = json.loads(clean_json(answer))
        if not isinstance(model_result, dict):
            raise ProtocolError("model result must be an object")
        return model_result.get("selected_item")


if __name__ == "__main__":
    try:
        SparkLLMNode()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError) as error:
        rospy.logfatal("%s", error)
