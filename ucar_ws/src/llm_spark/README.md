# llm_spark

讯飞星火 X2 的 ROS 1 Noetic 双目标分类节点。订阅编排器的 protocol v1 请求，
从三个 QR 候选中分别选择实物目标和仿真目标，并返回带原始 identity 的结果。

## 启动

凭据不保存在源码中。启动前通过环境变量或 ROS 私有参数提供：

```bash
export SPARK_API_PASSWORD='API_KEY:API_SECRET'
roslaunch llm_spark llm_spark.launch
```

也可仅对本次进程设置：

```bash
SPARK_API_PASSWORD='API_KEY:API_SECRET' \
  roslaunch llm_spark llm_spark.launch
```

小车部署使用私有文件时：

```bash
export SPARK_API_PASSWORD="$(cat ~/.config/ucar/spark_api_password)"
roslaunch llm_spark llm_spark.launch
```

## Topic

- 订阅：`/llm/classify/request` (`std_msgs/String`)
- 发布：`/llm/classify/result` (`std_msgs/String`)

请求必须包含：

- `protocol_version: 1`
- `task_id`
- `request_id`
- `physical_target_category`
- `simulation_target_category`
- 三个连续编号的 `candidates`

成功结果包含 `physical` 和 `simulation` 两个选择；错误结果保留 `task_id` 和
`request_id`，并返回 `status: error`。

## 参数

- `~url`：Spark X2 HTTP 地址；
- `~request_timeout`：单次 HTTP 超时，默认 30 秒；
- `~api_password`：可选私有参数，优先于 `SPARK_API_PASSWORD`。

## 测试

```bash
python3 -m unittest discover \
  -s src/llm_spark/test \
  -p "test_*.py" -v
```
