# three_body_sim2 调用说明

`three_body_sim2.c` 是新版 HTTP 引擎，默认监听：

```text
http://127.0.0.1:60002/
```

## 编译

```powershell
cd 'E:\codex专\TB参数空间'
gcc -O3 -fopenmp -std=c11 -Wall -Wextra -o three_body_engine2.exe three_body_sim2.c -lws2_32 -lm
```

如果不想用 OpenMP，也可以去掉 `-fopenmp`，只是会慢。

## 启动

```powershell
.\three_body_engine2.exe
```

健康检查：

```text
http://127.0.0.1:60002/health
```

## 参数

五个三体自由参数：

```text
y2    第二个天体的 y
vx2   第二个天体的 vx
vy2   第二个天体的 vy
vx3   第三个天体的 vx
vy3   第三个天体的 vy
```

渲染范围参数：

```text
xmin  左下角 x
ymin  左下角 y
xmax  右上角 x
ymax  右上角 y
```

图像大小：

```text
width
height
```

网格映射规则：

```text
x3 = xmin + col * (xmax - xmin) / (width - 1)
y3 = ymax - row * (ymax - ymin) / (height - 1)
```

积分器：

```text
integrator=legacy  旧版算法，和 three_body_sim.c 一致
integrator=rk4     四阶 Runge-Kutta，用于高精度对照
```

输出：

```text
output=png   输出 sim_output_percentile_inverted.png
output=data  输出 sim_result.txt 风格的文本数据
```

## 旧版等价参数

旧版参数：

```python
params = {
    "y2": 100.1,
    "vx2": 0,
    "vy2": 20,
    "vx3": 1,
    "vy3": 0,
}
```

在新版里要填成：

```python
params = {
    "y2": 100.1,
    "vx2": 0,
    "vy2": 20,
    "vx3": 1,
    "vy3": 0,
    "xmin": -500,
    "ymin": -499,
    "xmax": 499,
    "ymax": 500,
    "width": 1000,
    "height": 1000,
    "integrator": "legacy",
    "output": "png",
}
```

这组参数会复刻旧版网格：

```text
左上角:  x=-500, y=500
中心点:  x=0,    y=0
右下角:  x=499,  y=-499
```

## Python 保存 PNG 示例

```python
import requests

params = {
    "y2": 100.1,
    "vx2": 0,
    "vy2": 20,
    "vx3": 1,
    "vy3": 0,
    "xmin": -500,
    "ymin": -499,
    "xmax": 499,
    "ymax": 500,
    "width": 1000,
    "height": 1000,
    "integrator": "legacy",
    "output": "png",
}

response = requests.get("http://127.0.0.1:60002/", params=params, timeout=None)
response.raise_for_status()

with open("v2_100.1c0c20c1c0.png", "wb") as f:
    f.write(response.content)
```

## Python 保存数据示例

```python
import requests

params = {
    "y2": 100.1,
    "vx2": 0,
    "vy2": 20,
    "vx3": 1,
    "vy3": 0,
    "xmin": -500,
    "ymin": -499,
    "xmax": 499,
    "ymax": 500,
    "width": 1000,
    "height": 1000,
    "integrator": "legacy",
    "output": "data",
}

response = requests.get("http://127.0.0.1:60002/", params=params, timeout=None)
response.raise_for_status()

with open("v2_sim_result.txt", "w", encoding="utf-8") as f:
    f.write(response.text)
```

## 高精度对照示例

只需要把：

```python
"integrator": "legacy"
```

改成：

```python
"integrator": "rk4"
```

其余参数保持不变即可。
