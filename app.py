# encoding:utf-8
from flask import Flask, request, render_template_string
import requests

app = Flask(__name__)

BAIDU_AK = "fhAUhbYIJrWLWvzeCgl2RDRN92OBCaX8"

# 模拟 LoRa 收到的 GPS 原始坐标（WGS84）
SIMULATED_LORA_GPS = {
    "lat": "39.551000",
    "lng": "117.396000"
}

def convert_to_baidu_coords(gps_lng, gps_lat):
    """
    将 GPS 坐标 (WGS84) 转换为 百度坐标 (BD09)
    """
    url = "https://api.map.baidu.com/geoconv/v2/"
    params = {
        "coords": f"{gps_lng},{gps_lat}",
        "from": 1,       # 1: GPS 坐标（WGS84）
        "to": 5,         # 5: 百度坐标（BD09）
        "ak": BAIDU_AK,
    }

    resp = requests.get(url, params=params)
    if resp.status_code == 200:
        result = resp.json()
        if result.get("status") == 0:
            bd_point = result["result"][0]
            return f"{bd_point['y']},{bd_point['x']}"  # 注意顺序：纬度,经度
        else:
            print("坐标转换失败:", result.get("message"))
    else:
        print("请求 geoconv 接口失败")
    return None


def get_lora_origin():
    """
    从 LoRa 接收 GPS 坐标并转换为百度坐标
    """

    lat = SIMULATED_LORA_GPS["lat"]
    lng = SIMULATED_LORA_GPS["lng"]
    return convert_to_baidu_coords(gps_lng=lng, gps_lat=lat)


@app.route('/')
def index():
    print("已接收到 LoRa GPS 坐标")
    return render_template_string(open("index.html", encoding="utf-8").read())


@app.route('/submit', methods=['POST'])
def submit():
    lng = request.form.get('lng')
    lat = request.form.get('lat')

    if not lng or not lat:
        return "<h1>错误：缺少经纬度参数</h1>"

    # 终点：来自网页（视为百度坐标）
    destination = f"{lat},{lng}"

    # 起点：从 LoRa 获取 GPS 坐标并转换
    origin = get_lora_origin()
    if not origin:
        return "<h1>坐标转换失败</h1>"

    # 请求百度步行路线 API
    url = "https://api.map.baidu.com/direction/v2/walking"
    params = {
        "origin": origin,
        "destination": destination,
        "ak": BAIDU_AK,
    }

    response = requests.get(url=url, params=params)

    if response.status_code != 200:
        return f"<h1>请求失败</h1><p>状态码: {response.status_code}</p>"

    data = response.json()
    if data.get("status") != 0:
        return f"<h1>接口错误</h1><p>{data.get('message')}</p>"

    routes = data.get("result", {}).get("routes", [])
    if not routes:
        return "<h1>没有可用路线</h1>"

    steps = routes[0].get("steps", [])
    html_output = f"<h1>起点（来自 LoRa GPS → 百度坐标）：{origin}</h1>"
    html_output += f"<h2>终点：{destination}</h2>"
    html_output += f"<h3>总共 {len(steps)} 步:</h3>"

    for idx, step in enumerate(steps, 1):
        instruction = step.get("instructions", "无描述")
        distance = step.get("distance", 0)
        duration = step.get("duration", 0)
        path_str = step.get("path", "")
        points = path_str.split(";")

        html_output += f"<h4>步骤 {idx}</h4>"
        html_output += f"<p><b>指令:</b> {instruction}</p>"
        html_output += f"<p><b>距离:</b> {distance} 米 | <b>耗时:</b> {duration} 秒</p>"
        html_output += f"<details><summary>路径点（{len(points)}）</summary><ul>"

        for p in points[:3] + (["..."] if len(points) > 6 else []) + points[-3:]:
            html_output += f"<li>{p}</li>"
        html_output += "</ul></details><hr>"

    return html_output


if __name__ == '__main__':

    app.run(debug=True)
