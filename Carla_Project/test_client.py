import requests
import json

url = "http://127.0.0.1:8080/api/spawn" # 注意这里路径匹配了

frontend_payload = {
  "vehicle_blueprint": "sedan_tesla_model3", # 确保你同目录下有 sedan_tesla_model3.json
  "params": {
      "curb_weight_kg": 2000.0,
      "drag_coefficient_cd": 0.35
  }
}

print("💻 [Client] 正在向中枢服务器发送发车请求...")
try:
    response = requests.post(url, json=frontend_payload)
    print("\n📩 [Client] 收到服务器的回执:")
    print(json.dumps(response.json(), indent=4, ensure_ascii=False))
except Exception as e:
    print(f"❌ 请求失败: {e}")
