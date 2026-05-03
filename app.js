const $ = (id) => document.getElementById(id);

const state = {
  ws: null,
  telemetry: null,
  vehicles: [],
  selectedVehicle: null,
  mode: "待机",
  trafficEnabled: false,
  lastFrame: 0,
  roadOffset: 0,
};

const sceneCanvas = $("sceneCanvas");
const ctx = sceneCanvas.getContext("2d");

function fmt(value, digits = 0) {
  const n = Number(value);
  if (!Number.isFinite(n)) return "--";
  return n.toFixed(digits);
}

function pct(value) {
  return `${Math.round((Number(value) || 0) * 100)}%`;
}

function degFromSteer(value) {
  return Math.round((Number(value) || 0) * 540);
}

function setText(id, value) {
  const el = $(id);
  if (el) el.textContent = value;
}

function setClass(el, classes) {
  if (!el) return;
  el.className = classes;
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(await res.text());
  return res.json();
}

async function loadVehicles() {
  try {
    const data = await fetch("/api/vehicles").then((r) => r.json());
    state.vehicles = data.vehicles || [];
    const select = $("vehicleSelect");
    select.innerHTML = "";
    state.vehicles.forEach((vehicle) => {
      const option = document.createElement("option");
      option.value = vehicle.file;
      option.textContent = vehicle.name;
      select.appendChild(option);
    });
    state.selectedVehicle = state.vehicles.find((v) => v.name === "Tesla Model 3") || state.vehicles[0];
    if (state.selectedVehicle) {
      select.value = state.selectedVehicle.file;
      updateVehicleFacts(state.selectedVehicle);
    }
  } catch (err) {
    console.warn("loadVehicles failed", err);
  }
}

function updateVehicleFacts(vehicle) {
  if (!vehicle) return;
  setText("vehicleName", vehicle.name);
  setText("massValue", vehicle.massKg ? `${vehicle.massKg} kg` : "-- kg");
  setText("cdValue", vehicle.cd ?? "--");
}

function connectWs() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const wsPort = window.DASHBOARD_CONFIG?.wsPort || 8765;
  const url = `${protocol}//${location.hostname}:${wsPort}`;
  const ws = new WebSocket(url);
  state.ws = ws;

  ws.onopen = () => {
    setClass($("netState"), "pill ok");
    setText("netState", "后端在线");
    setText("wsState", "已连接");
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      state.telemetry = data;
      updateDashboard(data);
    } catch (err) {
      console.warn("bad websocket payload", err);
    }
  };

  ws.onclose = () => {
    setClass($("netState"), "pill danger");
    setText("netState", "后端断开");
    setText("wsState", "重连中");
    setTimeout(connectWs, 1200);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function updateDashboard(data) {
  const live = data.status === "live";
  const stale = data.status === "stale";

  setClass($("simState"), live ? "pill ok" : stale ? "pill warn" : "pill");
  setText("simState", live ? "Carla 实时" : stale ? "真值超时" : "仿真未接入");
  setText("downlinkState", live ? "接收中" : "等待 UDP 5003");
  setText("uplinkState", "UDP 5001");

  setText("speedValue", Math.round(data.speedKmh || 0));
  setText("gearValue", data.gear || "P");
  setText("steerValue", `${degFromSteer(data.steer)}°`);
  setText("throttleValue", pct(data.throttle));
  setText("brakeValue", pct(data.brake));
  setText("speedLimit", data.speedLimit ? `${data.speedLimit} km/h` : "--");
  setText("trafficLight", normalizeTrafficLight(data.trafficLight));
  setText("radarTargets", data.radarTargets ?? 0);
  setText("collisionActor", data.collision?.Actor || "None");
  setText("weatherValue", data.weather || "--");
  setText("trafficValue", data.traffic || "未加载");
  setText("modeText", data.mode || state.mode);

  const position = data.position || {};
  setText("posX", `${fmt(position.x, 2)} m`);
  setText("posY", `${fmt(position.y, 2)} m`);
  setText("posZ", `${fmt(position.z, 2)} m`);

  const attitude = data.attitude || {};
  setText("pitchValue", `${fmt(attitude.pitch, 1)}°`);
  setText("yawValue", `${fmt(attitude.yaw, 1)}°`);
  setText("rollValue", `${fmt(attitude.roll, 1)}°`);

  const rpm = data.wheelRpm || [0, 0, 0, 0];
  ["rpmFl", "rpmFr", "rpmRl", "rpmRr"].forEach((id, index) => setText(id, fmt(rpm[index], 0)));
  const loads = data.wheelLoad || [0, 0, 0, 0];
  ["loadFl", "loadFr", "loadRl", "loadRr"].forEach((id, index) => setText(id, `${fmt(loads[index], 0)} N`));

  updateGauge(data.speedKmh || 0);
  updateAdas(data);
  updateSensors(data);
  updateSceneCompliance(data.sceneCompliance);
}

function normalizeTrafficLight(value) {
  const text = String(value || "Unknown");
  if (text.includes("Red")) return "红灯";
  if (text.includes("Yellow")) return "黄灯";
  if (text.includes("Green")) return "绿灯";
  if (text.includes("Off")) return "关闭";
  return text.replace("TrafficLightState.", "");
}

function updateGauge(speed) {
  const degree = Math.max(0, Math.min(260, speed / 200 * 260));
  $("speedGauge").style.background = `conic-gradient(var(--cyan) 0deg ${degree}deg, rgba(53, 201, 255, 0.18) ${degree}deg 270deg, rgba(255,255,255,0.06) 270deg 360deg)`;
}

function updateAdas(data) {
  const mode = data.mode || "待机";
  const speed = Number(data.speedKmh || 0);
  const radar = Number(data.radarTargets || 0);
  const brake = Number(data.brake || 0);

  setChip("accChip", speed > 5 && mode !== "待机", mode === "智驾" ? "跟车巡航" : "可用");
  setChip("aebChip", radar > 15 || brake > 0.8, radar > 15 ? "介入" : "监测");
  setChip("lkaChip", mode === "智驾", mode === "智驾" ? "保持中" : "可用");
  setChip("tsrChip", true, "识别中");
  setChip("noaChip", mode === "智驾", mode === "智驾" ? "激活" : "未激活");
  setChip("driverChip", true, "正常");

  if (radar > 15) $("aebChip").className = "status-chip alert";
}

function setChip(id, active, label) {
  const chip = $(id);
  if (!chip) return;
  chip.className = active ? "status-chip ok" : "status-chip";
  const b = chip.querySelector("b");
  if (b) b.textContent = label;
}

function updateSensors(data) {
  const stale = data.status !== "live";
  const radarWarn = Number(data.radarTargets || 0) > 30;
  const gnss = Array.isArray(data.gnss) ? data.gnss : [];
  const gnssOk = gnss.some((v) => Math.abs(Number(v) || 0) > 0.0001);

  sensorText("cameraStatus", stale ? "待接入" : "正常", stale ? "warn" : "ok");
  sensorText("lidarStatus", stale ? "待接入" : "正常", stale ? "warn" : "ok");
  sensorText("radarStatus", radarWarn ? "目标密集" : stale ? "待接入" : "正常", radarWarn ? "warn" : stale ? "warn" : "ok");
  sensorText("gnssStatus", gnssOk ? "RTK 正常" : stale ? "待接入" : "搜星中", gnssOk ? "ok" : "warn");
  sensorText("ultraStatus", stale ? "待接入" : "正常", stale ? "warn" : "ok");
  setText("sensorHealth", stale ? "0/5" : "5/5");
}

function updateSceneCompliance(sceneCompliance = {}) {
  const categoryLabels = {
    vehicle_models: "车型",
    traffic_standards: "交通标准",
    barriers: "路障",
    covers: "覆盖物",
    manholes: "井盖",
    normal_vehicles: "普通对手车",
    emergency_vehicles: "紧急对手车",
    walkers: "行人",
    bicycles: "自行车",
    animals: "动物",
  };
  const categories = sceneCompliance.compliance?.categories || sceneCompliance.categories || {};
  const entries = Object.entries(categories);
  const passed = entries.filter(([, item]) => item.satisfied).length;
  setText("sceneComplianceScore", `${passed}/${entries.length || 10}`);

  const list = $("sceneComplianceList");
  if (!list) return;
  list.innerHTML = "";
  if (!entries.length) {
    const empty = document.createElement("span");
    empty.className = "scene-badge warn";
    empty.textContent = "待加载";
    list.appendChild(empty);
    return;
  }
  entries.forEach(([key, item]) => {
    const badge = document.createElement("span");
    badge.className = item.satisfied ? "scene-badge ok" : "scene-badge warn";
    badge.textContent = `${categoryLabels[key] || key} ${item.actual}/${item.required}`;
    list.appendChild(badge);
  });
}

function sensorText(id, text, klass) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  el.className = klass;
}

async function sendControl() {
  const payload = {
    steer: Number($("steerInput").value) / 100,
    throttle: Number($("throttleInput").value) / 100,
    brake: Number($("brakeInput").value) / 100,
    reverse: false,
    hand_brake: $("parkBtn").classList.contains("active"),
  };
  setText("steerInputValue", `${$("steerInput").value}%`);
  setText("throttleInputValue", `${$("throttleInput").value}%`);
  setText("brakeInputValue", `${$("brakeInput").value}%`);
  try {
    await postJson("/api/control", payload);
  } catch (err) {
    console.warn("control failed", err);
  }
}

function drawScene(time) {
  const data = state.telemetry || {};
  const width = sceneCanvas.width;
  const height = sceneCanvas.height;
  const speed = Number(data.speedKmh || 0);
  const steer = Number(data.steer || 0);
  const weather = String(data.weather || "");
  const night = weather.includes("深夜");
  const rain = weather.includes("雨");
  const fog = weather.includes("雾");

  state.roadOffset = (state.roadOffset + (speed / 90 + 0.2) * ((time - state.lastFrame) || 16) * 0.06) % 80;
  state.lastFrame = time;

  const sky = ctx.createLinearGradient(0, 0, 0, height * 0.55);
  if (night) {
    sky.addColorStop(0, "#08101c");
    sky.addColorStop(1, "#152436");
  } else if (rain || fog) {
    sky.addColorStop(0, "#536572");
    sky.addColorStop(1, "#253541");
  } else {
    sky.addColorStop(0, "#1c5f85");
    sky.addColorStop(1, "#8fb9c5");
  }
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, width, height);

  drawSunAndWeather(width, height, { night, rain, fog, time });

  ctx.fillStyle = night ? "#12251b" : "#1f5633";
  ctx.beginPath();
  ctx.moveTo(0, height * 0.52);
  ctx.lineTo(width * 0.28, height * 0.34);
  ctx.lineTo(width * 0.52, height * 0.50);
  ctx.lineTo(width * 0.72, height * 0.36);
  ctx.lineTo(width, height * 0.52);
  ctx.lineTo(width, height);
  ctx.lineTo(0, height);
  ctx.closePath();
  ctx.fill();

  const road = ctx.createLinearGradient(0, height * 0.44, 0, height);
  road.addColorStop(0, "#34414b");
  road.addColorStop(1, "#151c24");
  ctx.fillStyle = road;
  ctx.beginPath();
  ctx.moveTo(width * 0.42 + steer * 40, height * 0.47);
  ctx.lineTo(width * 0.58 + steer * 40, height * 0.47);
  ctx.lineTo(width * 0.91, height);
  ctx.lineTo(width * 0.09, height);
  ctx.closePath();
  ctx.fill();

  drawLaneLines(width, height, steer);
  drawHudOverlay(width, height, data);
  drawEgoCar(width, height, steer);
  drawTrafficObjects(width, height, data);

  if (fog) {
    ctx.fillStyle = "rgba(210, 228, 230, 0.20)";
    ctx.fillRect(0, 0, width, height);
  }

  requestAnimationFrame(drawScene);
}

function drawSunAndWeather(width, height, opts) {
  if (opts.night) {
    ctx.fillStyle = "#eaf6ff";
    ctx.beginPath();
    ctx.arc(width * 0.83, height * 0.16, 24, 0, Math.PI * 2);
    ctx.fill();
  } else {
    ctx.fillStyle = "#ffd36a";
    ctx.beginPath();
    ctx.arc(width * 0.82, height * 0.14, 30, 0, Math.PI * 2);
    ctx.fill();
  }

  if (opts.rain) {
    ctx.strokeStyle = "rgba(190, 230, 255, 0.55)";
    ctx.lineWidth = 2;
    for (let i = 0; i < 80; i += 1) {
      const x = (i * 73 + opts.time * 0.4) % width;
      const y = (i * 37 + opts.time * 0.7) % height;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x - 8, y + 26);
      ctx.stroke();
    }
  }
}

function drawLaneLines(width, height, steer) {
  ctx.strokeStyle = "rgba(236, 246, 255, 0.78)";
  ctx.lineWidth = 5;
  ctx.setLineDash([34, 46]);
  ctx.lineDashOffset = -state.roadOffset;

  ctx.beginPath();
  ctx.moveTo(width * 0.5 + steer * 30, height * 0.49);
  ctx.lineTo(width * 0.5, height);
  ctx.stroke();

  ctx.setLineDash([]);
  ctx.strokeStyle = "rgba(255, 213, 104, 0.78)";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(width * 0.40 + steer * 40, height * 0.50);
  ctx.lineTo(width * 0.20, height);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(width * 0.60 + steer * 40, height * 0.50);
  ctx.lineTo(width * 0.80, height);
  ctx.stroke();
}

function drawEgoCar(width, height, steer) {
  const x = width * 0.5;
  const y = height * 0.82;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(steer * 0.08);
  ctx.fillStyle = "#19232c";
  roundRect(ctx, -76, -92, 152, 184, 18);
  ctx.fill();
  ctx.fillStyle = "#35c9ff";
  roundRect(ctx, -46, -62, 92, 82, 12);
  ctx.fill();
  ctx.fillStyle = "#0b1118";
  roundRect(ctx, -60, 28, 120, 42, 10);
  ctx.fill();
  ctx.fillStyle = "#fff3c1";
  roundRect(ctx, -58, -88, 34, 12, 5);
  roundRect(ctx, 24, -88, 34, 12, 5);
  ctx.fill();
  ctx.restore();
}

function drawTrafficObjects(width, height, data) {
  const count = Math.min(6, Number(data.radarTargets || 0));
  for (let i = 0; i < count; i += 1) {
    const y = height * (0.48 + i * 0.055);
    const scale = 0.45 + i * 0.08;
    const x = width * (0.42 + (i % 3) * 0.08);
    ctx.fillStyle = i % 2 ? "#ffbd52" : "#6aa7ff";
    roundRect(ctx, x - 34 * scale, y - 16 * scale, 68 * scale, 32 * scale, 6);
    ctx.fill();
  }
}

function drawHudOverlay(width, height, data) {
  ctx.fillStyle = "rgba(8, 12, 17, 0.50)";
  roundRect(ctx, 18, 18, 250, 82, 8);
  ctx.fill();
  ctx.fillStyle = "#ecf6ff";
  ctx.font = "700 24px Microsoft YaHei";
  ctx.fillText(`${Math.round(data.speedKmh || 0)} km/h`, 36, 54);
  ctx.fillStyle = "#8fa3b7";
  ctx.font = "16px Microsoft YaHei";
  ctx.fillText(`${data.weather || "天气未设置"} / ${data.traffic || "交通未加载"}`, 36, 82);
}

function roundRect(context, x, y, width, height, radius) {
  context.beginPath();
  context.moveTo(x + radius, y);
  context.arcTo(x + width, y, x + width, y + height, radius);
  context.arcTo(x + width, y + height, x, y + height, radius);
  context.arcTo(x, y + height, x, y, radius);
  context.arcTo(x, y, x + width, y, radius);
  context.closePath();
}

function bindUi() {
  $("vehicleSelect").addEventListener("change", (event) => {
    state.selectedVehicle = state.vehicles.find((v) => v.file === event.target.value);
    updateVehicleFacts(state.selectedVehicle);
  });

  $("deployBtn").addEventListener("click", async () => {
    if (!state.selectedVehicle) return;
    setText("vehicleStatus", "下发中");
    try {
      await postJson("/api/spawn", {
        file: state.selectedVehicle.file,
        vehicle_blueprint: state.selectedVehicle.id,
        mode: state.mode,
      });
      setText("vehicleStatus", "已下发");
    } catch (err) {
      setText("vehicleStatus", "失败");
      console.warn(err);
    }
  });

  document.querySelectorAll("[data-weather]").forEach((button) => {
    button.addEventListener("click", async () => {
      document.querySelectorAll("[data-weather]").forEach((b) => b.classList.remove("active"));
      button.classList.add("active");
      await postJson("/api/weather", { weather: button.dataset.weather }).catch(console.warn);
    });
  });

  $("trafficBtn").addEventListener("click", async () => {
    state.trafficEnabled = !state.trafficEnabled;
    $("trafficBtn").textContent = state.trafficEnabled ? "清空交通参与者" : "加载交通参与者";
    await postJson("/api/traffic", { enabled: state.trafficEnabled }).catch(console.warn);
  });

  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", async () => {
      document.querySelectorAll("[data-mode]").forEach((b) => b.classList.remove("active"));
      button.classList.add("active");
      state.mode = button.dataset.mode;
      await postJson("/api/mode", { mode: state.mode }).catch(console.warn);
    });
  });

  ["steerInput", "throttleInput", "brakeInput"].forEach((id) => {
    $(id).addEventListener("input", sendControl);
  });

  $("centerSteerBtn").addEventListener("click", () => {
    $("steerInput").value = 0;
    sendControl();
  });

  $("parkBtn").addEventListener("click", () => {
    $("parkBtn").classList.toggle("active");
    $("brakeInput").value = $("parkBtn").classList.contains("active") ? 100 : 0;
    sendControl();
  });
}

function updateClock() {
  const now = new Date();
  setText("clock", now.toLocaleTimeString("zh-CN", { hour12: false }));
}

async function pollFallback() {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) return;
  try {
    const data = await fetch("/api/state").then((r) => r.json());
    state.telemetry = data;
    updateDashboard(data);
  } catch (err) {
    setClass($("netState"), "pill danger");
    setText("netState", "后端离线");
  }
}

async function init() {
  window.DASHBOARD_CONFIG = window.DASHBOARD_CONFIG || {};
  bindUi();
  updateClock();
  setInterval(updateClock, 1000);
  await loadVehicles();
  connectWs();
  setInterval(pollFallback, 1000);
  requestAnimationFrame(drawScene);
}

init();
