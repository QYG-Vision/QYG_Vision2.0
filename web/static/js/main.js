(() => {
  "use strict";

  const colors = ["#42e5ff", "#58f7b2", "#ffd166", "#ff5f72", "#b98cff", "#62a0ff", "#ff8bd1"];
  const defaults = new Set(["fps", "frame_age_ms", "detector_wait_ms", "yaw_error", "pitch_error"]);
  const labels = {
    fps: "FPS", frame_age_ms: "帧延迟 ms", detector_wait_ms: "Detector ms",
    tracker_ms: "Tracker ms", planner_ms: "Planner ms", publisher_ms: "Publisher ms",
    armor_count: "装甲板数", detector_queue_depth: "检测队列", target_x: "目标 X m",
    target_y: "目标 Y m", target_z: "目标 Z m", target_vx: "目标 Vx m/s",
    target_vy: "目标 Vy m/s", target_vz: "目标 Vz m/s", target_yaw: "目标 Yaw °",
    target_yaw_speed: "目标角速度 rad/s", target_yaw_cmd: "目标 Yaw 指令 °",
    target_pitch_cmd: "目标 Pitch 指令 °", command_yaw: "发送 Yaw °",
    command_pitch: "发送 Pitch °", gimbal_yaw: "反馈 Yaw °", gimbal_pitch: "反馈 Pitch °",
    yaw_error: "Yaw 误差 °", pitch_error: "Pitch 误差 °", control: "Control", fire: "Fire"
  };

  const state = { health: {}, data: null, videoConnected: false, selected: new Set(defaults), inFlight: new Set() };
  const elements = {
    video: document.getElementById("video-stream"), videoShell: document.getElementById("video-shell"),
    json: document.getElementById("json-tree"), logUpdated: document.getElementById("log-updated"),
    controls: document.getElementById("series-controls"), maxPoints: document.getElementById("max-points"),
    mainShell: document.getElementById("main-chart").parentElement,
    detailShell: document.getElementById("detail-chart").parentElement,
    detailSeries: document.getElementById("detail-series")
  };

  const chartOptions = {
    responsive: true, maintainAspectRatio: false, animation: false, normalized: true,
    interaction: { intersect: false, mode: "nearest" },
    plugins: { legend: { labels: { color: "#9dbac8", boxWidth: 10 } } },
    scales: {
      x: { ticks: { color: "#7897a8", maxTicksLimit: 8 }, grid: { color: "rgba(66,229,255,.07)" } },
      y: { ticks: { color: "#7897a8" }, grid: { color: "rgba(66,229,255,.07)" } }
    }
  };
  const mainChart = new Chart(document.getElementById("main-chart"), { type: "line", data: { labels: [], datasets: [] }, options: chartOptions });
  const detailChart = new Chart(document.getElementById("detail-chart"), { type: "line", data: { labels: [], datasets: [] }, options: chartOptions });

  function setStatus(key, online) {
    const pill = document.getElementById(`${key}-status`);
    pill.classList.toggle("online", online);
    pill.classList.toggle("offline", !online);
  }

  function clearCharts() {
    state.data = null;
    for (const chart of [mainChart, detailChart]) { chart.data.labels = []; chart.data.datasets = []; chart.update("none"); }
    elements.mainShell.classList.remove("online");
    elements.detailShell.classList.remove("online");
  }

  function renderJson(value, root = elements.json) {
    root.replaceChildren();
    const walk = (item, prefix) => {
      if (item && typeof item === "object" && !Array.isArray(item)) {
        for (const [key, child] of Object.entries(item)) {
          if (child && typeof child === "object" && !Array.isArray(child)) {
            const group = document.createElement("div"); group.className = "json-group";
            group.textContent = prefix ? `${prefix}.${key}` : key; root.appendChild(group); walk(child, "");
          } else {
            const row = document.createElement("div"); row.className = "json-row";
            const name = document.createElement("span"); name.className = "json-key"; name.textContent = key;
            const output = document.createElement("span"); output.className = "json-value";
            output.textContent = child === null ? "—" : typeof child === "boolean" ? (child ? "TRUE" : "FALSE") : String(child);
            row.append(name, output); root.appendChild(row);
          }
        }
      }
    };
    walk(value, "");
  }

  function availableSeries(data) {
    return Object.keys(data).filter((key) => key !== "time" && key !== "schema_version" && Array.isArray(data[key]));
  }

  function ensureControls(keys) {
    if (elements.controls.childElementCount) return;
    keys.forEach((key) => {
      const label = document.createElement("label"); const input = document.createElement("input");
      input.type = "checkbox"; input.checked = state.selected.has(key); input.dataset.key = key;
      input.addEventListener("change", () => { input.checked ? state.selected.add(key) : state.selected.delete(key); updateCharts(); });
      label.append(input, ` ${labels[key] || key}`); elements.controls.appendChild(label);
      const option = document.createElement("option"); option.value = key; option.textContent = labels[key] || key; elements.detailSeries.appendChild(option);
    });
    elements.detailSeries.value = keys.includes("yaw_error") ? "yaw_error" : keys[0];
  }

  function datasetsFor(keys) {
    return keys.map((key, index) => ({
      label: labels[key] || key, data: state.data[key], borderColor: colors[index % colors.length],
      backgroundColor: "transparent", borderWidth: 1.6, pointRadius: 0, spanGaps: false, tension: 0.08
    }));
  }

  function updateCharts() {
    if (!state.data) return;
    const keys = availableSeries(state.data); ensureControls(keys);
    mainChart.data.labels = state.data.time; mainChart.data.datasets = datasetsFor(keys.filter((key) => state.selected.has(key))); mainChart.update("none");
    const detailKey = elements.detailSeries.value || keys[0];
    detailChart.data.labels = state.data.time; detailChart.data.datasets = datasetsFor([detailKey]); detailChart.update("none");
    elements.mainShell.classList.add("online"); elements.detailShell.classList.add("online");
  }

  async function fetchJson(url) {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(`${response.status}`);
    return response.json();
  }

  async function pollOnce(name, operation) {
    if (state.inFlight.has(name)) return;
    state.inFlight.add(name);
    try { await operation(); } finally { state.inFlight.delete(name); }
  }

  function pollHealth() {
    return pollOnce("health", async () => {
      try {
        state.health = await fetchJson("/health");
        ["video", "data", "log", "producer"].forEach((key) => setStatus(key, Boolean(state.health[key])));
        if (state.health.video && !state.videoConnected) connectVideo();
        if (!state.health.video) disconnectVideo();
      } catch (_) {
        ["video", "data", "log", "producer"].forEach((key) => setStatus(key, false)); disconnectVideo();
      }
    });
  }

  function connectVideo() {
    state.videoConnected = true; elements.video.src = `/video?t=${Date.now()}`; elements.videoShell.classList.add("online");
  }
  function disconnectVideo() {
    if (state.videoConnected) elements.video.removeAttribute("src");
    state.videoConnected = false; elements.videoShell.classList.remove("online");
  }
  elements.video.addEventListener("error", disconnectVideo);

  function pollLog() {
    return pollOnce("log", async () => {
      try { const value = await fetchJson("/log"); renderJson(value); elements.logUpdated.textContent = new Date().toLocaleTimeString(); }
      catch (_) { elements.json.innerHTML = '<div class="offline-placeholder">状态源离线，正在重连…</div>'; elements.logUpdated.textContent = "离线"; }
    });
  }
  function pollData() {
    return pollOnce("data", async () => {
      const points = Math.min(600, Math.max(10, Number(elements.maxPoints.value) || 200));
      try { state.data = await fetchJson(`/data?max_points=${points}`); updateCharts(); }
      catch (_) { clearCharts(); }
    });
  }

  document.getElementById("fullscreen-button").addEventListener("click", () => elements.videoShell.requestFullscreen?.());
  elements.detailSeries.addEventListener("change", updateCharts);
  elements.maxPoints.addEventListener("change", pollData);
  pollHealth(); pollLog(); pollData();
  setInterval(pollHealth, 1000); setInterval(pollLog, 500); setInterval(pollData, 500);
})();
