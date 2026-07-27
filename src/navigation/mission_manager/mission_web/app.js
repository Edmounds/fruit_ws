"use strict";

const elements = {
  select: document.querySelector("#mission-select"),
  refresh: document.querySelector("#refresh-button"),
  reload: document.querySelector("#reload-button"),
  save: document.querySelector("#save-button"),
  currentFile: document.querySelector("#current-file"),
  message: document.querySelector("#message"),
  canvas: document.querySelector("#path-canvas"),
  plotPanel: document.querySelector("#plot-panel"),
  plotEmpty: document.querySelector("#plot-empty"),
  rows: document.querySelector("#point-rows"),
  pointCount: document.querySelector("#point-count"),
  waypointMode: document.querySelector("#waypoint-mode-button"),
  targetMode: document.querySelector("#target-mode-button"),
  mapAdd: document.querySelector("#map-add-button"),
};

const state = {
  filename: "",
  document: null,
  points: [],
  selectedIndex: -1,
  dirty: false,
  screenPoints: [],
  plotBounds: null,
  addKind: "waypoint",
  adding: false,
};

function setBusy(busy) {
  elements.select.disabled = busy;
  elements.refresh.disabled = busy;
  elements.reload.disabled = busy || !state.filename;
  elements.save.disabled = busy || !state.filename || !state.document;
  elements.mapAdd.disabled = busy || !state.document;
}

function setMessage(text = "", kind = "") {
  elements.message.textContent = text;
  elements.message.className = `message${kind ? ` ${kind}` : ""}`;
}

function updateStatus(message = "") {
  elements.currentFile.textContent = state.filename || "未选择文件";
  elements.currentFile.title = state.filename;
  if (message) {
    setMessage(message);
  } else if (state.dirty) {
    setMessage("有未保存的修改", "dirty");
  } else {
    setMessage("");
  }
}

async function request(url, options) {
  const response = await fetch(url, options);
  if (!response.ok) {
    let message = `请求失败 (${response.status})`;
    try {
      const payload = await response.json();
      if (payload.error) message = payload.error;
    } catch (_) {
      // Keep the HTTP fallback when the response is not JSON.
    }
    throw new Error(message);
  }
  return response;
}

async function refreshMissions(preferredName = state.filename) {
  setBusy(true);
  try {
    const response = await request("/api/missions");
    const payload = await response.json();
    const missions = Array.isArray(payload.missions) ? payload.missions : [];
    elements.select.replaceChildren();

    if (!missions.length) {
      const option = new Option("没有 YAML 文件", "");
      elements.select.add(option);
      clearMission("Mission 目录中没有 YAML 文件");
      return;
    }

    for (const filename of missions) {
      elements.select.add(new Option(filename, filename));
    }
    const selected = missions.includes(preferredName) ? preferredName : missions[0];
    elements.select.value = selected;
    if (selected !== state.filename || !state.document) {
      await loadMission(selected);
    } else {
      updateStatus("文件列表已刷新");
    }
  } catch (error) {
    setMessage(error.message, "error");
  } finally {
    setBusy(false);
  }
}

function clearMission(message = "") {
  state.filename = "";
  state.document = null;
  state.points = [];
  state.selectedIndex = -1;
  state.dirty = false;
  state.adding = false;
  updateAddControls();
  renderTable();
  drawPlot();
  updateStatus(message);
}

async function loadMission(filename) {
  if (!filename) return;
  setBusy(true);
  setMessage("正在加载...");
  try {
    const response = await request(`/api/missions/${encodeURIComponent(filename)}`);
    const yamlText = await response.text();
    const document = jsyaml.load(yamlText);
    if (!document || typeof document !== "object" || Array.isArray(document)) {
      throw new Error("YAML 顶层必须是对象");
    }

    state.filename = filename;
    state.document = document;
    state.points = collectPoints(document);
    state.selectedIndex = state.points.length ? 0 : -1;
    state.dirty = false;
    state.adding = false;
    updateAddControls();
    renderTable();
    drawPlot();
    updateStatus();
  } catch (error) {
    state.document = null;
    state.points = [];
    state.selectedIndex = -1;
    state.adding = false;
    updateAddControls();
    renderTable();
    drawPlot();
    setMessage(error.message, "error");
  } finally {
    setBusy(false);
  }
}

function collectPoints(document) {
  const points = [];
  const addPoint = (value, label, kind, location = {}) => {
    if (!value || typeof value !== "object" || Array.isArray(value)) return;
    points.push({
      ref: value,
      label,
      kind,
      ...location,
      values: {
        x: value.x == null ? "" : String(value.x),
        y: value.y == null ? "" : String(value.y),
        yaw: value.yaw == null ? "" : String(value.yaw),
      },
    });
  };

  addPoint(document.start, "start", "start");
  if (Array.isArray(document.targets)) {
    document.targets.forEach((target, targetIndex) => {
      if (!target || typeof target !== "object") return;
      if (Array.isArray(target.waypoints)) {
        target.waypoints.forEach((waypoint, waypointIndex) => {
          addPoint(waypoint, `target ${targetIndex} waypoint ${waypointIndex}`, "waypoint", {
            targetIndex,
            waypointIndex,
          });
        });
      }
      addPoint(target.pose, `target ${targetIndex} pose`, "target", { targetIndex });
    });
  }
  if (Array.isArray(document.goal_waypoints)) {
    document.goal_waypoints.forEach((waypoint, index) => {
      addPoint(waypoint, `goal waypoint ${index}`, "goalWaypoint", {
        goalWaypointIndex: index,
      });
    });
  }
  addPoint(document.goal, "goal", "goal");
  return points;
}

function renderTable() {
  elements.rows.replaceChildren();
  state.points.forEach((point, index) => {
    const row = document.createElement("tr");
    row.dataset.index = String(index);
    row.classList.toggle("selected", index === state.selectedIndex);

    const indexCell = document.createElement("td");
    indexCell.className = "index-col";
    indexCell.textContent = String(index);
    row.append(indexCell);

    const typeCell = document.createElement("td");
    typeCell.className = "type";
    const typeLabel = document.createElement("span");
    typeLabel.className = "point-type";
    const marker = document.createElement("span");
    marker.className = `point-marker ${pointColorKind(point)}`;
    const label = document.createElement("span");
    label.textContent = point.label;
    typeLabel.append(marker, label);
    typeCell.append(typeLabel);
    typeCell.title = point.label;
    row.append(typeCell);

    for (const field of ["x", "y", "yaw"]) {
      const cell = document.createElement("td");
      const input = document.createElement("input");
      input.type = "text";
      input.inputMode = "decimal";
      input.value = point.values[field];
      input.dataset.field = field;
      input.setAttribute("aria-label", `${point.label} ${field}`);
      input.addEventListener("input", () => editPoint(index, field, input));
      input.addEventListener("focus", () => selectPoint(index));
      cell.append(input);
      row.append(cell);
    }

    const actionsCell = document.createElement("td");
    actionsCell.className = "actions-col";
    const actions = document.createElement("div");
    actions.className = "row-actions";
    actions.append(
      makeActionButton("↑", "向前移动", !canMovePoint(point, -1), () => movePoint(index, -1)),
      makeActionButton("↓", "向后移动", !canMovePoint(point, 1), () => movePoint(index, 1)),
      makeActionButton("×", "删除点", false, () => deletePoint(index), "delete-button"),
    );
    actionsCell.append(actions);
    row.append(actionsCell);

    row.addEventListener("click", () => selectPoint(index));
    elements.rows.append(row);
  });
  elements.pointCount.textContent = `${state.points.length} 个点`;
}

function editPoint(index, field, input) {
  state.points[index].values[field] = input.value;
  const valid = isFiniteNumber(input.value);
  input.classList.toggle("invalid", !valid);
  if (valid) state.points[index].ref[field] = Number(input.value);
  state.dirty = true;
  selectPoint(index, false);
  updateStatus();
  drawPlot();
}

function pointColorKind(point) {
  if (point.kind === "waypoint" || point.kind === "goalWaypoint") return "waypoint";
  if (point.kind === "target" || point.kind === "goal") return "target";
  return "start";
}

function makeActionButton(text, title, disabled, action, extraClass = "") {
  const button = document.createElement("button");
  button.type = "button";
  button.className = `icon-button ${extraClass}`.trim();
  button.textContent = text;
  button.title = title;
  button.setAttribute("aria-label", title);
  button.disabled = disabled;
  button.addEventListener("click", (event) => {
    event.stopPropagation();
    action();
  });
  return button;
}

function canMovePoint(point, direction) {
  if (point.kind === "target") {
    const targets = Array.isArray(state.document?.targets) ? state.document.targets : [];
    return point.targetIndex + direction >= 0 && point.targetIndex + direction < targets.length;
  }
  if (point.kind === "waypoint") {
    const waypoints = state.document?.targets?.[point.targetIndex]?.waypoints || [];
    return point.waypointIndex + direction >= 0 && point.waypointIndex + direction < waypoints.length;
  }
  if (point.kind === "goalWaypoint") {
    const waypoints = state.document?.goal_waypoints || [];
    return point.goalWaypointIndex + direction >= 0 && point.goalWaypointIndex + direction < waypoints.length;
  }
  return false;
}

function rebuildPoints(selectedRef, fallbackIndex = 0) {
  state.points = collectPoints(state.document);
  const selectedIndex = state.points.findIndex((point) => point.ref === selectedRef);
  state.selectedIndex = selectedIndex >= 0
    ? selectedIndex
    : Math.min(Math.max(fallbackIndex, 0), state.points.length - 1);
  state.dirty = true;
  renderTable();
  drawPlot();
  updateStatus();
}

function movePoint(index, direction) {
  try {
    validatePoints();
    const point = state.points[index];
    if (!point || !canMovePoint(point, direction)) return;
    if (point.kind === "target") {
      swapItems(state.document.targets, point.targetIndex, point.targetIndex + direction);
    } else if (point.kind === "waypoint") {
      swapItems(
        state.document.targets[point.targetIndex].waypoints,
        point.waypointIndex,
        point.waypointIndex + direction,
      );
    } else if (point.kind === "goalWaypoint") {
      swapItems(
        state.document.goal_waypoints,
        point.goalWaypointIndex,
        point.goalWaypointIndex + direction,
      );
    }
    rebuildPoints(point.ref, index + direction);
  } catch (error) {
    setMessage(error.message, "error");
  }
}

function swapItems(items, first, second) {
  [items[first], items[second]] = [items[second], items[first]];
}

function deletePoint(index) {
  try {
    validatePoints();
    const point = state.points[index];
    if (!point) return;
    if (point.kind === "target") {
      const target = state.document.targets[point.targetIndex];
      const waypointCount = Array.isArray(target.waypoints) ? target.waypoints.length : 0;
      if (waypointCount && !window.confirm(`删除该目标点时会同时删除其 ${waypointCount} 个途径点。`)) {
        return;
      }
      state.document.targets.splice(point.targetIndex, 1);
    } else if (point.kind === "waypoint") {
      state.document.targets[point.targetIndex].waypoints.splice(point.waypointIndex, 1);
    } else if (point.kind === "goalWaypoint") {
      state.document.goal_waypoints.splice(point.goalWaypointIndex, 1);
    } else if (point.kind === "start") {
      delete state.document.start;
    } else if (point.kind === "goal") {
      delete state.document.goal;
    }
    rebuildPoints(null, index);
  } catch (error) {
    setMessage(error.message, "error");
  }
}

function isFiniteNumber(value) {
  if (String(value).trim() === "") return false;
  return Number.isFinite(Number(value));
}

function selectPoint(index, scroll = true) {
  if (index < 0 || index >= state.points.length) return;
  state.selectedIndex = index;
  elements.rows.querySelectorAll("tr").forEach((row, rowIndex) => {
    row.classList.toggle("selected", rowIndex === index);
  });
  if (scroll) {
    elements.rows.querySelector(`tr[data-index="${index}"]`)?.scrollIntoView({ block: "nearest" });
  }
  drawPlot();
}

function validatePoints() {
  let firstInvalid = null;
  state.points.forEach((point, index) => {
    for (const field of ["x", "y", "yaw"]) {
      const input = elements.rows.querySelector(`tr[data-index="${index}"] input[data-field="${field}"]`);
      const valid = isFiniteNumber(point.values[field]);
      input?.classList.toggle("invalid", !valid);
      if (!valid && !firstInvalid) firstInvalid = { index, input };
    }
  });
  if (firstInvalid) {
    selectPoint(firstInvalid.index);
    firstInvalid.input?.focus();
    throw new Error("所有 X、Y、Yaw 都必须是有限数字");
  }
}

async function saveCopy() {
  if (!state.filename || !state.document) return;
  try {
    validatePoints();
    for (const point of state.points) {
      for (const field of ["x", "y", "yaw"]) {
        point.ref[field] = Number(point.values[field]);
      }
    }
    const yamlText = jsyaml.dump(state.document, {
      noRefs: true,
      lineWidth: -1,
      noCompatMode: true,
    });
    setBusy(true);
    setMessage("正在保存...");
    const response = await request(
      `/api/missions/${encodeURIComponent(state.filename)}/save-copy`,
      {
        method: "POST",
        headers: { "Content-Type": "text/yaml; charset=utf-8" },
        body: yamlText,
      },
    );
    const payload = await response.json();
    if (!payload.filename) throw new Error("服务器未返回新文件名");
    state.dirty = false;
    await refreshMissions(payload.filename);
    updateStatus(`已另存为 ${payload.filename}`);
  } catch (error) {
    setMessage(error.message, "error");
  } finally {
    setBusy(false);
  }
}

function numericPoints() {
  return state.points.map((point, index) => ({
    index,
    kind: point.kind,
    x: Number(point.values.x),
    y: Number(point.values.y),
    valid: isFiniteNumber(point.values.x) && isFiniteNumber(point.values.y),
  })).filter((point) => point.valid);
}

function niceStep(span, targetTicks = 8) {
  const rough = span / targetTicks;
  const power = 10 ** Math.floor(Math.log10(rough));
  const fraction = rough / power;
  const niceFraction = fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10;
  return niceFraction * power;
}

function drawPlot() {
  const canvas = elements.canvas;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return;
  const dpr = window.devicePixelRatio || 1;
  const pixelWidth = Math.round(rect.width * dpr);
  const pixelHeight = Math.round(rect.height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);

  const points = numericPoints();
  elements.plotEmpty.hidden = points.length > 0 || state.adding;
  elements.plotEmpty.textContent = state.document
    ? "使用地图加点按钮添加路径点"
    : "请选择 Mission YAML 文件";
  state.screenPoints = [];

  const margin = { left: 58, right: 30, top: 28, bottom: 46 };
  const plotWidth = Math.max(1, rect.width - margin.left - margin.right);
  const plotHeight = Math.max(1, rect.height - margin.top - margin.bottom);
  const xs = points.map((point) => point.x).concat(0);
  const ys = points.map((point) => point.y).concat(0);
  let minX = Math.min(...xs);
  let maxX = Math.max(...xs);
  let minY = Math.min(...ys);
  let maxY = Math.max(...ys);
  const minSpan = 2;
  if (maxX - minX < minSpan) {
    const center = (minX + maxX) / 2;
    minX = center - minSpan / 2;
    maxX = center + minSpan / 2;
  }
  if (maxY - minY < minSpan) {
    const center = (minY + maxY) / 2;
    minY = center - minSpan / 2;
    maxY = center + minSpan / 2;
  }
  const padX = (maxX - minX) * 0.1;
  const padY = (maxY - minY) * 0.1;
  minX -= padX;
  maxX += padX;
  minY -= padY;
  maxY += padY;

  const unitsPerPixel = Math.max(
    (maxX - minX) / plotWidth,
    (maxY - minY) / plotHeight,
  );
  const xCenter = (minX + maxX) / 2;
  const yCenter = (minY + maxY) / 2;
  minX = xCenter - (unitsPerPixel * plotWidth) / 2;
  maxX = xCenter + (unitsPerPixel * plotWidth) / 2;
  minY = yCenter - (unitsPerPixel * plotHeight) / 2;
  maxY = yCenter + (unitsPerPixel * plotHeight) / 2;
  state.plotBounds = { minX, maxX, minY, maxY, margin, plotWidth, plotHeight };

  const toScreen = (x, y) => ({
    x: margin.left + ((x - minX) / (maxX - minX)) * plotWidth,
    y: margin.top + ((maxY - y) / (maxY - minY)) * plotHeight,
  });
  const xStep = niceStep(maxX - minX);
  const yStep = niceStep(maxY - minY);

  context.font = "11px Inter, sans-serif";
  context.lineWidth = 1;
  context.strokeStyle = "#e2e6e8";
  context.fillStyle = "#6d777e";
  context.textAlign = "center";
  context.textBaseline = "top";
  for (let x = Math.ceil(minX / xStep) * xStep; x <= maxX + xStep * 0.01; x += xStep) {
    const screen = toScreen(x, 0);
    context.beginPath();
    context.moveTo(screen.x, margin.top);
    context.lineTo(screen.x, margin.top + plotHeight);
    context.stroke();
    context.fillText(formatTick(x), screen.x, margin.top + plotHeight + 8);
  }
  context.textAlign = "right";
  context.textBaseline = "middle";
  for (let y = Math.ceil(minY / yStep) * yStep; y <= maxY + yStep * 0.01; y += yStep) {
    const screen = toScreen(0, y);
    context.beginPath();
    context.moveTo(margin.left, screen.y);
    context.lineTo(margin.left + plotWidth, screen.y);
    context.stroke();
    context.fillText(formatTick(y), margin.left - 9, screen.y);
  }

  const origin = toScreen(0, 0);
  context.strokeStyle = "#879199";
  context.lineWidth = 1.4;
  context.beginPath();
  context.moveTo(margin.left, origin.y);
  context.lineTo(margin.left + plotWidth, origin.y);
  context.moveTo(origin.x, margin.top + plotHeight);
  context.lineTo(origin.x, margin.top);
  context.stroke();
  context.fillStyle = "#4f5960";
  context.textAlign = "right";
  context.textBaseline = "bottom";
  context.fillText("X", margin.left + plotWidth, origin.y - 5);
  context.textAlign = "left";
  context.fillText("Y", origin.x + 5, margin.top + 2);

  state.screenPoints = points.map((point) => ({ ...point, ...toScreen(point.x, point.y) }));
  context.strokeStyle = "#278270";
  context.lineWidth = 2;
  context.lineJoin = "round";
  context.beginPath();
  state.screenPoints.forEach((point, index) => {
    if (index === 0) context.moveTo(point.x, point.y);
    else context.lineTo(point.x, point.y);
  });
  context.stroke();

  state.screenPoints.forEach((point) => {
    const selected = point.index === state.selectedIndex;
    const colorKind = point.kind === "waypoint" || point.kind === "goalWaypoint"
      ? "waypoint"
      : point.kind === "target" || point.kind === "goal" ? "target" : "start";
    const colors = {
      waypoint: { fill: "#e6f2f8", stroke: "#2476a3" },
      target: { fill: "#fff0e5", stroke: "#d06a28" },
      start: { fill: "#edf0f2", stroke: "#68747c" },
    }[colorKind];
    context.beginPath();
    context.arc(point.x, point.y, selected ? 7 : 5, 0, Math.PI * 2);
    context.fillStyle = colors.fill;
    context.fill();
    context.strokeStyle = selected ? "#20262d" : colors.stroke;
    context.lineWidth = selected ? 3 : 2;
    context.stroke();
    context.fillStyle = selected ? "#20262d" : colors.stroke;
    context.font = "bold 12px Inter, sans-serif";
    context.textAlign = "left";
    context.textBaseline = "bottom";
    context.fillText(String(point.index), point.x + 8, point.y - 6);
  });
}

function formatTick(value) {
  if (Math.abs(value) < 1e-10) return "0";
  return Number(value.toPrecision(6)).toString();
}

function updateAddControls() {
  elements.waypointMode.setAttribute("aria-pressed", String(state.addKind === "waypoint"));
  elements.targetMode.setAttribute("aria-pressed", String(state.addKind === "target"));
  elements.mapAdd.classList.toggle("active", state.adding);
  elements.mapAdd.textContent = state.adding ? "完成加点" : "＋ 地图加点";
  elements.canvas.classList.toggle("adding", state.adding);
}

function setAddKind(kind) {
  state.addKind = kind;
  updateAddControls();
  if (state.adding) {
    setMessage(kind === "waypoint" ? "地图加点：途径点" : "地图加点：目标点");
  }
}

function toggleMapAdding() {
  if (!state.document) return;
  state.adding = !state.adding;
  updateAddControls();
  if (state.adding) {
    setMessage(state.addKind === "waypoint" ? "地图加点：途径点" : "地图加点：目标点");
  } else {
    updateStatus();
  }
  drawPlot();
}

function addPointAt(screenX, screenY) {
  const bounds = state.plotBounds;
  if (!bounds) return;
  const { margin, plotWidth, plotHeight, minX, maxX, minY, maxY } = bounds;
  if (
    screenX < margin.left
    || screenX > margin.left + plotWidth
    || screenY < margin.top
    || screenY > margin.top + plotHeight
  ) {
    setMessage("请在坐标网格内添加点", "error");
    return;
  }

  try {
    validatePoints();
    const pose = {
      x: roundCoordinate(minX + ((screenX - margin.left) / plotWidth) * (maxX - minX)),
      y: roundCoordinate(maxY - ((screenY - margin.top) / plotHeight) * (maxY - minY)),
      yaw: 0,
    };
    if (state.addKind === "target") {
      addTarget(pose);
    } else {
      addWaypoint(pose);
    }
    rebuildPoints(pose, state.points.length);
    setMessage(state.addKind === "waypoint" ? "已增加途径点" : "已增加目标点", "dirty");
  } catch (error) {
    setMessage(error.message, "error");
  }
}

function roundCoordinate(value) {
  return Number(value.toFixed(6));
}

function addTarget(pose) {
  if (!Array.isArray(state.document.targets)) state.document.targets = [];
  const selected = state.points[state.selectedIndex];
  let insertIndex = state.document.targets.length;
  if (selected?.kind === "start") {
    insertIndex = 0;
  } else if (Number.isInteger(selected?.targetIndex)) {
    insertIndex = selected.targetIndex + 1;
  }
  state.document.targets.splice(insertIndex, 0, { pose });
}

function addWaypoint(pose) {
  const selected = state.points[state.selectedIndex];
  if (selected?.kind === "goal" || selected?.kind === "goalWaypoint") {
    if (state.document.goal_waypoints == null) state.document.goal_waypoints = [];
    if (!Array.isArray(state.document.goal_waypoints)) {
      throw new Error("goal_waypoints 不是数组");
    }
    const insertIndex = selected.kind === "goalWaypoint"
      ? selected.goalWaypointIndex + 1
      : state.document.goal_waypoints.length;
    state.document.goal_waypoints.splice(insertIndex, 0, pose);
    return;
  }

  const targets = Array.isArray(state.document.targets) ? state.document.targets : [];
  if (!targets.length) {
    throw new Error("请先增加一个目标点，再为它增加途径点");
  }
  const targetIndex = Number.isInteger(selected?.targetIndex)
    ? selected.targetIndex
    : selected?.kind === "start" ? 0 : targets.length - 1;
  const target = targets[targetIndex];
  if (target.waypoints == null) target.waypoints = [];
  if (!Array.isArray(target.waypoints)) {
    throw new Error(`target ${targetIndex} 的 waypoints 不是数组`);
  }
  const insertIndex = selected?.kind === "waypoint" && selected.targetIndex === targetIndex
    ? selected.waypointIndex + 1
    : target.waypoints.length;
  target.waypoints.splice(insertIndex, 0, pose);
}

elements.select.addEventListener("change", () => loadMission(elements.select.value));
elements.refresh.addEventListener("click", () => refreshMissions());
elements.reload.addEventListener("click", () => loadMission(state.filename));
elements.save.addEventListener("click", saveCopy);
elements.waypointMode.addEventListener("click", () => setAddKind("waypoint"));
elements.targetMode.addEventListener("click", () => setAddKind("target"));
elements.mapAdd.addEventListener("click", toggleMapAdding);
elements.canvas.addEventListener("click", (event) => {
  const rect = elements.canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  if (state.adding) {
    addPointAt(x, y);
    return;
  }
  let closest = null;
  for (const point of state.screenPoints) {
    const distance = Math.hypot(point.x - x, point.y - y);
    if (distance <= 14 && (!closest || distance < closest.distance)) {
      closest = { index: point.index, distance };
    }
  }
  if (closest) selectPoint(closest.index);
});

new ResizeObserver(drawPlot).observe(elements.plotPanel);
window.addEventListener("keydown", (event) => {
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "s") {
    event.preventDefault();
    saveCopy();
  }
});

setBusy(false);
updateAddControls();
refreshMissions();
