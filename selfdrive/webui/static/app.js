let schema = null;
let values = {};
let activePanel = null;
let toastTimer = null;
let debugTimer = null;
const runningActions = new Set();
const DEBUG_PANEL = "Debug";

const $ = (id) => document.getElementById(id);

function msg(key, fallback) {
  return schema?.messages?.[key] || fallback;
}

function showToast(message) {
  const toast = $("toast");
  toast.textContent = message;
  toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("show"), 2400);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: {"Content-Type": "application/json"},
    ...options,
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.error || body.message || response.statusText);
  }
  return body;
}

function applyChrome() {
  document.documentElement.lang = schema.language === "main_ko" ? "ko" : "en";
  document.title = msg("app_title", "K230 Settings");
  $("app-title").textContent = msg("app_title", "K230 Settings");
  $("subtitle").textContent = msg("subtitle", "openpilot Params editor");
  $("search").placeholder = msg("search_placeholder", "Search settings");
}

function settingValue(setting) {
  return values[setting.key] ?? setting.default ?? "";
}

function displayValue(value) {
  if (value === null || value === undefined || value === "") return "-";
  if (typeof value === "number") return Number.isInteger(value) ? String(value) : value.toFixed(4).replace(/0+$/, "").replace(/\.$/, "");
  return String(value);
}

function settingMatches(setting, query) {
  if (!query) return true;
  const panelTitle = schema.panel_titles?.[setting.panel] || setting.panel;
  const text = [
    panelTitle, setting.title, setting.key, setting.description,
    setting.details, setting.value_guide, setting.default,
  ].join(" ").toLowerCase();
  return text.includes(query.toLowerCase());
}

function stopDebugTimer() {
  if (debugTimer) {
    clearInterval(debugTimer);
    debugTimer = null;
  }
}

function renderTabs() {
  const tabs = $("tabs");
  tabs.innerHTML = "";
  for (const panel of schema.panels) {
    const button = document.createElement("button");
    button.textContent = schema.panel_titles?.[panel] || panel;
    button.className = panel === activePanel ? "active" : "";
    button.onclick = () => {
      activePanel = panel;
      render();
    };
    tabs.appendChild(button);
  }
}

function textInput(setting, value, multiline = false) {
  const input = document.createElement(multiline ? "textarea" : "input");
  input.value = value;
  input.onchange = async () => save(setting, input.value);
  return input;
}

function controlFor(setting) {
  const value = settingValue(setting);
  if (setting.type === "bool") {
    const label = document.createElement("label");
    label.className = "switch";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = value === "1" || value === true;
    input.onchange = async () => {
      await save(setting, input.checked ? "1" : "0");
    };
    const span = document.createElement("span");
    const updateLabel = () => span.textContent = input.checked ? msg("on", "On") : msg("off", "Off");
    input.addEventListener("change", updateLabel);
    updateLabel();
    label.append(input, span);
    return label;
  }
  if (setting.type === "enum" && setting.options.length > 0) {
    const select = document.createElement("select");
    for (const option of setting.options) {
      const item = document.createElement("option");
      item.value = option.value;
      item.textContent = option.label;
      item.selected = option.value === String(value);
      select.appendChild(item);
    }
    select.onchange = async () => save(setting, select.value);
    return select;
  }
  if (setting.type === "number") {
    const input = document.createElement("input");
    input.type = "number";
    input.value = value;
    if (setting.step !== null) input.step = setting.step;
    if (setting.min !== null) input.min = setting.min;
    if (setting.max !== null) input.max = setting.max;
    input.onchange = async () => save(setting, input.value);
    return input;
  }
  if (setting.type === "csv" || String(value).length > 80) {
    return textInput(setting, value, true);
  }
  if (setting.type === "action") {
    return actionControl(setting);
  }
  if (setting.readonly || setting.type === "readonly") {
    const div = document.createElement("div");
    div.className = "readonly";
    div.textContent = displayValue(value);
    return div;
  }
  return textInput(setting, value, false);
}

function actionControl(setting) {
  const group = document.createElement("div");
  group.className = "action-group";
  let input = null;
  if (setting.input_key) {
    input = document.createElement("input");
    input.placeholder = setting.input_placeholder || "";
    input.value = values.GithubUsername || "";
    group.appendChild(input);
  }
  const button = document.createElement("button");
  button.className = setting.danger ? "action-btn danger" : "action-btn";
  button.textContent = runningActions.has(setting.action) ? msg("running", "Running") : setting.title;
  button.disabled = runningActions.has(setting.action);
  button.onclick = async () => {
    const payload = {};
    if (input) {
      if (!input.value.trim()) {
        showToast(msg("input_required", "Input is required"));
        return;
      }
      payload[setting.input_key] = input.value.trim();
    }
    if (setting.confirm_text) {
      const promptText = msg("type_to_confirm", "Type %1 to confirm").replace("%1", setting.confirm_text);
      if (prompt(promptText) !== setting.confirm_text) return;
    } else if (!confirm(`${setting.title}: ${msg("confirm", "Are you sure?")}`)) {
      return;
    }
    await runAction(setting, payload);
  };
  group.appendChild(button);
  return group;
}

async function runAction(setting, payload) {
  runningActions.add(setting.action);
  renderCurrentPanel();
  try {
    const result = await api(`/api/actions/${setting.action}`, {
      method: "POST",
      body: JSON.stringify(payload),
    });
    showToast(result.message || `${setting.title}: ${msg("done", "Done")}`);
    const params = await api("/api/params");
    values = params.values;
  } catch (error) {
    showToast(`${setting.title}: ${error.message}`);
  } finally {
    runningActions.delete(setting.action);
    renderCurrentPanel();
  }
}

async function save(setting, value) {
  try {
    const result = await api(`/api/params/${setting.key}`, {
      method: "POST",
      body: JSON.stringify({value}),
    });
    values[setting.key] = result.value;
    showToast(`${setting.key}: ${msg("saved", "Saved")}`);
    if (setting.key === "LanguageSetting") {
      await loadData(activePanel);
    } else {
      renderCurrentPanel();
    }
  } catch (error) {
    showToast(`${msg("save_failed", "Save failed")}: ${error.message}`);
  }
}

function chip(text, className = "") {
  const span = document.createElement("span");
  span.className = `chip ${className}`.trim();
  span.textContent = text;
  return span;
}

function renderSettingHelp(meta, setting) {
  const hasDefault = setting.default !== null && setting.default !== undefined;
  const chips = document.createElement("div");
  chips.className = "chips";
  if (hasDefault) {
    const same = String(settingValue(setting)) === String(setting.default);
    chips.appendChild(chip(same ? msg("matches_default", "Default OK") : msg("changed", "Changed"), same ? "ok" : "changed"));
    chips.appendChild(chip(`${msg("default", "Default")}: ${displayValue(setting.default)}`));
  }
  if (setting.live) chips.appendChild(chip(msg("live", "Live"), "live"));
  if (chips.childNodes.length) meta.appendChild(chips);

  const help = document.createElement("div");
  help.className = "help";
  if (setting.details) {
    const line = document.createElement("div");
    line.textContent = `${msg("details", "Function")}: ${setting.details}`;
    help.appendChild(line);
  }
  if (setting.value_guide) {
    const line = document.createElement("div");
    line.textContent = `${msg("value_guide", "Value / unit")}: ${setting.value_guide}`;
    help.appendChild(line);
  }
  if (help.childNodes.length) meta.appendChild(help);
}

function renderSettings() {
  stopDebugTimer();
  const query = $("search").value;
  const section = $("settings");
  section.innerHTML = "";
  let rendered = 0;
  for (const setting of schema.settings) {
    if (setting.panel !== activePanel || !settingMatches(setting, query)) continue;
    const row = document.createElement("div");
    row.className = "row";

    const meta = document.createElement("div");
    const title = document.createElement("div");
    title.className = "title";
    title.textContent = setting.title;
    const key = document.createElement("div");
    key.className = "key";
    key.textContent = setting.key;
    meta.append(title, key);
    if (setting.description) {
      const desc = document.createElement("div");
      desc.className = "desc";
      desc.textContent = setting.description;
      meta.appendChild(desc);
    }
    renderSettingHelp(meta, setting);

    row.append(meta, controlFor(setting));
    section.appendChild(row);
    rendered += 1;
  }
  if (rendered === 0) {
    const empty = document.createElement("div");
    empty.className = "empty";
    empty.textContent = msg("no_settings", "No settings");
    section.appendChild(empty);
  }
}

function debugItem(label, value) {
  const div = document.createElement("div");
  div.className = "debug-item";
  const name = document.createElement("span");
  name.textContent = label;
  const val = document.createElement("strong");
  val.textContent = displayValue(value);
  div.append(name, val);
  return div;
}

function renderDebugData(data) {
  const section = $("settings");
  section.innerHTML = "";
  if (!data.ok) {
    const empty = document.createElement("div");
    empty.className = "empty";
    empty.textContent = data.error || msg("no_debug_data", "No debug data");
    section.appendChild(empty);
    return;
  }

  const status = document.createElement("div");
  status.className = "debug-grid";
  status.append(
    debugItem("controlsState", data.updated ? msg("updated", "Updated") : msg("stale", "Stale")),
    debugItem("alive", data.alive),
    debugItem("valid", data.valid),
    debugItem("logMonoTime", data.logMonoTime),
  );
  section.appendChild(status);

  const alerts = document.createElement("div");
  alerts.className = "debug-card";
  const alertsTitle = document.createElement("h2");
  alertsTitle.textContent = "alertTextMsg";
  alerts.appendChild(alertsTitle);
  for (const [index, value] of data.alerts.entries()) {
    alerts.appendChild(debugItem(`msg${index + 1}`, value || "-"));
  }
  section.appendChild(alerts);

  const controls = document.createElement("div");
  controls.className = "debug-card";
  const controlsTitle = document.createElement("h2");
  controlsTitle.textContent = "controls";
  controls.appendChild(controlsTitle);
  for (const [key, value] of Object.entries(data.controls || {})) {
    controls.appendChild(debugItem(key, value));
  }
  section.appendChild(controls);

  const lateral = document.createElement("div");
  lateral.className = "debug-card";
  const lateralTitle = document.createElement("h2");
  lateralTitle.textContent = `lateralState ${data.lateralState?.which || ""}`;
  lateral.appendChild(lateralTitle);
  const entries = Object.entries(data.lateralState?.values || {});
  if (entries.length === 0) {
    lateral.appendChild(debugItem("state", "-"));
  } else {
    for (const [key, value] of entries) lateral.appendChild(debugItem(key, value));
  }
  section.appendChild(lateral);
}

async function refreshDebug() {
  if (activePanel !== DEBUG_PANEL) return;
  try {
    renderDebugData(await api("/api/debug"));
  } catch (error) {
    renderDebugData({ok: false, error: error.message});
  }
}

function renderDebug() {
  $("settings").innerHTML = "";
  refreshDebug();
  if (!debugTimer) debugTimer = setInterval(refreshDebug, 1000);
}

function renderCurrentPanel() {
  if (activePanel === DEBUG_PANEL) {
    renderDebug();
  } else {
    renderSettings();
  }
}

function render() {
  applyChrome();
  renderTabs();
  renderCurrentPanel();
}

async function loadData(preferredPanel = null) {
  schema = await api("/api/schema");
  const params = await api("/api/params");
  values = params.values;
  activePanel = preferredPanel && schema.panels.includes(preferredPanel) ? preferredPanel : (activePanel || schema.panels[0]);
  render();
}

async function init() {
  $("search").addEventListener("input", renderCurrentPanel);
  await loadData();
}

init().catch((error) => {
  showToast(`${msg("load_failed", "Load failed")}: ${error.message}`);
});
