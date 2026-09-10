/* Shared viewer for Playground and tutorial embeds. Physics lives in Python. */
(() => {
    "use strict";
    const assets = new URL("playground/", document.currentScript.src);
    const zh = document.documentElement.lang.toLowerCase().startsWith("zh");
    const t = zh ? {
        title: "IR-SIM 在线实验室", subtitle: "真实 Python · 浏览器计算 · 无需服务器",
        launch: "启动 Python", run: "运行", pause: "暂停", step: "单步", reset: "应用 YAML / 重置",
        stop: "释放运行时", download: "下载 YAML", scene: "示例场景", free: "自由运动", obstacle: "碰撞与障碍物", lidar: "LiDAR 与复合形状",
        control: "控制方式", manual: "滑块指令（单个差速机器人）", automatic: "YAML 中的行为",
        speed: "前向速度 v (m/s)", yaw: "角速度 ω (rad/s)", duration: "运行时长", continuous: "持续运行", seed: "随机种子",
        yaml: "场景 YAML", hint: "修改 state 中的初始航向或 world.step_time，然后应用 YAML。指令滑块在下一次 step 时生效。",
        time: "回放已计算画面 (s)", frame: "Matplotlib 画面时刻", history: "最多保留最近 200 帧 / 32 MiB；回放不会回退仿真。",
        objects: "对象状态", object: "查看对象", inspectTime: "状态时刻", model: "类型 / 运动学", shape: "形状",
        state: "状态 state", velocity: "速度 velocity", goal: "当前目标 goal", flags: "状态标志",
        inspectHint: "显示模型原始数组；字段顺序和单位取决于运动学模型。切换对象只影响查看，不改变控制对象。",
        idle: "尚未启动：点击启动后才加载 Python 与依赖。",
        python: "正在加载 Python / WebAssembly…", packages: "正在加载科学计算依赖与 IR-SIM…",
        ready: "就绪", running: "运行中", paused: "已暂停", complete: "本次运行结束", busy: "正在计算…",
        limit: "已达到 2,000 步上限，请重置后继续。",
        error: "运行失败。可修正并应用 YAML，或释放运行时后重新启动。", unavailable: "请通过 HTTP(S) 打开文档。file:// 不支持模块 Worker。",
        source: "源码", collision: "碰撞", arrived: "到达", stopped: "停止标志", static: "静态", modified: "YAML 已修改，请应用后再运行。",
        plot: "IR-SIM 原生 Matplotlib 仿真画面", placeholder: "启动 Python 后显示原生 Matplotlib 画面",
        open: "打开在线实验", loading: "正在加载场景…", timeout: "请求超时。请释放环境后重试。",
    } : {
        title: "IR-SIM Online Playground", subtitle: "Real Python · Runs in your browser · No server",
        launch: "Launch Python", run: "Run", pause: "Pause", step: "Step", reset: "Apply YAML / Reset",
        stop: "Release runtime", download: "Download YAML", scene: "Example scene", free: "Free motion", obstacle: "Collision & obstacles", lidar: "LiDAR & compound shape",
        control: "Control source", manual: "Sliders (one diff robot)", automatic: "Behaviors in YAML",
        speed: "Forward speed v (m/s)", yaw: "Yaw rate ω (rad/s)", duration: "Run duration", continuous: "Continuous", seed: "Random seed",
        yaml: "Scene YAML", hint: "Edit the initial heading in state or world.step_time, then apply YAML. Command sliders take effect on the next step.",
        time: "Replay computed frames (s)", frame: "Matplotlib frame time", history: "Up to 200 recent frames / 32 MiB; replay does not rewind the simulation.",
        objects: "Object inspector", object: "Inspect object", inspectTime: "State time", model: "Role / kinematics", shape: "Shape",
        state: "State", velocity: "Velocity", goal: "Current goal", flags: "Flags",
        inspectHint: "Native arrays; component order and units depend on the kinematics model. Inspecting an object does not change which robot is controlled.",
        idle: "Not started: Python and dependencies load only when you launch.",
        python: "Loading Python / WebAssembly…", packages: "Loading scientific packages and IR-SIM…",
        ready: "Ready", running: "Running", paused: "Paused", complete: "Run finished", busy: "Computing…",
        limit: "2,000-step limit reached. Reset to continue.",
        error: "Run failed. Correct and apply the YAML, or release and relaunch the runtime.", unavailable: "Open the docs over HTTP(S). file:// does not support module workers.",
        source: "Source", collision: "Collision", arrived: "Arrived", stopped: "Stop flag", static: "Static", modified: "YAML changed. Apply it before running.",
        plot: "Native IR-SIM Matplotlib simulation frame", placeholder: "Launch Python to display the native Matplotlib scene",
        open: "Open in Playground", loading: "Loading scene…", timeout: "Request timed out. Release the runtime and try again.",
    };
    document.querySelectorAll("[data-kinematics-lab]").forEach((root, index) => {
        const id = `irsim-lab-${index}`, standalone = root.dataset.mode === "playground";
        const field = name => root.querySelector(`[name="${name}"]`);
        const button = name => root.querySelector(`[data-action="${name}"]`);
        const slider = (name, label, min, max, step, value) => `<div class="kl-field">
            <label for="${id}-${name}">${label}<output data-value="${name}" for="${id}-${name}"></output></label>
            <input id="${id}-${name}" name="${name}" type="range" min="${min}" max="${max}" step="${step}" value="${value}"></div>`;
        root.innerHTML = `
            <div class="kl-heading"><div><strong>${t.title}</strong><p>${t.subtitle}</p></div>
                <button type="button" data-action="launch" disabled>${t.launch}</button></div>
            <div class="kl-metadata"><div class="kl-status" role="status" aria-live="polite">${t.loading}</div><div class="kl-version"></div></div>
            <div class="kl-settings"><div class="kl-field">
                <label for="${id}-scene">${t.scene}</label>
                <select id="${id}-scene" name="scene"><option value="kinematics">${t.free}</option><option value="obstacle">${t.obstacle}</option><option value="lidar">${t.lidar}</option></select>
                </div><div class="kl-field">
                <label for="${id}-control">${t.control}</label>
                <select id="${id}-control" name="control"><option value="manual">${t.manual}</option><option value="automatic">${t.automatic}</option></select>
                </div><div class="kl-field">
                <label for="${id}-duration">${t.duration}</label>
                <select id="${id}-duration" name="duration"><option value="Infinity">${t.continuous}</option><option value="4">4 s</option><option value="10">10 s</option><option value="20">20 s</option></select>
                </div><div class="kl-field">
                <label for="${id}-seed">${t.seed}</label><input id="${id}-seed" name="seed" type="number" min="0" max="4294967295" step="1" value="0">
            </div></div>
            <div class="kl-workspace"><div class="kl-controls">
                ${slider("speed", t.speed, -1.2, 1.2, 0.1, 0.8)}
                ${slider("yaw", t.yaw, -1.5, 1.5, 0.1, 0.6)}
                <div class="kl-toolbar">
                    <button type="button" data-action="run" disabled>${t.run}</button><button type="button" data-action="step" disabled>${t.step}</button>
                    <button type="button" data-action="reset" disabled>${t.reset}</button><button type="button" data-action="stop" disabled>${t.stop}</button>
                </div>
            </div><div class="kl-stage"><div class="kl-figure">
                <div class="kl-placeholder">${t.placeholder}</div><img class="kl-frame" alt="${t.plot}" hidden>
                <div class="kl-frame-caption"></div>
            </div><div class="kl-timeline">${slider("time", t.time, 0, 0, 1, 0)}<p>${t.history}</p></div></div>
            <section class="kl-inspector" aria-labelledby="${id}-inspector">
                <div class="kl-inspector-heading"><strong id="${id}-inspector">${t.objects}</strong><span data-inspect-time>${t.inspectTime}: —</span></div>
                <label for="${id}-object">${t.object}</label><select id="${id}-object" name="object" disabled><option>—</option></select>
                <div class="kl-readout">
                    ${["model", "shape", "state", "velocity", "goal"].map(key => `<div><span>${t[key]}</span><output data-${key}>—</output></div>`).join("")}
                    <div><span>${t.flags}</span><div class="kl-flags">
                        ${["collision", "arrived", "stopped", "static"].map(key => `<output data-flag="${key}">${t[key]}: —</output>`).join("")}
                    </div></div>
                </div><p>${t.inspectHint}</p>
            </section></div>
            <details class="kl-editor"><summary>${t.yaml}</summary><p>${t.hint}</p>
                <label class="visually-hidden" for="${id}-yaml">${t.yaml}</label><textarea id="${id}-yaml" name="yaml" spellcheck="false" rows="15" maxlength="64000"></textarea>
                <button type="button" data-action="download">${t.download}</button>
                ${standalone ? "" : `<a data-open href="../playground/index.html">${t.open}</a>`}
            </details>`;
        let worker = null, serial = 0, busy = false, ready = false, running = false;
        let history = [], historyBytes = 0, dirty = false, targetTime = 0, timer, sceneRequest = 0;
        let objectKey = "";
        const pending = new Map();
        const status = message => { root.querySelector(".kl-status").textContent = message; };
        function controls() {
            const exhausted = history.at(-1)?.steps >= 2000;
            button("launch").disabled = busy || Boolean(worker) || !field("yaml").value || location.protocol === "file:";
            button("stop").disabled = !worker;
            button("reset").disabled = !ready || busy || running;
            button("step").disabled = !ready || busy || running || dirty || exhausted;
            button("run").disabled = !ready || dirty || exhausted || (busy && !running);
            button("run").textContent = running ? t.pause : t.run;
            for (const name of ["yaml", "seed", "scene", "control", "duration"]) field(name).disabled = busy || running;
            for (const name of ["speed", "yaw"]) field(name).disabled = field("control").value !== "manual";
        }
        function request(message) {
            const requestId = ++serial;
            return new Promise((resolve, reject) => {
                const timeout = setTimeout(() => { pending.delete(requestId); reject(new Error(t.timeout)); }, message.method === "initialize" ? 180000 : 15000);
                pending.set(requestId, {resolve, reject, timeout});
                worker.postMessage({id: requestId, ...message});
            });
        }
        function release() {
            running = false; clearTimeout(timer);
            if (worker) worker.terminate();
            worker = null; ready = false; busy = false;
            for (const item of pending.values()) { clearTimeout(item.timeout); item.reject(Object.assign(new Error("Runtime released."), {name: "AbortError"})); }
            pending.clear(); controls();
        }
        function fail(error) {
            if (error.name === "AbortError") return;
            if (error.remote && ready) {
                running = false; clearTimeout(timer); dirty = true; busy = false; controls();
            } else { release(); }
            status(`${t.error}\n${String(error).slice(-1600)}`);
        }
        async function create() {
            busy = true; controls(); status(t.busy);
            try {
                const snapshot = await request({method: "create", yaml: field("yaml").value, seed: Number(field("seed").value)});
                field("control").querySelector('[value="manual"]').disabled = !snapshot.manual_control;
                if (!snapshot.manual_control) field("control").value = "automatic";
                remember(snapshot, true); dirty = false; status(t.ready);
            } finally { busy = false; controls(); }
        }
        async function advance(count = 1) {
            busy = true; controls();
            try {
                const action = field("control").value === "manual" ? [Number(field("speed").value), Number(field("yaw").value)] : null;
                const result = await request({method: "step", action, count});
                remember(result);
                if (result.done || result.time >= targetTime - 1e-9 || result.steps >= 2000) {
                    running = false; status(result.steps >= 2000 ? t.limit : t.complete);
                }
                return result;
            } finally { busy = false; controls(); }
        }
        async function loop() {
            if (!running) return;
            try {
                const start = performance.now(), last = history.at(-1);
                // Aim for at most 10 image updates/s without changing step_time.
                const count = Math.max(1, Math.min(20, Math.ceil(0.1 / last.dt), Math.ceil((targetTime - last.time) / last.dt)));
                const result = await advance(count);
                if (running) timer = setTimeout(loop, Math.max(0, count * result.dt * 1000 - (performance.now() - start)));
            } catch (error) { fail(error); }
        }
        function clearHistory() {
            for (const snapshot of history) URL.revokeObjectURL(snapshot.image);
            history = []; historyBytes = 0;
        }
        function remember(snapshot, reset = false) {
            const blob = new Blob([snapshot.frame], {type: "image/png"});
            delete snapshot.frame;
            if (reset) clearHistory();
            snapshot.image = URL.createObjectURL(blob); snapshot.bytes = blob.size;
            history.push(snapshot); historyBytes += blob.size;
            while (history.length > 1 && (history.length > 200 || historyBytes > 32 * 1024 * 1024)) {
                const oldest = history.shift(); historyBytes -= oldest.bytes; URL.revokeObjectURL(oldest.image);
            }
            render(history.length - 1);
        }
        function render(k) {
            const snapshot = history[k];
            if (!snapshot) return;
            const img = root.querySelector(".kl-frame");
            img.src = snapshot.image; img.hidden = false;
            root.querySelector(".kl-placeholder").hidden = true;
            root.querySelector(".kl-frame-caption").textContent = `${t.frame}: ${snapshot.frame_time.toFixed(2)} s`;
            field("time").max = history.length - 1; field("time").value = k;
            field("time").setAttribute("aria-valuetext", `${snapshot.time.toFixed(2)} s`);
            root.querySelector('[data-value="time"]').value = snapshot.time.toFixed(2);
            inspect(snapshot);
        }
        function inspect(snapshot) {
            const select = field("object");
            const key = JSON.stringify(snapshot.objects.map(obj => [obj.id, obj.name]));
            if (key !== objectKey) {
                const previous = select.value;
                select.replaceChildren(...snapshot.objects.map(obj => {
                    const option = document.createElement("option");
                    option.value = obj.id; option.textContent = `${obj.name} · #${obj.id}`;
                    return option;
                }));
                if (snapshot.objects.some(obj => String(obj.id) === previous)) select.value = previous;
                objectKey = key;
            }
            select.disabled = !snapshot.objects.length;
            const obj = snapshot.objects.find(item => String(item.id) === select.value);
            if (!obj) return;
            root.querySelector("[data-inspect-time]").textContent = `${t.inspectTime}: ${snapshot.time.toFixed(2)} s`;
            root.querySelector("[data-model]").value = `${obj.role} / ${obj.kinematics ?? "—"}`;
            root.querySelector("[data-shape]").value = obj.shape;
            for (const key of ["state", "velocity", "goal"]) {
                root.querySelector(`[data-${key}]`).value = obj[key] === null ? "—" : `[${obj[key].map(v => v.toFixed(4)).join(", ")}]`;
            }
            for (const output of root.querySelectorAll("[data-flag]")) {
                const key = output.dataset.flag;
                output.value = `${t[key]}: ${obj[key]}`; output.dataset.value = obj[key];
            }
        }
        function updateLabels() {
            for (const name of ["speed", "yaw"]) root.querySelector(`[data-value="${name}"]`).value = Number(field(name).value).toFixed(1);
        }
        async function loadScene(name) {
            const token = ++sceneRequest; busy = true; controls(); status(t.loading);
            try {
                const response = await fetch(new URL(`${name}.yaml`, assets));
                if (!response.ok) throw new Error(`Scene: HTTP ${response.status}`);
                const source = await response.text();
                if (token !== sceneRequest) return;
                field("yaml").value = source; field("yaw").value = name === "kinematics" ? 0.6 : 0;
                field("control").querySelector('[value="manual"]').disabled = false;
                field("control").value = "manual";
                updateLabels(); dirty = true; status(ready ? t.modified : t.idle);
            } finally { busy = false; controls(); }
        }
        button("launch").addEventListener("click", async () => {
            try {
                worker = new Worker(new URL("worker.mjs", assets), {type: "module"});
                worker.onmessage = ({data}) => {
                    if (data.status) { status(t[data.status] || data.status); return; }
                    const item = pending.get(data.id);
                    if (!item) return;
                    pending.delete(data.id); clearTimeout(item.timeout);
                    if (data.error) item.reject(Object.assign(new Error(data.error), {remote: true})); else item.resolve(data.result);
                };
                worker.onerror = event => fail(event.message || "Worker failed.");
                busy = true; controls();
                const metadata = await request({method: "initialize"});
                root.querySelector(".kl-version").textContent = `IR-SIM ${metadata.version} · ${t.source} ${metadata.source.slice(0, 12)} · Pyodide ${metadata.pyodide}`;
                ready = true; await create();
            } catch (error) { fail(error); }
        });
        button("run").addEventListener("click", () => {
            running = !running; clearTimeout(timer); controls(); status(running ? t.running : t.paused);
            if (running) { targetTime = history.at(-1).time + Number(field("duration").value); loop(); }
        });
        button("step").addEventListener("click", () => { targetTime = Infinity; advance().then(result => {
            if (!result.done && result.steps < 2000) status(t.ready);
        }).catch(fail); });
        button("reset").addEventListener("click", () => create().catch(fail));
        button("stop").addEventListener("click", () => { release(); status(t.idle); });
        field("scene").addEventListener("change", () => loadScene(field("scene").value).catch(fail));
        field("control").addEventListener("change", controls);
        field("time").addEventListener("input", () => render(Number(field("time").value)));
        field("object").addEventListener("change", () => {
            const snapshot = history[Number(field("time").value)];
            if (snapshot) inspect(snapshot);
        });
        for (const name of ["speed", "yaw"]) field(name).addEventListener("input", updateLabels);
        for (const name of ["yaml", "seed"]) field(name).addEventListener("input", () => { dirty = true; status(t.modified); controls(); });
        button("download").addEventListener("click", () => {
            const url = URL.createObjectURL(new Blob([field("yaml").value], {type: "text/yaml"}));
            const link = document.createElement("a"); link.href = url; link.download = "playground.yaml"; link.click();
            setTimeout(() => URL.revokeObjectURL(url), 1000);
        });
        const openLink = root.querySelector("[data-open]");
        if (openLink) openLink.addEventListener("click", () => {
            const payload = {yaml: field("yaml").value, speed: field("speed").value, yaw: field("yaw").value, seed: field("seed").value, control: field("control").value};
            openLink.hash = encodeURIComponent(JSON.stringify(payload));
        });
        window.addEventListener("pagehide", () => { release(); clearHistory(); }); updateLabels();
        if (location.protocol === "file:") { status(t.unavailable); controls(); return; }
        loadScene("kinematics").then(() => {
            if (standalone && location.hash.length > 1) {
                const payload = JSON.parse(decodeURIComponent(location.hash.slice(1)));
                if (typeof payload.yaml !== "string" || payload.yaml.length > 64000) throw new Error("Invalid shared scene.");
                field("yaml").value = payload.yaml;
                for (const key of ["speed", "yaw", "seed"]) if (payload[key] !== undefined) field(key).value = payload[key];
                if (["manual", "automatic"].includes(payload.control)) field("control").value = payload.control;
                updateLabels(); controls();
            }
        }).catch(fail);
    });
})();
