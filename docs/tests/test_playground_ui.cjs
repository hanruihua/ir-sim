// DOM checks only; pyodide_smoke.cjs separately exercises the real Python engine.
// NODE_PATH=<jsdom install>/node_modules node --test docs/tests/test_playground_ui.cjs
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const {JSDOM} = require("jsdom");
const script = fs.readFileSync(path.join(__dirname, "../source/_static/kinematics-lab.js"), "utf8");
const tick = () => new Promise(resolve => setTimeout(resolve, 0));

function setup(language = "en", protocol = "https:", standalone = false) {
    const dom = new JSDOM(`<html lang="${language}"><body><div data-kinematics-lab ${standalone ? 'data-mode="playground"' : ""}></div></body></html>`, {
        url: `${protocol}//example.test/en/get_started/kinematics.html`, runScripts: "outside-only",
    });
    const {window} = dom;
    Object.defineProperty(window.document, "currentScript", {value: {src: "https://example.test/en/_static/kinematics-lab.js"}});
    window.fetch = async url => ({ok: true, text: async () => fs.readFileSync(path.join(__dirname, "../source/_static/playground", new URL(url).pathname.split("/").at(-1)), "utf8")});
    const calls = [], workers = [], urls = new Set();
    let imageId = 0;
    window.URL.createObjectURL = () => { const url = `blob:frame-${++imageId}`; urls.add(url); return url; };
    window.URL.revokeObjectURL = url => urls.delete(url);
    const snapshot = steps => ({time: steps * 0.2, dt: 0.2, steps, frame_time: steps * 0.2, manual_control: true, done: false, frame: new Uint8Array([137,80,78,71]), objects: [{
        id: 0, name: "robot_0", role: "robot", kinematics: "diff", shape: "circle",
        state: steps ? [1, 2, 0.3] : [0, 0, 0], velocity: steps ? [0.8, 0.6] : [0, 0], goal: [4, 4, 0],
        collision: false, arrived: false, stopped: false, static: false,
    }]});
    window.Worker = class {
        constructor(url, options) { this.steps = 0; workers.push(this); assert.equal(options.type, "module"); }
        postMessage(message) {
            calls.push(message);
            queueMicrotask(() => {
                if (this.terminated || this.hold) return;
                if (this.fail) { this.onmessage({data: {id: message.id, error: "Invalid scene"}}); return; }
                let result;
                if (message.method === "initialize") result = {version: "2.11.0", source: "abc123abcdef456", pyodide: "314.0.6"};
                if (message.method === "create") { this.steps = 0; result = snapshot(0); }
                if (message.method === "step") { this.steps += message.count; result = snapshot(this.steps); }
                if (result && this.automatic) result.manual_control = false;
                if (result && this.done) result.done = true;
                if (result?.objects && this.objects) result.objects = this.objects(this.steps);
                this.onmessage({data: {id: message.id, result}});
            });
        }
        terminate() { this.terminated = true; }
    };
    window.eval(script);
    return {dom, calls, workers, urls, root: window.document.querySelector("[data-kinematics-lab]")};
}

// Advance only the paced simulation loop; request timeouts keep their real clocks.
function manualFrames(window) {
    const originalSet = window.setTimeout.bind(window), originalClear = window.clearTimeout.bind(window);
    const queued = new Map();
    let id = 0;
    window.setTimeout = (callback, delay) => {
        if (delay > 500) return originalSet(callback, delay);
        queued.set(--id, callback); return id;
    };
    window.clearTimeout = timer => { if (!queued.delete(timer)) originalClear(timer); };
    return {
        get size() { return queued.size; },
        async next() {
            assert.equal(queued.size, 1, "only one frame may be scheduled");
            const [timer, callback] = queued.entries().next().value;
            queued.delete(timer); callback(); await tick();
        },
    };
}

for (const standalone of [false, true]) {
    test(`compact layout keeps controls together and YAML accessible (standalone=${standalone})`, async () => {
        const {dom, root} = setup("en", "https:", standalone);
        try {
            await tick();
            for (const name of ["scene", "control", "duration", "seed"]) assert.ok(root.querySelector(`.kl-settings [name="${name}"]`));
            for (const name of ["speed", "yaw"]) assert.ok(root.querySelector(`.kl-workspace > .kl-controls [name="${name}"]`));
            assert.equal(root.querySelectorAll(".kl-controls .kl-toolbar button").length, 4);
            assert.ok(root.querySelector(".kl-stage .kl-frame"));
            assert.ok(root.querySelector('.kl-stage [name="time"]'));
            assert.ok(root.querySelector(".kl-workspace > .kl-inspector"));
            const editor = root.querySelector(".kl-editor");
            assert.equal(editor.open, false);
            editor.querySelector("summary").click();
            assert.equal(editor.open, true);
            assert.match(editor.querySelector("textarea").value, /world:/);
        } finally { dom.window.close(); }
    });
}

for (const language of ["en", "zh-CN"]) {
    test(`continuous default, live commands, pause and safety limit (${language})`, async () => {
        const {dom, root, calls, workers} = setup(language);
        const button = name => root.querySelector(`[data-action="${name}"]`);
        const field = name => root.querySelector(`[name="${name}"]`);
        try {
            await tick(); button("launch").click(); await tick();
            assert.equal(field("duration").value, "Infinity");
            const frames = manualFrames(dom.window);
            button("run").click(); await tick();
            assert.equal(field("duration").disabled, true);
            assert.equal(field("speed").disabled, false);
            assert.equal(field("yaw").disabled, false);
            for (let i = 0; i < 24; i++) await frames.next();
            assert.equal(workers[0].steps, 25); // Still running beyond the previous 4 s default.
            assert.equal(frames.size, 1);
            field("speed").value = "0.3"; field("yaw").value = "-0.4";
            for (const name of ["speed", "yaw"]) field(name).dispatchEvent(new dom.window.Event("input"));
            await frames.next();
            assert.deepEqual(Array.from(calls.at(-1).action), [0.3, -0.4]);
            button("run").click();
            assert.equal(frames.size, 0);
            assert.equal(field("duration").disabled, false);
            button("run").click(); await tick();
            assert.equal(workers[0].steps, 27); // Resume without recreating the environment.
            workers[0].steps = 1999;
            await frames.next();
            assert.equal(workers[0].steps, 2000);
            assert.equal(frames.size, 0);
            assert.equal(button("run").disabled, true);
            assert.equal(button("step").disabled, true);
            assert.match(root.querySelector(".kl-status").textContent, /2,000/);
            button("reset").click(); await tick();
            assert.equal(button("run").disabled, false);
            assert.equal(button("step").disabled, false);
            assert.equal(field("duration").value, "Infinity");
            workers[0].done = true;
            button("run").click(); await tick();
            assert.equal(frames.size, 0); // Native completion still ends a continuous run.
        } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
    });
}

test("fixed durations still stop at the selected simulated time", async () => {
    const {dom, root, workers} = setup();
    const button = name => root.querySelector(`[data-action="${name}"]`);
    try {
        await tick(); button("launch").click(); await tick();
        const frames = manualFrames(dom.window);
        for (const duration of [4, 10, 20]) {
            button("reset").click(); await tick();
            root.querySelector('[name="duration"]').value = String(duration);
            button("run").click(); await tick();
            for (let step = 1; step < duration / 0.2; step++) await frames.next();
            assert.equal(frames.size, 0);
            assert.equal(workers[0].steps, duration / 0.2);
            assert.equal(button("step").disabled, false);
        }
    } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
});

for (const language of ["en", "zh-CN"]) {
    test(`lazy initialization, Python snapshots, replay, reset (${language})`, async () => {
        const {dom, root, calls, workers, urls} = setup(language);
        const button = name => root.querySelector(`[data-action="${name}"]`);
        try {
            await tick(); assert.equal(workers.length, 0);
            assert.equal(button("step").disabled, true);
            for (const label of root.querySelectorAll("label[for]")) assert.ok(dom.window.document.getElementById(label.htmlFor));
            button("launch").click(); await tick();
            assert.deepEqual(calls.map(c => c.method), ["initialize", "create"]);
            assert.match(root.querySelector(".kl-version").textContent, /IR-SIM 2.11.0/);
            assert.equal(button("step").disabled, false);
            button("step").click(); await tick();
            assert.equal(root.querySelector("[data-state]").value, "[1.0000, 2.0000, 0.3000]");
            assert.match(root.querySelector(".kl-frame").src, /^blob:/);
            assert.equal(root.querySelector(".kl-frame").hidden, false);
            assert.equal(root.querySelectorAll("svg").length, 0);
            const time = root.querySelector('[name="time"]');
            time.value = 0; time.dispatchEvent(new dom.window.Event("input"));
            assert.equal(root.querySelector("[data-state]").value, "[0.0000, 0.0000, 0.0000]");
            assert.equal(calls.length, 3); // Replay does not call Python.
            button("reset").click(); await tick();
            assert.equal(time.max, "0");
            assert.equal(urls.size, 1); // Reset releases old PNG URLs.
            root.querySelector('[name="yaml"]').dispatchEvent(new dom.window.Event("input"));
            assert.equal(button("step").disabled, true);
            button("reset").click(); await tick();
            button("run").click(); await tick();
            button("run").click(); // pause, no queued next step
            assert.equal(button("step").disabled, false);
            button("stop").click();
            assert.equal(workers[0].terminated, true);
            assert.equal(button("launch").disabled, false);
        } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
    });
}

test("Python errors allow YAML correction; releasing an in-flight step is safe", async () => {
    const {dom, root, workers} = setup();
    const button = name => root.querySelector(`[data-action="${name}"]`);
    try {
        await tick(); button("launch").click(); await tick();
        workers[0].fail = true; button("reset").click(); await tick();
        assert.match(root.querySelector(".kl-status").textContent, /Invalid scene/);
        assert.equal(button("reset").disabled, false);
        assert.equal(button("step").disabled, true);
        workers[0].fail = false; button("reset").click(); await tick();
        workers[0].hold = true; button("step").click(); button("stop").click(); await tick();
        assert.equal(button("launch").disabled, false);
        assert.doesNotMatch(root.querySelector(".kl-status").textContent, /failed/);
    } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
});

test("all example scenes default to sliders; explicit YAML control survives reset", async () => {
    const {dom, root, calls} = setup();
    const button = name => root.querySelector(`[data-action="${name}"]`);
    const field = name => root.querySelector(`[name="${name}"]`);
    try {
        await tick(); button("launch").click(); await tick();
        for (const scene of ["kinematics", "obstacle", "lidar"]) {
            field("scene").value = scene;
            field("scene").dispatchEvent(new dom.window.Event("change")); await tick();
            button("reset").click(); await tick();
            assert.equal(field("control").value, "manual");
            assert.equal(field("speed").disabled, false);
            assert.equal(field("yaw").disabled, false);
            button("step").click(); await tick();
            assert.deepEqual(Array.from(calls.at(-1).action), [0.8, scene === "kinematics" ? 0.6 : 0]);
            field("control").value = "automatic";
            field("control").dispatchEvent(new dom.window.Event("change"));
            button("reset").click(); await tick();
            assert.equal(field("control").value, "automatic");
            button("step").click(); await tick();
            assert.equal(calls.at(-1).action, null);
        }
    } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
});

test("YAML behavior mode and bounded frame replay", async () => {
    const {dom, root, calls, workers, urls} = setup();
    const button = name => root.querySelector(`[data-action="${name}"]`);
    try {
        await tick(); button("launch").click(); await tick();
        workers[0].automatic = true;
        button("reset").click(); await tick();
        assert.equal(root.querySelector('[name="control"]').value, "automatic");
        assert.equal(root.querySelector('[name="speed"]').disabled, true);
        for (let i = 0; i < 205; i++) { button("step").click(); await tick(); }
        assert.equal(calls.at(-1).action, null);
        assert.equal(root.querySelector('[name="time"]').max, "199");
        assert.equal(urls.size, 200);
        dom.window.dispatchEvent(new dom.window.Event("pagehide"));
        assert.equal(urls.size, 0);
    } finally { dom.window.close(); }
});

for (const language of ["en", "zh-CN"]) {
    test(`object selection follows live snapshots and replay (${language})`, async () => {
        const {dom, root, calls, workers} = setup(language);
        const button = name => root.querySelector(`[data-action="${name}"]`);
        const field = name => root.querySelector(`[name="${name}"]`);
        const value = name => root.querySelector(`[data-${name}]`).value;
        try {
            await tick();
            assert.equal(field("object").disabled, true);
            button("launch").click(); await tick();
            workers[0].objects = steps => [
                {id: 0, name: "same", role: "robot", kinematics: "acker", shape: "rectangle", state: [steps, 2, 0.3, 0.1], velocity: [0.8, 0.2], goal: [4, 4, 0], collision: false, arrived: false, stopped: false, static: false},
                {id: 5, name: "same", role: "obstacle", kinematics: "omni", shape: "circle", state: [5, steps, 0], velocity: [0, 0.5], goal: [5, 5, 0], collision: steps > 0, arrived: false, stopped: steps > 0, static: false},
                {id: 9, name: "<b>wall</b>", role: "obstacle", kinematics: null, shape: "rectangle", state: [8, 8, 0], velocity: [0, 0], goal: null, collision: false, arrived: false, stopped: false, static: true},
            ];
            button("reset").click(); await tick();
            assert.equal(field("object").options.length, 3);
            assert.equal(value("state"), "[0.0000, 2.0000, 0.3000, 0.1000]");
            assert.equal(field("object").options[2].textContent, "<b>wall</b> · #9");
            assert.equal(field("object").querySelector("b"), null); // Object names are text, not markup.
            field("object").value = "5";
            field("object").dispatchEvent(new dom.window.Event("change"));
            assert.equal(value("model"), "obstacle / omni");
            assert.equal(value("velocity"), "[0.0000, 0.5000]");
            const frames = manualFrames(dom.window);
            button("run").click(); await tick();
            assert.equal(field("object").disabled, false);
            await frames.next(); button("run").click();
            assert.equal(field("object").value, "5");
            assert.equal(value("state"), "[5.0000, 2.0000, 0.0000]");
            assert.equal(root.querySelector('[data-flag="stopped"]').dataset.value, "true");
            const callCount = calls.length;
            field("time").value = "0"; field("time").dispatchEvent(new dom.window.Event("input"));
            assert.equal(value("state"), "[5.0000, 0.0000, 0.0000]");
            assert.match(root.querySelector("[data-inspect-time]").textContent, /0\.00 s/);
            assert.equal(root.querySelector('[data-flag="collision"]').dataset.value, "false");
            field("object").value = "9"; field("object").dispatchEvent(new dom.window.Event("change"));
            assert.equal(value("goal"), "—");
            assert.equal(value("model"), "obstacle / —");
            assert.equal(root.querySelector('[data-flag="static"]').dataset.value, "true");
            assert.equal(field("control").value, "manual"); // Inspecting an obstacle never redirects commands.
            assert.equal(calls.length, callCount);
            workers[0].objects = null; button("reset").click(); await tick();
            assert.equal(field("object").value, "0"); // Removed selections fall back to an existing object.
            assert.equal(field("object").options.length, 1);
            assert.equal(value("goal"), "[4.0000, 4.0000, 0.0000]");
        } finally { dom.window.dispatchEvent(new dom.window.Event("pagehide")); dom.window.close(); }
    });
}

test("file URLs get an HTTP hint without attempting to start a worker", async () => {
    const {dom, root, workers} = setup("en", "file:");
    try {
        await tick(); assert.equal(workers.length, 0);
        assert.match(root.querySelector(".kl-status").textContent, /HTTP/);
        assert.equal(root.querySelector('[data-action="launch"]').disabled, true);
    } finally { dom.window.close(); }
});
