// Optional integration check: NODE_PATH=<pyodide install>/node_modules
// IRSIM_BUNDLE=<built docs>/_static/playground node docs/tests/pyodide_smoke.cjs
const fs = require("node:fs");
const path = require("node:path");
const {pathToFileURL} = require("node:url");
const assert = require("node:assert/strict");
const {loadPyodide} = require("pyodide");

(async () => {
    const assets = path.resolve(__dirname, "../source/_static/playground");
    const bundle = process.env.IRSIM_BUNDLE || path.resolve(__dirname, "../build/html/_static/playground");
    const manifest = JSON.parse(fs.readFileSync(path.join(bundle, "manifest.json"), "utf8"));
    assert.equal(require("pyodide/package.json").version, manifest.pyodide);
    const {initialize, dispatch} = await import(pathToFileURL(path.join(assets, "runtime.mjs")));
    const py = await loadPyodide({packageCacheDir: process.env.PYODIDE_CACHE});
    const metadata = await initialize(py, manifest,
        fs.readFileSync(path.join(bundle, manifest.archive)),
        fs.readFileSync(path.join(assets, "simulation.py"), "utf8"));
    assert.equal(metadata.version, manifest.version);
    const source = fs.readFileSync(path.join(assets, "kinematics.yaml"), "utf8");
    for (const dt of [0.2, 0.05]) {
        let result = dispatch(py, {method: "create", yaml: source.replace("step_time: 0.2", `step_time: ${dt}`)});
        for (let i = 0; i < Math.round(4 / dt); i++) result = dispatch(py, {method: "step", action: [0.8, 0.6]});
        assert.equal(result.time, 4);
        assert.equal(Buffer.from(result.frame).subarray(1, 4).toString(), "PNG");
        assert.equal(result.frame_time, 4);
        assert.equal(result.objects[0].kinematics, "diff");
        assert.deepEqual(result.objects[0].velocity, [0.8, 0.6]);
        assert.deepEqual(result.objects[0].goal, [4, 4, 0]);
        if (dt === 0.2) result.objects[0].state.forEach((v, i) => assert.ok(Math.abs(v - [1.03852807, 2.25970740, 2.4][i]) < 1e-8));
        console.log("real IR-SIM", metadata.version, "dt", dt, "state", result.objects[0].state);
    }
    let result = dispatch(py, {method: "create", yaml: fs.readFileSync(path.join(assets, "obstacle.yaml"), "utf8")});
    for (let i = 0; i < 100 && !result.done; i++) result = dispatch(py, {method: "step", action: [0.8, 0]});
    assert.ok(result.done && result.objects[0].collision);
    assert.equal(result.objects[0].stopped, true);
    assert.equal(result.objects[1].static, true);
    assert.deepEqual(result.objects[1].goal, JSON.parse(py.runPython("json.dumps(playground.env.objects[1].goal.ravel().tolist())")));
    assert.ok(result.objects[0].state[0] < 5);
    for (const sensor of ["lidar2d", "fmcw_lidar2d"]) {
        dispatch(py, {method: "create", yaml: fs.readFileSync(path.join(assets, "lidar.yaml"), "utf8").replace("name: lidar2d", `name: ${sensor}`)});
        const start = performance.now();
        for (let i = 0; i < 10; i++) result = dispatch(py, {method: "step"});
        assert.equal(Buffer.from(result.frame).subarray(1, 4).toString(), "PNG");
        assert.equal(result.time, 1);
        assert.ok(py.runPython("len(playground.env._env_plot.ax.collections)") > 0);
        console.log(sensor, "native render ms/frame", ((performance.now() - start) / 10).toFixed(1), "PNG bytes", result.frame.byteLength);
    }
    const robots = ["diff", "omni", "acker"].map((name, y) => `
  - kinematics: {name: ${name}}
    shape: {name: rectangle, length: 0.6, width: 0.3, wheelbase: 0.4}
    state: [0, ${y}, 0]
    goal: [4, ${y}, 0]
    behavior: {name: dash}`).join("\n");
    dispatch(py, {method: "create", yaml: `world: {step_time: 0.1}\nrobot:\n${robots}`});
    result = dispatch(py, {method: "step", count: 3});
    assert.equal(result.manual_control, false);
    assert.ok(result.objects.every(obj => obj.state[0] > 0));
    console.log("PASS: diff/omni/acker multi-robot YAML behaviors");
    assert.throws(() => dispatch(py, {method: "create", yaml: "[]"}));
    console.log("PASS: source bundle, make/step, Matplotlib PNG, compound, sensors, collision, invalid YAML");
})().catch(error => { console.error(String(error)); process.exitCode = 1; });
