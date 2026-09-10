import {initialize, dispatch} from "./runtime.mjs";

let pyodide;
async function resource(name, kind = "text") {
    const response = await fetch(new URL(name, import.meta.url));
    if (!response.ok) throw new Error(`${name}: HTTP ${response.status}`);
    return response[kind]();
}

self.onmessage = async ({data}) => {
    const {id, ...request} = data;
    try {
        let result;
        if (request.method === "initialize") {
            const manifest = await resource("manifest.json", "json");
            self.postMessage({status: "python"});
            const base = `https://cdn.jsdelivr.net/pyodide/v${manifest.pyodide}/full/`;
            const {loadPyodide} = await import(`${base}pyodide.mjs`);
            pyodide = await loadPyodide({indexURL: base});
            self.postMessage({status: "packages"});
            const [archive, adapter] = await Promise.all([
                resource(manifest.archive, "arrayBuffer"), resource("simulation.py"),
            ]);
            const digest = await crypto.subtle.digest("SHA-256", archive);
            const hash = Array.from(new Uint8Array(digest), b => b.toString(16).padStart(2, "0")).join("");
            if (hash !== manifest.sha256) throw new Error("IR-SIM source checksum mismatch.");
            result = await initialize(pyodide, manifest, archive, adapter);
        } else {
            if (!pyodide) throw new Error("Python is not ready.");
            result = dispatch(pyodide, request);
        }
        self.postMessage({id, result}, result.frame ? [result.frame.buffer] : []);
    } catch (error) {
        self.postMessage({id, error: String(error)});
    }
};
