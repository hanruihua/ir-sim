// Shared by the module worker and the real WebAssembly integration test.
export async function initialize(pyodide, manifest, archive, adapter) {
    await pyodide.loadPackage(manifest.packages);
    const requirements = pyodide.toPy(manifest.requirements);
    pyodide.globals.set("_requirements", requirements);
    try {
        await pyodide.runPythonAsync("import micropip\nawait micropip.install(_requirements)");
    } finally {
        pyodide.globals.delete("_requirements");
        requirements.destroy();
    }
    pyodide.unpackArchive(new Uint8Array(archive), "zip", {extractDir: "/irsim-src"});
    await pyodide.runPythonAsync("import sys\nsys.path.insert(0, '/irsim-src')");
    await pyodide.runPythonAsync(adapter);
    const version = pyodide.runPython("irsim.__version__");
    if (version !== manifest.version) throw new Error("IR-SIM source/version mismatch.");
    return {version, source: manifest.sha256, pyodide: manifest.pyodide};
}

export function dispatch(pyodide, request) {
    // Data goes through a Python variable, never interpolated into Python code.
    pyodide.globals.set("_request_json", JSON.stringify(request));
    try {
        const result = JSON.parse(pyodide.runPython("handle_request(_request_json)"));
        const frame = pyodide.runPython("playground.frame");
        try { result.frame = frame.toJs(); } finally { frame.destroy(); }
        return result;
    } finally {
        pyodide.globals.delete("_request_json");
    }
}
