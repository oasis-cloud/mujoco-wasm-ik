import { spawnSync } from "node:child_process";
import { copyFileSync, existsSync, mkdirSync, readFileSync } from "node:fs";
import { join } from "node:path";
import { ROOT, VENDOR, LOCK_PATH, parseArgs, syncDeps, writeLock } from "./sync-deps.mjs";

const PATCH = join(ROOT, "overlay/patches/wasm-CMakeLists.txt.patch");
const OVERLAY_BINDINGS = join(ROOT, "overlay/ik_bindings.cc");
const OVERLAY_DEPS = join(ROOT, "overlay/deps.cmake");
const SMOKE = join(ROOT, "overlay/smoke.mjs");
const DIST = join(ROOT, "dist");
const MUJOCO = join(VENDOR, "mujoco");

function fail(message) {
  console.error(message);
  process.exit(1);
}

function bash(script, cwd) {
  const emsdkEnv = join(ROOT, "emsdk/emsdk_env.sh");
  const wrapped = existsSync(emsdkEnv)
    ? `source "${emsdkEnv}" && ${script}`
    : script;
  const result = spawnSync("bash", ["-lc", wrapped], {
    cwd,
    stdio: "inherit",
    encoding: "utf8",
  });
  if (result.status !== 0) {
    fail(`build command failed:\n${script}`);
  }
}

export function applyOverlay() {
  if (!existsSync(join(MUJOCO, "wasm/CMakeLists.txt"))) {
    fail(
      `vendor/mujoco is missing wasm/CMakeLists.txt. Run npm run sync, or clone MuJoCo into vendor/mujoco.`,
    );
  }

  const ikDir = join(MUJOCO, "wasm/ik");
  mkdirSync(ikDir, { recursive: true });
  copyFileSync(OVERLAY_BINDINGS, join(ikDir, "ik_bindings.cc"));
  copyFileSync(OVERLAY_DEPS, join(ikDir, "deps.cmake"));

  const cmake = readFileSync(join(MUJOCO, "wasm/CMakeLists.txt"), "utf8");
  if (cmake.includes("ik/deps.cmake")) {
    console.log("wasm/CMakeLists.txt already includes IK overlay, skip patch");
    return;
  }

  const patched = spawnSync(
    "patch",
    ["-p1", "--forward", "--batch", "-i", PATCH],
    {
      cwd: MUJOCO,
      encoding: "utf8",
    },
  );
  if (patched.status !== 0) {
    fail(`Failed to apply overlay/patches/wasm-CMakeLists.txt.patch

Upstream likely changed wasm/CMakeLists.txt. Inspect:
  overlay/patches/wasm-CMakeLists.txt.patch
  vendor/mujoco/wasm/CMakeLists.txt

If minc added sources, update overlay/deps.cmake.
If IK APIs changed, update overlay/ik_bindings.cc.
Then re-run: npm run build

patch stderr:
${patched.stderr || patched.stdout || ""}`);
  }
  console.log("applied overlay/patches/wasm-CMakeLists.txt.patch");
}

function compileWasm() {
  if (!existsSync(join(MUJOCO, "wasm/package.json"))) {
    fail("vendor/mujoco/wasm/package.json missing");
  }
  bash("npm install --prefix ./wasm", MUJOCO);
  bash(
    "emcmake cmake -B build -G Ninja -DMUJOCO_WASM_THREADS=OFF -DMUJOCO_BUILD_TESTS_WASM=OFF && cmake --build build --target mujoco_wasm -j",
    MUJOCO,
  );
}

function copyDist() {
  const src = join(MUJOCO, "wasm/dist");
  mkdirSync(DIST, { recursive: true });
  for (const name of ["mujoco.js", "mujoco.wasm", "mujoco.d.ts"]) {
    const from = join(src, name);
    if (!existsSync(from)) {
      fail(`missing build artifact ${from}`);
    }
    copyFileSync(from, join(DIST, name));
  }
  const dts = readFileSync(join(DIST, "mujoco.d.ts"), "utf8");
  if (!dts.includes("solveIK")) {
    fail("dist/mujoco.d.ts has no solveIK; IK overlay was not linked.");
  }
  console.log("copied wasm artifacts to dist/");
}

function smoke() {
  const result = spawnSync(process.execPath, [SMOKE], {
    cwd: ROOT,
    stdio: "inherit",
    encoding: "utf8",
  });
  if (result.status !== 0) {
    fail(`smoke failed (exit ${result.status}). versions.lock.json was not updated.

Fix overlay/ik_bindings.cc, overlay/deps.cmake, or overlay/patches/wasm-CMakeLists.txt.patch, then re-run npm run build.`);
  }
}

const { yes, noUpdate } = parseArgs();
const { pending, changed } = await syncDeps({ yes, noUpdate });
applyOverlay();
compileWasm();
copyDist();
smoke();

if (changed) {
  writeLock(pending);
  console.log(`updated ${LOCK_PATH}`);
} else {
  console.log("versions.lock.json unchanged");
}
