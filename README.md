# mujoco-wasm-ik

在浏览器里使用 **MuJoCo 物理 + Mink 风格微分 IK**（minc + DAQP）的 WebAssembly 模块。IK 与 `MjModel` / `MjData` 编进**同一份** `mujoco.wasm`，不另起 wasm。

本仓库是库，**不包含**具体机械臂场景、Three.js 或夹爪逻辑。应用示例见同级目录的 xArm7 仿真项目。

```js
import loadMujoco from "mujoco-wasm-ik";

const mujoco = await loadMujoco();
const model = mujoco.MjModel.from_xml_string(xml);
const data = new mujoco.MjData(model);
const ik = new mujoco.IkConfiguration(model, data);
const ee = new mujoco.FrameTask("ee", "site", 1, 1, 1, 1);
ee.setTargetFromConfiguration(ik);
const vel = mujoco.solveIK(ik, [ee], [], dt, 1e-12);
```

预编译产物在 `dist/`。只当依赖用时不必安装 Emscripten，也不必拉取上游源码。

## 仓库布局

```
mujoco-wasm-ik/
├── package.json              # npm 入口 → dist/
├── versions.lock.json        # 当前采用的官方 tag
├── overlay/
│   ├── ik_bindings.cc        # IK Embind（改这里，不要改官方 bindings.cc）
│   ├── deps.cmake
│   ├── smoke.mjs
│   └── patches/wasm-CMakeLists.txt.patch
├── scripts/
│   ├── sync-deps.mjs         # 检测 / 询问 / clone 官方源
│   └── build.mjs             # sync → overlay → cmake → smoke
├── dist/                     # mujoco.js / mujoco.wasm / mujoco.d.ts
├── vendor/                   # gitignore：mujoco minc daqp eigen
└── emsdk/                    # gitignore：本地 Emscripten
```

`mujoco` / `minc` / `daqp` / `eigen` **不进 git**。构建时按 `versions.lock.json` 从官方仓库拉取到 `vendor/`，再打上 overlay。

## 构建 SOP

改了 `overlay/`、minc、DAQP 或 MuJoCo 后需要重编。

**不要改** `vendor/mujoco/wasm/codegen/generated/bindings.cc`。不要为 IK 单独再编一份 wasm。

浏览器默认没有 COOP/COEP，请编**单线程**：脚本已传 `-DMUJOCO_WASM_THREADS=OFF`。

### 1. 前置

| 工具 | 版本 |
|------|------|
| Emscripten | **4.0.10** |
| CMake | ≥ 3.16 |
| Ninja | 最新 |
| Node.js | ≥ 18 |
| git | 用于拉取官方 tag |
| patch | 用于打 overlay |

```bash
brew install cmake ninja
```

### 2. Emscripten

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install 4.0.10
./emsdk activate 4.0.10
source ./emsdk_env.sh
```

每次新开终端手动 cmake 前都要 source。`npm run build` 会自动 `source emsdk/emsdk_env.sh`（若该目录存在）。

### 3. 同步官方依赖并编译

```bash
npm run build
```

流程：

1. `git ls-remote` 对比 `versions.lock.json` 与官方最新 **semver tag**
2. 有新版本则询问是否更新（`[y/N]`）
3. 按选定 tag clone/checkout 到 `vendor/`
4. 拷贝 `overlay/ik_bindings.cc`、`deps.cmake`，并对官方 `wasm/CMakeLists.txt` 打补丁
5. `emcmake` + `ninja mujoco_wasm`（`THREADS=OFF`）
6. 复制产物到 `dist/`，跑 `npm run smoke`
7. **smoke 通过才写入新的 lock**；失败则不改 lock，留下 `vendor/` 方便对照

常用参数：

```bash
npm run build -- --no-update   # 不询问，按 lock 编译
npm run build -- --yes         # 全部升到最新 tag（CI）
npm run sync -- --no-update    # 只拉取，不编译
npm run smoke                  # 只测当前 dist/
```

第一次若 `vendor/` 已有无 `.git` 的目录，脚本会保留它们，避免重下一遍。之后若选择升级，会删掉该目录再 clone 官方 tag。

`dist/mujoco.d.ts` 必须能搜到 `solveIK`。换过 THREADS 选项时先删 `vendor/mujoco/build`。

### 4. 升级失败时改哪里

| 现象 | 改这里 |
|------|--------|
| `patch` 失败 | `overlay/patches/wasm-CMakeLists.txt.patch` 与官方 `wasm/CMakeLists.txt` |
| minc 新增 `.cpp` | `overlay/deps.cmake` 的源文件列表 |
| IK / Embind API 变了 | `overlay/ik_bindings.cc` |
| smoke 失败 | 同上，不要手改 `vendor/` 里的官方文件当长期方案 |

## 不要做的事

| 不要 | 原因 |
|------|------|
| 把 `vendor/` 提交进 git | 官方源码按 lock 拉取 |
| 手改 `codegen/generated/bindings.cc` | 官方生成文件 |
| 默认 cmake 不关 THREADS 给普通网页用 | 需要 Cross-Origin Isolation |
| npm 只发 `.js` 不发 `.wasm` | 必须同一次编译 |

## 许可

本仓库 overlay 为 Apache-2.0。上游见 [NOTICE](./NOTICE)：MuJoCo / MINC Apache-2.0，DAQP MIT，Eigen MPL2。
