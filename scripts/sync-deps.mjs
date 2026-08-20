import { spawnSync } from "node:child_process";
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import readline from "node:readline/promises";
import { stdin as input, stdout as output } from "node:process";

export const ROOT = dirname(dirname(fileURLToPath(import.meta.url)));
export const VENDOR = join(ROOT, "vendor");
export const LOCK_PATH = join(ROOT, "versions.lock.json");

export const DEPS = {
  mujoco: {
    repo: "https://github.com/google-deepmind/mujoco.git",
    dir: "mujoco",
  },
  minc: {
    repo: "https://github.com/martantoine/Minc.git",
    dir: "minc",
  },
  daqp: {
    repo: "https://github.com/darnstrom/daqp.git",
    dir: "daqp",
  },
  eigen: {
    repo: "https://gitlab.com/libeigen/eigen.git",
    dir: "eigen",
  },
};

export function parseArgs(argv = process.argv.slice(2)) {
  return {
    yes: argv.includes("--yes") || argv.includes("-y"),
    noUpdate: argv.includes("--no-update"),
  };
}

export function loadLock() {
  return JSON.parse(readFileSync(LOCK_PATH, "utf8"));
}

export function writeLock(lock) {
  writeFileSync(LOCK_PATH, `${JSON.stringify(lock, null, 2)}\n`);
}

function parseVer(tag) {
  const s = String(tag).replace(/^[vV]/, "");
  if (/rc|nightly|alpha|beta|pre/i.test(s)) {
    return null;
  }
  const m = s.match(/^(\d+)\.(\d+)\.(\d+)$/);
  if (!m) {
    return null;
  }
  return { raw: tag, n: [Number(m[1]), Number(m[2]), Number(m[3])] };
}

function cmpVer(a, b) {
  for (let i = 0; i < 3; i += 1) {
    if (a.n[i] !== b.n[i]) {
      return a.n[i] - b.n[i];
    }
  }
  return 0;
}

function latestTag(tags) {
  const parsed = tags.map(parseVer).filter(Boolean);
  if (parsed.length === 0) {
    return null;
  }
  parsed.sort(cmpVer);
  return parsed[parsed.length - 1].raw;
}

function sameRef(a, b) {
  return String(a).replace(/^[vV]/, "").toLowerCase() ===
    String(b).replace(/^[vV]/, "").toLowerCase();
}

function git(args, opts = {}) {
  return spawnSync("git", args, {
    encoding: "utf8",
    ...opts,
  });
}

function listRemoteTags(repo) {
  const result = git(["ls-remote", "--tags", "--refs", repo]);
  if (result.status !== 0) {
    throw new Error(
      `git ls-remote failed for ${repo}: ${result.stderr || result.stdout}`,
    );
  }
  return result.stdout
    .trim()
    .split("\n")
    .filter(Boolean)
    .map((line) => line.split("/").pop());
}

function hasGit(dir) {
  return existsSync(join(dir, ".git"));
}

async function promptUpdate(name, lockedTag, latest, options) {
  const question = `[${name}] locked ${lockedTag} → latest ${latest}  更新?`;
  if (options.noUpdate) {
    console.log(`${question} (--no-update, skip)`);
    return false;
  }
  if (options.yes) {
    console.log(`${question} (--yes)`);
    return true;
  }
  if (!process.stdin.isTTY) {
    console.log(`${question} (non-TTY, skip)`);
    return false;
  }
  const rl = readline.createInterface({ input, output });
  try {
    const answer = await rl.question(`${question} [y/N] `);
    return /^y(es)?$/i.test(answer.trim());
  } finally {
    rl.close();
  }
}

function checkoutTag(repo, dest, tag) {
  mkdirSync(VENDOR, { recursive: true });
  if (!existsSync(dest)) {
    console.log(`clone ${repo} @ ${tag} → ${dest}`);
    const result = git(
      ["clone", "--depth", "1", "--branch", String(tag), repo, dest],
      { stdio: "inherit" },
    );
    if (result.status !== 0) {
      throw new Error(`clone failed: ${repo} @ ${tag}`);
    }
    return;
  }
  if (!hasGit(dest)) {
    console.log(`keep ${dest} (no .git; already present for ${tag})`);
    return;
  }
  console.log(`checkout ${tag} in ${dest}`);
  const fetch = git(
    ["fetch", "--depth", "1", "origin", `refs/tags/${tag}:refs/tags/${tag}`],
    { cwd: dest, stdio: "inherit" },
  );
  const checkout = git(["checkout", "--force", String(tag)], {
    cwd: dest,
    stdio: "inherit",
  });
  if (fetch.status !== 0 || checkout.status !== 0) {
    throw new Error(`failed to checkout ${tag} in ${dest}`);
  }
}

function replaceCheckout(repo, dest, tag) {
  if (existsSync(dest)) {
    console.log(`replace ${dest} with ${tag}`);
    rmSync(dest, { recursive: true, force: true });
  }
  checkoutTag(repo, dest, tag);
}

export async function syncDeps(options = parseArgs()) {
  const lock = loadLock();
  const pending = structuredClone(lock);
  let changed = false;

  for (const [name, meta] of Object.entries(DEPS)) {
    const locked = lock.deps[name];
    if (!locked?.tag) {
      throw new Error(`versions.lock.json missing tag for ${name}`);
    }
    const dest = join(VENDOR, meta.dir);
    const tags = listRemoteTags(meta.repo);
    const latest = latestTag(tags);
    let want = locked.tag;

    if (latest && !sameRef(latest, locked.tag)) {
      const update = await promptUpdate(name, locked.tag, latest, options);
      if (update) {
        want = latest;
        pending.deps[name] = { ...locked, repo: meta.repo, tag: latest };
        changed = true;
        if (existsSync(dest) && !hasGit(dest)) {
          replaceCheckout(meta.repo, dest, want);
          continue;
        }
      }
    } else {
      console.log(
        `[${name}] locked ${locked.tag}${latest ? ` (latest ${latest})` : " (no semver tags)"}`,
      );
    }

    checkoutTag(meta.repo, dest, want);
  }

  return { lock, pending, changed };
}

const thisFile = fileURLToPath(import.meta.url);
const invoked = process.argv[1] ? resolve(process.argv[1]) : "";
if (invoked === thisFile) {
  await syncDeps(parseArgs());
}
