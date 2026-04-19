// bundle-folders.mjs
// Usage:
//   node bundle-folders.mjs --out codebundle.md src backend/dal backend/db/schema
// Options:
//   --out <file>          Output markdown file (default: codebundle.md)
//   --ext <csv>           File extensions to include (default: ts,tsx,js,json,sql,md,yml,yaml,sh,py,css,html)
//   --max-bytes <num>     Skip files larger than this (default: 500000)
//   --git                 Use `git ls-files` (respects .gitignore) limited to given folders
//   --exclude <csv>       Extra exclude globs (simple contains checks), e.g. ".env,dist,.git,node_modules"

// Examples
// Bundle exactly two folders:
// node bundle-folders.mjs --out kielo-mvp.md backend/dal backend/db/schema
// Respect .gitignore and your pathspecs (recommended if it’s a git repo):
// node bundle-folders.mjs --git --out kielo-mvp.md backend backend/db/schema
// Custom extensions & excludes:
// node bundle-folders.mjs --out code.md --ext ts,sql,md --exclude ".env,dist" backend/dal backend/db/schema

import { execSync } from "node:child_process";
import { promises as fs } from "node:fs";
import { basename, extname, join, relative, sep } from "node:path";
import { fileURLToPath } from "node:url";

const CWD = process.cwd();

function parseArgs(argv) {
  const args = { folders: [] };
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--out") args.out = argv[++i];
    else if (a === "--ext") args.ext = argv[++i];
    else if (a === "--max-bytes") args.maxBytes = Number(argv[++i]);
    else if (a === "--git") args.git = true;
    else if (a === "--exclude") args.exclude = argv[++i];
    else if (!a.startsWith("-")) args.folders.push(a);
  }
  return args;
}

const args = parseArgs(process.argv);
if (!args.folders || args.folders.length === 0) {
  console.error("Provide one or more folders:\n  node bundle-folders.mjs --out codebundle.md backend/dal backend/db/schema");
  process.exit(1);
}

const OUT = args.out ?? "codebundle.md";
const INCLUDE_EXT = new Set((args.ext ?? "ts,tsx,js,json,sql,md,yml,yaml,sh,py,css,html").split(",").map((s) => "." + s.trim().replace(/^\./, "").toLowerCase()));
const MAX_FILE_BYTES = Number.isFinite(args.maxBytes) ? args.maxBytes : 500_000;

// Default exclusions (dir/filename contains any of these)
const DEFAULT_EXCLUDES = [
  "node_modules",
  ".git",
  ".next",
  ".turbo",
  ".DS_Store",
  ".env",
  ".env.local",
  ".env.development",
  ".env.production",
  ".env.test",
  ".lock",
  ".png",
  ".jpg",
  ".jpeg",
  ".gif",
  ".webp",
  ".pdf",
  ".mp3",
  ".mp4",
  ".wav",
  ".zip",
  ".7z",
  ".tar",
  ".gz",
];
const EXTRA_EXCLUDES = (args.exclude ?? "")
  .split(",")
  .map((s) => s.trim())
  .filter(Boolean);

const EXCLUDES = DEFAULT_EXCLUDES.concat(EXTRA_EXCLUDES);

function looksExcluded(path) {
  const lower = path.toLowerCase();
  return EXCLUDES.some((needle) => needle && lower.includes(needle.toLowerCase()));
}

function langFence(path) {
  const ext = extname(path).toLowerCase().slice(1);
  return ["ts", "tsx", "js", "json", "sql", "md", "yml", "yaml", "sh", "py", "css", "html"].includes(ext) ? ext : "";
}

async function listFilesFS(folders) {
  const out = [];
  async function walk(dir) {
    let entries = [];
    try {
      entries = await fs.readdir(dir, { withFileTypes: true });
    } catch {
      return;
    }
    for (const e of entries) {
      const p = join(dir, e.name);
      if (looksExcluded(p)) continue;
      if (e.isDirectory()) {
        await walk(p);
      } else {
        out.push(p);
      }
    }
  }
  for (const f of folders) {
    await walk(join(CWD, f));
  }
  return out;
}

function listFilesGit(folders) {
  try {
    const args = ["ls-files", "--"].concat(folders);
    const out = execSync(`git ${args.map((s) => `'${s.replace(/'/g, `'\\''`)}'`).join(" ")}`, {
      stdio: ["ignore", "pipe", "ignore"],
      cwd: CWD,
      encoding: "utf8",
    });
    return out
      .split("\n")
      .filter(Boolean)
      .map((p) => join(CWD, p));
  } catch {
    return [];
  }
}

function onlyWantedExt(paths) {
  return paths.filter((p) => INCLUDE_EXT.has(extname(p).toLowerCase()));
}

function normalizeRel(p) {
  return relative(CWD, p).split(sep).join("/");
}

async function readSafe(p) {
  try {
    const stat = await fs.stat(p);
    if (!stat.isFile()) return null;
    if (stat.size > MAX_FILE_BYTES) return null;
    return await fs.readFile(p, "utf8");
  } catch {
    return null;
  }
}

(async () => {
  let files = args.git ? listFilesGit(args.folders) : await listFilesFS(args.folders);
  if (!files.length && args.git) {
    // fallback to FS walk if git failed or not a repo
    files = await listFilesFS(args.folders);
  }

  // filter by ext and excludes again (git may include ignored paths if misused)
  files = onlyWantedExt(files).filter((p) => !looksExcluded(p));

  // sort for stable output
  files.sort((a, b) => normalizeRel(a).localeCompare(normalizeRel(b)));

  let out = `# Code Bundle (${new Date().toISOString()})\n`;
  let kept = 0,
    skipped = 0;

  for (const f of files) {
    const txt = await readSafe(f);
    if (txt == null) {
      skipped++;
      continue;
    }
    const rel = normalizeRel(f);
    out += `\n---\n## ${rel}\n\n\`\`\`${langFence(f)}\n${txt}\n\`\`\`\n`;
    kept++;
  }

  await fs.writeFile(OUT, out, "utf8");
  console.log(`Wrote ${OUT}`);
  console.log(`Files included: ${kept}, skipped: ${skipped}`);
})();
