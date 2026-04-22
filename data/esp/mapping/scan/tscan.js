export function parseTscanPayload(payload) {
  // payload examples:
  // - "BEGIN,+"
  // - "BEGIN,-"
  // - "DONE"
  // - "CANCEL"
  // - thin: "0.00,41.8"
  // - debug: "0,0.00,41.8,573,48331"
  const s = String(payload || "").trim();
  if (!s) return null;

  if (s.startsWith("BEGIN")) return { kind: "begin", payload: s };
  if (s.startsWith("DONE")) return { kind: "done" };
  if (s.startsWith("CANCEL")) return { kind: "cancel" };

  const parts = s.split(",");
  if (parts.length === 2) {
    const angleDeg = Number(parts[0]);
    const distCm = Number(parts[1]);
    if (!Number.isFinite(angleDeg) || !Number.isFinite(distCm)) return null;
    return { kind: "sample", seq: 0, angleDeg, distCm, ticksAbs: 0, ms: 0 };
  }
  if (parts.length === 5) {
    const seq = Number(parts[0]);
    const angleDeg = Number(parts[1]);
    const distCm = Number(parts[2]);
    const ticksAbs = Number(parts[3]);
    const ms = Number(parts[4]);
    if (!Number.isFinite(seq) || !Number.isFinite(angleDeg) || !Number.isFinite(distCm)) return null;
    return { kind: "sample", seq, angleDeg, distCm, ticksAbs, ms };
  }
  return null;
}
