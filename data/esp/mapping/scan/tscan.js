export function parseTscanPayload(payload) {
  // SECTION: Decode TSCAN payload variants emitted by ESP/Mega.
  // WHY: Supports control frames (`BEGIN|DONE|CANCEL`) and thin/full sample payloads.
  const s = String(payload || "").trim();
  if (!s) return null;

  if (s.startsWith("BEGIN")) return { kind: "begin", payload: s };
  if (s.startsWith("DONE")) return { kind: "done" };
  if (s.startsWith("CANCEL")) return { kind: "cancel" };

  const parts = s.split(",");
  if (parts.length === 2) {
    // CONTRACT: Thin samples omit sequence/ticks/time and default those fields to zero.
    const angleDeg = Number(parts[0]);
    const distCm = Number(parts[1]);
    if (!Number.isFinite(angleDeg) || !Number.isFinite(distCm)) return null;
    return { kind: "sample", seq: 0, angleDeg, distCm, ticksAbs: 0, ms: 0 };
  }
  if (parts.length === 5) {
    // CONTRACT: Full samples carry `seq,angle,dist,ticksAbs,ms`.
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
