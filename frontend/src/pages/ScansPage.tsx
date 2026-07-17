import { useCallback, useEffect, useRef, useState } from "react";
import { Link, useParams } from "react-router-dom";

import { deleteScan, listScans, reprocess, updateScanSettings, uploadInput, uploadScan } from "../api";
import type { InputKind, Scan } from "../types";

function formatBytes(n: number | null): string {
  if (n === null) return "—";
  if (n < 1024 ** 2) return `${(n / 1024).toFixed(1)} КиБ`;
  if (n < 1024 ** 3) return `${(n / 1024 ** 2).toFixed(1)} МиБ`;
  return `${(n / 1024 ** 3).toFixed(2)} ГиБ`;
}

const PPK_KINDS: { kind: InputKind; label: string }[] = [
  { kind: "trajectory", label: "Траектория (.pos)" },
  { kind: "rover_obs", label: "Ровер RINEX (obs)" },
  { kind: "base_rinex", label: "База RINEX" },
  { kind: "nav", label: "Эфемериды (nav)" },
];

function PpkPanel({ scanId, onDone }: { scanId: string; onDone: () => void }) {
  const [busy, setBusy] = useState<string | null>(null);
  const [message, setMessage] = useState<string | null>(null);

  async function attach(kind: InputKind, file: File) {
    setBusy(kind);
    setMessage(null);
    try {
      await uploadInput(scanId, kind, file);
      setMessage(`Файл «${file.name}» прикреплён (${kind}).`);
    } catch (err) {
      setMessage(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  }

  async function runReprocess() {
    setBusy("reprocess");
    setMessage(null);
    try {
      await reprocess(scanId);
      setMessage("Пересчёт запущен (PPK → георепривязка → octree), распаковка не повторяется.");
      onDone();
    } catch (err) {
      setMessage(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(null);
    }
  }

  return (
    <div style={{ padding: "8px 10px" }}>
      {PPK_KINDS.map(({ kind, label }) => (
        <div className="row" key={kind} style={{ marginBottom: 6 }}>
          <span style={{ width: 180 }} className="muted">
            {label}
          </span>
          <input
            type="file"
            disabled={busy !== null}
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) void attach(kind, f);
              e.target.value = "";
            }}
          />
        </div>
      ))}
      <div className="row" style={{ marginTop: 8 }}>
        <button disabled={busy !== null} onClick={() => void runReprocess()}>
          Пересчитать с PPK-коррекцией
        </button>
        {message && <span className="muted">{message}</span>}
      </div>
    </div>
  );
}

export default function ScansPage() {
  const { projectId } = useParams<{ projectId: string }>();
  const [scans, setScans] = useState<Scan[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [progress, setProgress] = useState<number | null>(null);
  const [ppkFor, setPpkFor] = useState<string | null>(null);
  const [reprocessPending, setReprocessPending] = useState<Set<string>>(new Set());
  const fileInput = useRef<HTMLInputElement>(null);

  const refresh = useCallback(async () => {
    if (!projectId) return;
    try {
      setScans(await listScans(projectId));
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [projectId]);

  useEffect(() => {
    void refresh();
    const timer = setInterval(() => void refresh(), 5000);
    return () => clearInterval(timer);
  }, [refresh]);

  async function onFile(file: File) {
    if (!projectId) return;
    setError(null);
    setProgress(0);
    try {
      await uploadScan(projectId, file, setProgress);
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setProgress(null);
    }
  }

  async function handleDeleteScan(scanId: string) {
    if (!confirm("Удалить скан? Все данные будут удалены навсегда.")) return;
    try {
      await deleteScan(scanId);
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  async function handleTogglePhotogrammetry(scanId: string, enabled: boolean) {
    try {
      await updateScanSettings(scanId, { photogrammetry_enabled: enabled });
      setReprocessPending((prev) => {
        const next = new Set(prev);
        next.add(scanId);
        return next;
      });
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  async function handleReprocess(scanId: string) {
    try {
      await reprocess(scanId, "dense_stereo");
      setReprocessPending((prev) => {
        const next = new Set(prev);
        next.delete(scanId);
        return next;
      });
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  async function handleReprocessFull(scanId: string) {
    if (!confirm("Запустить полный пересчёт с нуля? Все промежуточные данные будут пересчитаны."))
      return;
    try {
      await reprocess(scanId, "decode_raw");
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  return (
    <div className="page">
      <h1>
        <Link to="/">Проекты</Link> / Сканы
      </h1>
      {error && <p className="error">{error}</p>}

      <div className="card row">
        <input
          ref={fileInput}
          type="file"
          accept=".zip"
          style={{ display: "none" }}
          onChange={(e) => {
            const f = e.target.files?.[0];
            if (f) void onFile(f);
            e.target.value = "";
          }}
        />
        <button disabled={progress !== null} onClick={() => fileInput.current?.click()}>
          Загрузить данные
        </button>
        {progress !== null && (
          <>
            <div className="progress">
              <div style={{ width: `${Math.round(progress * 100)}%` }} />
            </div>
            <span className="muted">{Math.round(progress * 100)}%</span>
          </>
        )}
      </div>

      <div className="card">
        {scans === null ? (
          <p className="muted">Загрузка…</p>
        ) : scans.length === 0 ? (
          <p className="muted">Сканов пока нет.</p>
        ) : (
          <table>
            <thead>
              <tr>
                <th>Скан</th>
                <th>Статус</th>
                <th>Размер</th>
                <th>RTK</th>
                <th title="Включить Dense Stereo фотограмметрию (требует стерео-камеры в bag-файле)">Фото</th>
                <th>Создан</th>
                <th />
              </tr>
            </thead>
            <tbody>
              {scans.map((s) => (
                <>
                  <tr key={s.id}>
                    <td>
                      <Link to={`/scans/${s.id}`}>{s.id.slice(0, 8)}…</Link>
                    </td>
                    <td>
                      <span className={`badge ${s.status}`}>{s.status}</span>
                    </td>
                    <td>{formatBytes(s.size_bytes)}</td>
                    <td>{s.rtk_fixed ? <span className="badge rtk">fixed</span> : "—"}</td>
                    <td style={{ whiteSpace: "nowrap" }}>
                      <input
                        type="checkbox"
                        checked={s.photogrammetry_enabled}
                        onChange={(e) =>
                          void handleTogglePhotogrammetry(s.id, e.target.checked)
                        }
                        title="Включить фотограмметрию"
                      />
                      {reprocessPending.has(s.id) && (
                        <button
                          className="secondary"
                          onClick={() => void handleReprocess(s.id)}
                          title="Запустить пересчёт с Dense Stereo"
                          style={{ marginLeft: 6, fontSize: "0.8em" }}
                        >
                          Пересчитать
                        </button>
                      )}
                    </td>
                    <td className="muted">{new Date(s.created_at).toLocaleString()}</td>
                    <td style={{ whiteSpace: "nowrap" }}>
                      <button
                        className="secondary"
                        onClick={() => void handleReprocessFull(s.id)}
                        title="Пересчитать с нуля (decode_raw → build_octree)"
                        style={{ marginRight: 4 }}
                      >
                        ↺
                      </button>
                      <button
                        className="secondary"
                        onClick={() => setPpkFor(ppkFor === s.id ? null : s.id)}
                        style={{ marginRight: 4 }}
                      >
                        PPK
                      </button>
                      <button
                        className="secondary"
                        onClick={() => void handleDeleteScan(s.id)}
                        title="Удалить скан"
                        style={{ padding: "2px 8px", color: "#c00" }}
                      >
                        ✕
                      </button>
                    </td>
                  </tr>
                  {ppkFor === s.id && (
                    <tr key={`${s.id}-ppk`}>
                      <td colSpan={6}>
                        <PpkPanel scanId={s.id} onDone={() => void refresh()} />
                      </td>
                    </tr>
                  )}
                </>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}
