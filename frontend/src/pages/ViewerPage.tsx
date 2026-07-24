import { useEffect, useRef, useState } from "react";
import { Link, useParams } from "react-router-dom";

import { downloadScan, scanPreview, scanStatus } from "../api";
import type { Preview, ScanDetails } from "../types";
import { ColorMode, CopcViewer, ViewerStats } from "../viewer/copcViewer";

export default function ViewerPage() {
  const { scanId } = useParams<{ scanId: string }>();
  const containerRef = useRef<HTMLDivElement>(null);
  const [details, setDetails] = useState<ScanDetails | null>(null);
  const [preview, setPreview] = useState<Preview | null>(null);
  const [stats, setStats] = useState<ViewerStats | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [downloading, setDownloading] = useState(false);
  const [colorMode, setColorMode] = useState<ColorMode>("rgb");
  const viewerRef = useRef<CopcViewer | null>(null);

  // Poll status while the pipeline is running.
  useEffect(() => {
    if (!scanId) return;
    let stopped = false;
    async function poll() {
      try {
        const d = await scanStatus(scanId!);
        if (stopped) return;
        setDetails(d);
        if (d.status === "completed" || d.status === "failed") return;
        setTimeout(() => void poll(), 3000);
      } catch (err) {
        if (!stopped) setError(err instanceof Error ? err.message : String(err));
      }
    }
    void poll();
    return () => {
      stopped = true;
    };
  }, [scanId]);

  // Start the streaming viewer once processing is done and a COPC exists.
  useEffect(() => {
    if (!scanId || details?.status !== "completed" || !containerRef.current) return;
    let viewer: CopcViewer | null = null;
    let cancelled = false;
    void (async () => {
      try {
        const p = await scanPreview(scanId);
        if (cancelled) return;
        setPreview(p);
        if (!p.copc_url || !containerRef.current) return;
        viewer = new CopcViewer(containerRef.current, p.copc_url, setStats);
        viewerRef.current = viewer;
        await viewer.load();
      } catch (err) {
        if (!cancelled) setError(err instanceof Error ? err.message : String(err));
      }
    })();
    return () => {
      cancelled = true;
      viewer?.dispose();
      viewerRef.current = null;
    };
  }, [scanId, details?.status]);

  function chooseColorMode(mode: ColorMode) {
    setColorMode(mode);
    viewerRef.current?.setColorMode(mode);
  }

  async function onDownload() {
    if (!scanId) return;
    setDownloading(true);
    setError(null);
    try {
      await downloadScan(scanId, `${scanId}.laz`);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setDownloading(false);
    }
  }

  return (
    <div className="viewer-wrap">
      <div ref={containerRef} className="viewer-canvas" />
      <div className="viewer-hud">
        <h2>
          <Link to={details ? `/projects/${details.project_id}` : "/"}>← Сканы</Link>
        </h2>
        {error && <p className="error">{error}</p>}
        {details && (
          <>
            <p>
              Статус: <span className={`badge ${details.status}`}>{details.status}</span>{" "}
              {details.rtk_fixed && <span className="badge rtk">RTK fixed</span>}
            </p>
            {details.status !== "completed" && (
              <ul style={{ listStyle: "none", marginTop: 6 }}>
                {details.jobs.map((j) => (
                  <li key={j.id}>
                    <span className={`badge ${j.status}`}>{j.status}</span>{" "}
                    <span className="muted">{j.pipeline_step}</span>
                    {j.error_message && <div className="error">{j.error_message}</div>}
                  </li>
                ))}
              </ul>
            )}
            <p className="muted" style={{ marginTop: 6 }}>
              Точек: {details.num_points?.toLocaleString() ?? "—"} · CRS:{" "}
              {details.crs_epsg ? `EPSG:${details.crs_epsg}` : "—"}
            </p>
          </>
        )}
        {stats && (
          <p className="muted">
            Загружено узлов октодерева: {stats.loadedNodes}/{stats.totalNodes} (
            {stats.loadedPoints.toLocaleString()} точек){stats.done ? "" : "…"}
          </p>
        )}
        {stats && (
          <label className="muted" style={{ marginTop: 6, display: "block" }}>
            Отображение:{" "}
            <select
              value={colorMode}
              onChange={(e) => chooseColorMode(e.target.value as ColorMode)}
            >
              <option value="height">Высота</option>
              <option value="intensity" disabled={!stats.hasIntensity}>
                Интенсивность
              </option>
              <option value="rgb" disabled={!stats.hasRgb}>
                RGB
              </option>
            </select>
          </label>
        )}
        {details?.status === "completed" && preview && !preview.copc_url && (
          <p className="muted">COPC-ассет отсутствует — просмотр недоступен, но LAZ можно скачать.</p>
        )}
        {details?.status === "completed" && (
          <div style={{ marginTop: 8 }}>
            <button disabled={downloading} onClick={() => void onDownload()}>
              {downloading ? "Скачивание…" : "Экспорт LAZ"}
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
