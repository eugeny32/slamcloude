import type {
  InputKind,
  Preview,
  Project,
  Scan,
  ScanDetails,
  ScanInput,
  UploadInit,
} from "./types";
export type { Scan };

export const API_URL: string =
  (import.meta.env.VITE_API_URL as string | undefined) ?? "http://localhost:8000";

const KEY_STORAGE = "slamcloude_api_key";

export function getApiKey(): string | null {
  return localStorage.getItem(KEY_STORAGE);
}

export function setApiKey(key: string): void {
  localStorage.setItem(KEY_STORAGE, key);
}

export function clearApiKey(): void {
  localStorage.removeItem(KEY_STORAGE);
}

export class ApiError extends Error {
  constructor(
    public status: number,
    detail: string,
  ) {
    super(detail);
  }
}

async function request<T>(path: string, init: RequestInit = {}): Promise<T> {
  const headers = new Headers(init.headers);
  const key = getApiKey();
  if (key) headers.set("X-API-Key", key);
  if (init.body && typeof init.body === "string") {
    headers.set("Content-Type", "application/json");
  }
  const res = await fetch(`${API_URL}${path}`, { ...init, headers });
  if (!res.ok) {
    let detail = res.statusText;
    try {
      const body = (await res.json()) as { detail?: unknown };
      if (typeof body.detail === "string") detail = body.detail;
    } catch {
      /* non-JSON error body */
    }
    throw new ApiError(res.status, detail);
  }
  if (res.status === 204 || res.headers.get("content-length") === "0") {
    return undefined as T;
  }
  return (await res.json()) as T;
}

export const listProjects = (): Promise<Project[]> => request("/projects");

export const createProject = (name: string): Promise<Project> =>
  request("/projects", { method: "POST", body: JSON.stringify({ name }) });

export const listScans = (projectId: string): Promise<Scan[]> =>
  request(`/projects/${projectId}/scans`);

export const scanStatus = (scanId: string): Promise<ScanDetails> =>
  request(`/scans/${scanId}/status`);

export const scanPreview = (scanId: string): Promise<Preview> =>
  request(`/scans/${scanId}/preview`);

export const listInputs = (scanId: string): Promise<ScanInput[]> =>
  request(`/scans/${scanId}/inputs`);

export const reprocess = (scanId: string, fromStep = "ppk_correction"): Promise<unknown> =>
  request(`/scans/${scanId}/reprocess`, {
    method: "POST",
    body: JSON.stringify({ from_step: fromStep }),
  });

/** Chunked upload: init -> PUT parts (File.slice, streamed) -> complete. */
export async function uploadScan(
  projectId: string,
  file: File,
  onProgress: (fraction: number) => void,
): Promise<ScanDetails> {
  const init = await request<UploadInit>("/scans/upload", {
    method: "POST",
    body: JSON.stringify({
      project_id: projectId,
      filename: file.name,
      file_size: file.size,
    }),
  });

  const parts: { part_number: number; etag: string }[] = [];
  for (let i = 0; i < init.num_parts; i++) {
    const blob = file.slice(i * init.part_size, Math.min(file.size, (i + 1) * init.part_size));
    const res = await request<{ part_number: number; etag: string }>(
      `/scans/${init.scan_id}/upload/parts/${i + 1}` +
        `?upload_id=${encodeURIComponent(init.upload_id)}`,
      { method: "PUT", body: blob },
    );
    parts.push({ part_number: res.part_number, etag: res.etag });
    onProgress((i + 1) / init.num_parts);
  }

  return request<ScanDetails>(`/scans/${init.scan_id}/upload/complete`, {
    method: "POST",
    body: JSON.stringify({ upload_id: init.upload_id, parts }),
  });
}

export async function uploadInput(
  scanId: string,
  kind: InputKind,
  file: File,
): Promise<ScanInput> {
  return request<ScanInput>(
    `/scans/${scanId}/inputs/${kind}?filename=${encodeURIComponent(file.name)}`,
    { method: "PUT", body: file },
  );
}

/** Follows the 307 redirect to the presigned URL and saves the blob. */
export async function downloadScan(scanId: string, filename: string): Promise<void> {
  const key = getApiKey();
  const res = await fetch(`${API_URL}/scans/${scanId}/download?format=las`, {
    headers: key ? { "X-API-Key": key } : {},
  });
  if (!res.ok) {
    let detail = res.statusText;
    try {
      detail = ((await res.json()) as { detail: string }).detail;
    } catch {
      /* ignore */
    }
    throw new ApiError(res.status, detail);
  }
  const blob = await res.blob();
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

export interface UserInfo {
  id: string;
  email: string;
}

export const getMe = (): Promise<UserInfo> => request("/users/me");

export const rotateApiKey = (): Promise<{ api_key: string }> =>
  request("/users/me/rotate-key", { method: "POST" });

export const renameProject = (projectId: string, name: string): Promise<Project> =>
  request(`/projects/${projectId}`, { method: "PUT", body: JSON.stringify({ name }) });

export const deleteProject = (projectId: string): Promise<void> =>
  request(`/projects/${projectId}`, { method: "DELETE" });

export const deleteScan = (scanId: string): Promise<void> =>
  request(`/scans/${scanId}`, { method: "DELETE" });

export const updateScanSettings = (
  scanId: string,
  settings: { photogrammetry_enabled: boolean },
): Promise<Scan> =>
  request(`/scans/${scanId}/settings`, {
    method: "PATCH",
    body: JSON.stringify(settings),
  });
