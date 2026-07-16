export interface Project {
  id: string;
  name: string;
  created_at: string;
}

export type ScanStatus = "uploading" | "uploaded" | "processing" | "completed" | "failed";
export type JobStatus = "pending" | "running" | "completed" | "failed";

export interface Job {
  id: string;
  pipeline_step: string;
  status: JobStatus;
  started_at: string | null;
  finished_at: string | null;
  error_message: string | null;
}

export interface Asset {
  id: string;
  asset_type: "las" | "copc" | "mesh" | "splat";
  storage_path: string;
  file_size: number;
  version: number;
  created_at: string;
}

export interface Scan {
  id: string;
  status: ScanStatus;
  captured_at: string | null;
  rtk_fixed: boolean;
  size_bytes: number | null;
  created_at: string;
}

export interface ScanDetails extends Scan {
  project_id: string;
  raw_file_path: string | null;
  checksum_sha256: string | null;
  num_points: number | null;
  source_format: string | null;
  crs_epsg: number | null;
  bbox: [number, number, number, number] | null;
  jobs: Job[];
  assets: Asset[];
}

export interface UploadInit {
  scan_id: string;
  upload_id: string;
  part_size: number;
  num_parts: number;
}

export interface Preview {
  scan_id: string;
  status: ScanStatus;
  num_points: number | null;
  bbox: [number, number, number, number] | null;
  crs_epsg: number | null;
  copc_url: string | null;
}

export type InputKind = "trajectory" | "rover_obs" | "base_rinex" | "nav";

export interface ScanInput {
  kind: InputKind;
  storage_path: string;
  file_size: number;
  uploaded_at: string;
}
