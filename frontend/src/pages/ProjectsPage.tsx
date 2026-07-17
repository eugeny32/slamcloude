import { FormEvent, useEffect, useRef, useState } from "react";
import { Link } from "react-router-dom";

import {
  createProject,
  deleteProject,
  getApiKey,
  getMe,
  listProjects,
  renameProject,
  rotateApiKey,
  setApiKey,
} from "../api";
import type { Project } from "../types";

function ApiKeySection() {
  const [email, setEmail] = useState<string | null>(null);
  const [copied, setCopied] = useState(false);
  const [rotating, setRotating] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const rawKey = getApiKey();

  useEffect(() => {
    getMe()
      .then((u) => setEmail(u.email))
      .catch(() => setEmail(null));
  }, []);

  async function copy() {
    if (!rawKey) return;
    try {
      await navigator.clipboard.writeText(rawKey);
    } catch {
      const ta = document.createElement("textarea");
      ta.value = rawKey;
      ta.style.position = "fixed";
      ta.style.opacity = "0";
      document.body.appendChild(ta);
      ta.focus();
      ta.select();
      document.execCommand("copy");
      document.body.removeChild(ta);
    }
    setCopied(true);
    setTimeout(() => setCopied(false), 1500);
  }

  async function rotate() {
    if (!confirm("Сгенерировать новый API ключ? Старый ключ перестанет работать.")) return;
    setRotating(true);
    setMsg(null);
    try {
      const { api_key } = await rotateApiKey();
      setApiKey(api_key);
      setMsg("Новый ключ сохранён.");
    } catch (err) {
      setMsg(err instanceof Error ? err.message : String(err));
    } finally {
      setRotating(false);
    }
  }

  if (!rawKey) return null;

  const displayKey = rawKey.slice(0, 8) + "••••••••••••••••";

  return (
    <div className="card" style={{ marginBottom: 16 }}>
      <div className="row" style={{ alignItems: "center", gap: 8, flexWrap: "wrap" }}>
        {email && (
          <span className="muted" style={{ marginRight: 8 }}>
            {email}
          </span>
        )}
        <span style={{ fontFamily: "monospace", fontSize: 13 }}>{displayKey}</span>
        <button className="secondary" onClick={() => void copy()} style={{ padding: "2px 10px" }}>
          {copied ? "Скопировано!" : "Копировать ключ"}
        </button>
        <button
          className="secondary"
          onClick={() => void rotate()}
          disabled={rotating}
          style={{ padding: "2px 10px" }}
        >
          {rotating ? "…" : "Сменить ключ"}
        </button>
        {msg && <span className="muted">{msg}</span>}
      </div>
    </div>
  );
}

function ProjectRow({
  project,
  onRename,
  onDelete,
}: {
  project: Project;
  onRename: (id: string, name: string) => Promise<void>;
  onDelete: (id: string) => Promise<void>;
}) {
  const [editing, setEditing] = useState(false);
  const [editName, setEditName] = useState(project.name);
  const [busy, setBusy] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);

  function startEdit() {
    setEditName(project.name);
    setEditing(true);
    setTimeout(() => inputRef.current?.focus(), 0);
  }

  async function saveEdit() {
    const trimmed = editName.trim();
    if (!trimmed || trimmed === project.name) {
      setEditing(false);
      return;
    }
    setBusy(true);
    try {
      await onRename(project.id, trimmed);
    } finally {
      setBusy(false);
      setEditing(false);
    }
  }

  async function handleDelete() {
    if (!confirm(`Удалить проект «${project.name}»? Все сканы и данные будут удалены навсегда.`))
      return;
    setBusy(true);
    try {
      await onDelete(project.id);
    } finally {
      setBusy(false);
    }
  }

  return (
    <tr>
      <td>
        {editing ? (
          <input
            ref={inputRef}
            value={editName}
            onChange={(e) => setEditName(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") void saveEdit();
              if (e.key === "Escape") setEditing(false);
            }}
            onBlur={() => void saveEdit()}
            style={{ width: "100%", boxSizing: "border-box" }}
            disabled={busy}
          />
        ) : (
          <Link to={`/projects/${project.id}`}>{project.name}</Link>
        )}
      </td>
      <td className="muted">{new Date(project.created_at).toLocaleString()}</td>
      <td style={{ whiteSpace: "nowrap" }}>
        {!editing && (
          <button
            className="secondary"
            onClick={startEdit}
            disabled={busy}
            title="Переименовать"
            style={{ marginRight: 4, padding: "2px 8px" }}
          >
            ✎
          </button>
        )}
        <button
          className="secondary"
          onClick={() => void handleDelete()}
          disabled={busy}
          title="Удалить"
          style={{ padding: "2px 8px", color: "#c00" }}
        >
          ✕
        </button>
      </td>
    </tr>
  );
}

export default function ProjectsPage() {
  const [projects, setProjects] = useState<Project[] | null>(null);
  const [name, setName] = useState("");
  const [error, setError] = useState<string | null>(null);

  async function refresh() {
    try {
      setProjects(await listProjects());
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  useEffect(() => {
    void refresh();
  }, []);

  async function submit(e: FormEvent) {
    e.preventDefault();
    try {
      await createProject(name.trim());
      setName("");
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  async function handleRename(id: string, newName: string) {
    try {
      await renameProject(id, newName);
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  async function handleDelete(id: string) {
    try {
      await deleteProject(id);
      await refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  return (
    <div className="page">
      <ApiKeySection />
      <h1>Проекты</h1>
      {error && <p className="error">{error}</p>}
      <form className="card row" onSubmit={submit}>
        <input
          type="text"
          placeholder="Название нового проекта"
          value={name}
          onChange={(e) => setName(e.target.value)}
        />
        <button disabled={!name.trim()}>Создать</button>
      </form>
      <div className="card">
        {projects === null ? (
          <p className="muted">Загрузка…</p>
        ) : projects.length === 0 ? (
          <p className="muted">Проектов пока нет — создайте первый.</p>
        ) : (
          <table>
            <thead>
              <tr>
                <th>Название</th>
                <th>Создан</th>
                <th></th>
              </tr>
            </thead>
            <tbody>
              {projects.map((p) => (
                <ProjectRow
                  key={p.id}
                  project={p}
                  onRename={handleRename}
                  onDelete={handleDelete}
                />
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}
