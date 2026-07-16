import { FormEvent, useEffect, useState } from "react";
import { Link } from "react-router-dom";

import { createProject, listProjects } from "../api";
import type { Project } from "../types";

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

  return (
    <div className="page">
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
              </tr>
            </thead>
            <tbody>
              {projects.map((p) => (
                <tr key={p.id}>
                  <td>
                    <Link to={`/projects/${p.id}`}>{p.name}</Link>
                  </td>
                  <td className="muted">{new Date(p.created_at).toLocaleString()}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}
