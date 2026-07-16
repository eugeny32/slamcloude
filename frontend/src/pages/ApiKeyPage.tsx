import { FormEvent, useState } from "react";
import { useNavigate } from "react-router-dom";

import { listProjects, setApiKey } from "../api";

export default function ApiKeyPage() {
  const [key, setKey] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const navigate = useNavigate();

  async function submit(e: FormEvent) {
    e.preventDefault();
    setBusy(true);
    setError(null);
    setApiKey(key.trim());
    try {
      await listProjects(); // validates the key
      navigate("/");
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="page" style={{ maxWidth: 420 }}>
      <h1>Вход</h1>
      <form className="card" onSubmit={submit}>
        <p className="muted" style={{ marginBottom: 8 }}>
          API-ключ выдаётся администратором (`app.cli create-user`).
        </p>
        <input
          type="password"
          placeholder="sk_..."
          value={key}
          onChange={(e) => setKey(e.target.value)}
          autoFocus
        />
        {error && <p className="error">{error}</p>}
        <div style={{ marginTop: 12 }}>
          <button disabled={busy || !key.trim()}>Войти</button>
        </div>
      </form>
    </div>
  );
}
