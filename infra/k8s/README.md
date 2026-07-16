# Kubernetes-манифесты (заготовка)

Появятся после стабилизации MVP. Планируемая раскладка:

- `backend` — Deployment + HPA, stateless
- `worker` — Deployment на GPU-нодах (`nvidia.com/gpu` в resources), масштабируется
  по глубине очереди (KEDA + Redis)
- Postgres/Redis/S3 — managed-сервисы облака, не в кластере
