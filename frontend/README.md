# frontend

React + TypeScript (Vite). Стриминговый 3D-просмотр облаков точек: COPC
читается HTTP range-запросами (библиотека `copc`), узлы октодерева
загружаются от грубых уровней к детальным в пределах бюджета точек —
файл целиком не скачивается. Рендер — three.js.

Страницы: вход по API-ключу → проекты → сканы (chunked-загрузка LAS/LAZ из
браузера, PPK-панель: прикрепление траектории/RINEX и пересчёт) → 3D-просмотр
с прогрессом пайплайна и экспортом LAZ.

## Запуск

```powershell
npm install
npm run dev -- --host 127.0.0.1   # http://127.0.0.1:5173, API: VITE_API_URL (по умолчанию http://localhost:8000)
npm run build                      # tsc + vite build → dist/
```

В docker-compose собирается в nginx-образ и доступен на http://localhost:3000.

## Известная проблема среды (Windows + OneDrive)

Если репозиторий лежит в синхронизируемой OneDrive папке (`Documents`),
`vite build` падает с фантомным ENOENT, а dev-сервер зависает (вотчер файлов).
Обход: `npx vite build --outDir "$env:LOCALAPPDATA\Temp\slamcloude-dist" --emptyOutDir`
или перенести репозиторий вне OneDrive (например, `C:\dev`). В Docker проблемы нет.
