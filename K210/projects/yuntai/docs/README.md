# Yuntai project archive on K210 SD

This project is paused as of 2026-07-19.

## Directories

- `runtime/`: current experimental runtime files.
- `backups/`: historical scripts, failed experiments, previous launchers and ZIP snapshots.
- `diagnostics/`: standalone camera and laser diagnostic scripts.
- `docs/`: project status and handoff notes.
- `run_yuntai.py`: explicit launcher; the SD root does not auto-start this project.

## Manual start

Run from a temporary CanMV IDE script:

```python
exec(open("/sd/projects/yuntai/run_yuntai.py").read())
```

The root `/sd/main.py` is intentionally neutral so another vision project can be developed without this project starting automatically.

