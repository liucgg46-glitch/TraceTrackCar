# Next vision project

Keep the next project isolated in this directory.

Recommended layout:

```text
next_vision/
|- runtime/
|- diagnostics/
|- docs/
`- run_project.py
```

Use an explicit launcher and add only that project's `runtime` directory to `sys.path`. Do not replace files inside `projects/yuntai/`.

