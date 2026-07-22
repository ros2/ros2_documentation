# Pagefind migration kit

Portable package to add **Pagefind search** to a clean
`ros2/ros2_documentation` fork (or this repo’s `feature/pagefind-search` branch).

| Doc | Purpose |
|-----|---------|
| [IMPLEMENTATION.md](IMPLEMENTATION.md) | Step-by-step apply, verify, tag pages |
| [FILE_MANIFEST.md](FILE_MANIFEST.md) | Exact file list + what was excluded |
| [apply.sh](apply.sh) | One-shot copy + patch |
| `files/` | New files to copy |
| `patches/` | Pagefind-only diffs vs upstream `rolling` |

```bash
# From a clean ros2_documentation checkout that contains this kit:
./pagefind-migration-kit/apply.sh
make html-search
```

In this repo, the same stack is already committed on branch **`feature/pagefind-search`**
(based on `upstream/rolling`, Pagefind-only).
