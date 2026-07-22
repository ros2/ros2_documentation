#!/usr/bin/env bash
# Apply the Pagefind migration kit onto a clean ros2_documentation checkout.
# Run from the documentation repo root (parent of this kit), or pass REPO_ROOT.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(cd "$SCRIPT_DIR/.." && pwd)}"
FILES="$SCRIPT_DIR/files"
PATCHES="$SCRIPT_DIR/patches"

cd "$REPO_ROOT"

if [[ ! -f conf.py || ! -f Makefile ]]; then
  echo "error: $REPO_ROOT does not look like ros2_documentation (missing conf.py/Makefile)" >&2
  exit 1
fi

echo "==> Copying new Pagefind files into $REPO_ROOT"
cp -v "$FILES/pagefind.yml" "$REPO_ROOT/pagefind.yml"
mkdir -p "$REPO_ROOT/plugins" "$REPO_ROOT/source/_static" "$REPO_ROOT/source/_templates" "$REPO_ROOT/test"
cp -v "$FILES/plugins/"*.py "$REPO_ROOT/plugins/"
cp -v "$FILES/source/_static/pagefind-docsearch.css" "$REPO_ROOT/source/_static/"
cp -v "$FILES/source/_templates/"*.html "$REPO_ROOT/source/_templates/"
cp -v "$FILES/test/"*.py "$REPO_ROOT/test/"

echo "==> Applying patches"
patch -p1 < "$PATCHES/Makefile.patch"
patch -p1 < "$PATCHES/conf.py.patch"
patch -p1 < "$PATCHES/requirements.txt.patch"
patch -p1 < "$PATCHES/test.yml.patch"
patch -p1 < "$PATCHES/README.md.patch"

echo
echo "Pagefind files applied."
echo "Next:"
echo "  pip install -r requirements.txt -c constraints.txt"
echo "  make html-search"
echo "  # then open http://localhost:8000 after: python -m http.server 8000 --directory build/html"
echo
echo "See pagefind-migration-kit/IMPLEMENTATION.md for verification and meta tagging."
