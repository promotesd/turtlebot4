"""Fail when a local Markdown link points to a missing repository path."""

from __future__ import annotations

from pathlib import Path
import re
import sys
from urllib.parse import unquote

LINK = re.compile(r'(?<!!)\[[^]]*]\(([^)]+)\)')


def markdown_files(root: Path) -> list[Path]:
    """Return tracked-source Markdown candidates without build output."""
    ignored = {'.git', 'build', 'install', 'log'}
    return sorted(
        path
        for path in root.rglob('*.md')
        if not any(part in ignored for part in path.parts)
    )


def missing_links(root: Path) -> list[str]:
    """Return readable diagnostics for missing local links."""
    missing: list[str] = []
    for document in markdown_files(root):
        for match in LINK.finditer(document.read_text(encoding='utf-8')):
            target = match.group(1).strip().split(maxsplit=1)[0].strip('<>')
            if target.startswith(('http://', 'https://', 'mailto:', '#')):
                continue
            path_text = unquote(target.split('#', 1)[0])
            if not path_text:
                continue
            destination = (document.parent / path_text).resolve()
            if not destination.exists():
                missing.append(f'{document}:{match.start(1)}: missing {target}')
    return missing


def main() -> int:
    """Check the current repository and print all failures."""
    failures = missing_links(Path.cwd())
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    print('All local Markdown links resolve.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
