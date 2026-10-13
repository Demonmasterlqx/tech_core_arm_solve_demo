#!/usr/bin/env python3
"""Build a verified delivery ZIP (Python 3.10+, Git; optional local document tools)."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tempfile
from urllib.parse import quote, unquote, urlsplit
import zipfile


MUJOCO = Path('.deps/mujoco-3.3.7')
CSS = """<style>
body { max-width: 56rem; margin: 2rem auto; padding: 0 1.3rem;
font-family: 'Noto Serif CJK SC', serif; line-height: 1.8; color: #18202a; }
h1,h2,h3 { line-height: 1.4; } pre { overflow-x: auto; padding: 1rem;
background: #f3f5f7; } code { font-family: monospace; } table {
border-collapse: collapse; width: 100%; } th,td { border: 1px solid #ccd2d9;
padding: .5rem; vertical-align: top; } img { max-width: 100%; }
math[display='block'] { overflow-x: auto; margin: 1.2rem 0; }
a { color: #175b9b; } @media print { body { max-width: none; margin: 0; } }
</style>"""
# Covers ordinary inline Markdown links/images and reference definitions. Code
# fences are left untouched, so command examples are never rewritten as links.
INLINE_LINK = re.compile(r'(!?\[[^\]\n]*\]\()(<[^>\n]+>|[^\s)]+)([^)\n]*\))')
REFERENCE_LINK = re.compile(r'^(\s{0,3}\[[^\]\n]+\]:\s*)(<[^>\n]+>|\S+)(.*)$')


def run(argv: list[str], *, cwd: Path | None = None, timeout: int = 180) -> str:
    result = subprocess.run(argv, cwd=cwd, check=False, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            timeout=timeout)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {argv!r}\n"
                           f"{result.stdout[-6000:]}\n{result.stderr[-6000:]}")
    return result.stdout.strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def check_symlinks(root: Path) -> None:
    """Require every link to resolve inside its tree; reject special files."""
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            path = Path(directory) / name
            if path.is_symlink():
                target = path.resolve(strict=True)
                if not within(target, root.resolve()):
                    raise RuntimeError(f'Symlink escapes packaged tree: {path}')
            elif not (path.is_dir() or path.is_file()):
                raise RuntimeError(f'Unsupported special file: {path}')


def ensure_renderers() -> dict[str, str]:
    for executable in ('pandoc', 'xelatex', 'fc-match', 'kpsewhich'):
        if not shutil.which(executable):
            raise RuntimeError(f'Required local renderer unavailable: {executable}; '
                               'install it first, or use --no-render for diagnosis')
    family = run(['fc-match', '-f', '%{family}', 'Noto Serif CJK SC'])
    if 'Noto Serif CJK SC' not in family:
        raise RuntimeError(f'Required CJK font unavailable; fc-match returned {family!r}')
    mono = run(['fc-match', '-f', '%{family}', 'Noto Sans Mono CJK SC'])
    if 'Noto Sans Mono CJK SC' not in mono:
        raise RuntimeError(f'Required CJK monospace font unavailable: {mono!r}')
    latin_mono = run(['fc-match', '-f', '%{family}', 'DejaVu Sans Mono'])
    if 'DejaVu Sans Mono' not in latin_mono:
        raise RuntimeError(f'Required Latin monospace font unavailable: {latin_mono!r}')
    for name in ('lmodern.sty', 'pzdr.tfm', 'xeCJK.sty', 'fvextra.sty', 'xurl.sty', 'etoolbox.sty'):
        package = run(['kpsewhich', name])
        if not package or not Path(package).is_file():
            raise RuntimeError(f'Required local XeLaTeX package unavailable: {name}')
    return {'font': family, 'monospace_font': latin_mono, 'cjk_monospace_font': mono, 'pandoc': run(['pandoc', '--version']).splitlines()[0],
            'xelatex': run(['xelatex', '--version']).splitlines()[0]}


def rewrite_links(text: str, source: Path, handbook: Path, repository: Path) -> str:
    def target(value: str) -> str:
        bracketed = value.startswith('<') and value.endswith('>')
        raw = value[1:-1] if bracketed else value
        parsed = urlsplit(raw)
        if parsed.scheme or parsed.netloc or not parsed.path:
            return value
        resolved = (source.parent / unquote(parsed.path)).resolve()
        if not within(resolved, repository):
            raise RuntimeError(f'Document link escapes repository: {source}: {raw}')
        if not resolved.exists():
            raise RuntimeError(f'Broken document link: {source}: {raw}')
        if within(resolved, handbook):
            # Handbook assets are copied with the documents, preserving layout.
            new_path = os.path.relpath(resolved, source.parent)
        else:
            new_path = '../git_repository/' + resolved.relative_to(repository).as_posix()
        rewritten = quote(new_path, safe='/._~-')
        if parsed.query:
            rewritten += '?' + parsed.query
        if parsed.fragment:
            rewritten += '#' + parsed.fragment
        return '<' + rewritten + '>' if bracketed else rewritten

    output: list[str] = []
    fence: str | None = None
    for line in text.splitlines(keepends=True):
        marker = re.match(r'^\s{0,3}(`{3,}|~{3,})', line)
        if marker:
            kind = marker.group(1)[0]
            if fence is None:
                fence = kind
            elif fence == kind:
                fence = None
            output.append(line)
            continue
        if fence:
            output.append(line)
            continue
        reference = REFERENCE_LINK.match(line.rstrip('\n'))
        if reference:
            ending = '\n' if line.endswith('\n') else ''
            line = reference.group(1) + target(reference.group(2)) + reference.group(3) + ending
        else:
            line = INLINE_LINK.sub(lambda match: match.group(1) + target(match.group(2))
                                   + match.group(3), line)
        output.append(line)
    return ''.join(output)


class OfflineHTML(HTMLParser):
    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if value and ((name == 'src' and tag != 'a') or
                          (tag == 'link' and name == 'href')):
                if not value.startswith(('data:', '#')):
                    raise RuntimeError(f'HTML contains a non-embedded resource: {value}')


def render_documents(documents: Path, mains: list[str], scratch: Path) -> None:
    header = scratch / 'style.html'
    header.write_text(CSS, encoding='utf-8')
    tex_header = scratch / 'formatting.tex'
    tex_header.write_text(r"""\usepackage{fvextra}
\usepackage{xurl}
\usepackage{etoolbox}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{commandchars=\\\{\},breaklines=true,breakanywhere=true,fontsize=\small}
\DefineVerbatimEnvironment{verbatim}{Verbatim}{breaklines=true,breakanywhere=true,fontsize=\small}
\AtBeginEnvironment{longtable}{\small}
\setlength{\emergencystretch}{3em}
""", encoding='utf-8')
    # Insert legal wrap positions in inline identifiers/paths, including tables.
    # The document sources remain untouched; this only affects the PDF view.
    code_filter = scratch / 'wrap-inline-code.lua'
    code_filter.write_text(r"""function Code(element)
  local escaped = {['\\']='\\textbackslash{}', ['{']='\\{', ['}']='\\}',
    ['$']='\\$', ['&']='\\&', ['#']='\\#', ['%']='\\%', ['_']='\\_',
    ['^']='\\textasciicircum{}', ['~']='\\textasciitilde{}'}
  local parts = {}
  local run = 0
  for char in element.text:gmatch('.') do
    table.insert(parts, escaped[char] or char)
    if char:match('[A-Za-z0-9]') then run = run + 1 else run = 0 end
    if char:match('[/._:-]') or run >= 10 then
      table.insert(parts, '\\allowbreak{}')
      run = 0
    end
  end
  return pandoc.RawInline('latex', '\\texttt{' .. table.concat(parts) .. '}')
end
""", encoding='utf-8')
    # Pandoc creates its TeX intermediates under TMPDIR; neither repository nor
    # documentation is used as a compilation scratch directory.
    for name in mains:
        source = documents / name
        common = ['pandoc', str(source), '--standalone', '--from=markdown+tex_math_single_backslash',
                  '--metadata=lang:zh-CN', '--resource-path=' + str(documents)]
        run(common + ['--to=html5', '--mathml', '--embed-resources',
                      '--include-in-header=' + str(header),
                      '--output=' + str(source.with_suffix('.html'))], cwd=documents)
        OfflineHTML().feed(source.with_suffix('.html').read_text(encoding='utf-8'))
        run(common + ['--pdf-engine=xelatex', '--include-in-header=' + str(tex_header),
                      '--lua-filter=' + str(code_filter),
                      '-V', 'CJKmainfont=Noto Serif CJK SC',
                      '-V', 'CJKmonofont=Noto Sans Mono CJK SC',
                      '-V', 'monofont=DejaVu Sans Mono',
                      '-V', 'mainfont=Noto Serif CJK SC', '-V', 'geometry:margin=22mm',
                      '-V', 'fontsize=11pt', '--output=' + str(source.with_suffix('.pdf'))],
            cwd=scratch, timeout=300)
        if not source.with_suffix('.pdf').read_bytes().startswith(b'%PDF-'):
            raise RuntimeError(f'Rendered PDF is invalid: {name}')


def iter_files(root: Path) -> list[tuple[Path, str]]:
    """Return ZIP file entries, dereferencing in-tree symlinks, including dirs."""
    result: list[tuple[Path, str]] = []

    def visit(actual: Path, name: str, ancestors: frozenset[Path]) -> None:
        resolved = actual.resolve(strict=True)
        if not within(resolved, root.resolve()):
            raise RuntimeError(f'Archive entry resolves outside package: {actual}')
        if resolved.is_dir():
            if resolved in ancestors:
                raise RuntimeError(f'Symlink directory cycle: {actual}')
            result.append((resolved, name + '/'))
            for child in sorted(resolved.iterdir()):
                visit(child, name + '/' + child.name, ancestors | {resolved})
        elif resolved.is_file():
            result.append((resolved, name))
        else:
            raise RuntimeError(f'Unsupported archive entry: {actual}')

    for child in sorted(root.iterdir()):
        visit(child, child.name, frozenset())
    return result


def write_archive(root: Path, output: Path) -> tuple[int, dict[str, str]]:
    entries = iter_files(root)
    hashes: dict[str, str] = {}
    for path, name in entries:
        if not name.endswith('/') and name != 'documents/manifest.json':
            hashes[name] = sha256(path)
    manifest_path = root / 'documents/manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['files_sha256'] = hashes
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6,
                         allowZip64=True) as archive:
        for path, name in entries:
            info = zipfile.ZipInfo.from_file(path, name)
            info.create_system = 3
            mode = stat.S_IMODE(path.stat().st_mode)
            info.external_attr = ((stat.S_IFDIR if name.endswith('/') else stat.S_IFREG) | mode) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            if name.endswith('/'):
                info.external_attr |= 0x10
                archive.writestr(info, b'')
            else:
                with path.open('rb') as source, archive.open(info, 'w', force_zip64=True) as destination:
                    shutil.copyfileobj(source, destination, length=1024 * 1024)
    with zipfile.ZipFile(output) as archive:
        damaged = archive.testzip()
        if damaged:
            raise RuntimeError(f'ZIP CRC verification failed: {damaged}')
        members = {item.filename for item in archive.infolist() if not item.is_dir()}
        if members != set(hashes) | {'documents/manifest.json'}:
            raise RuntimeError('ZIP member list does not match the manifest')
        if {name.split('/')[0] for name in archive.namelist()} != {'git_repository', 'documents'}:
            raise RuntimeError('ZIP top-level folders do not match the delivery contract')
        for name, expected in hashes.items():
            digest = hashlib.sha256()
            with archive.open(name) as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            if digest.hexdigest() != expected:
                raise RuntimeError(f'ZIP manifest verification failed: {name}')
    return len(hashes), hashes


def clean_git(repository: Path) -> bool:
    return not run(['git', 'status', '--porcelain=v1', '--untracked-files=all'], cwd=repository)


def build(args: argparse.Namespace) -> dict[str, object]:
    repository = Path(run(['git', 'rev-parse', '--show-toplevel'])).resolve()
    output = Path(args.output).expanduser().absolute()
    if output.suffix.lower() != '.zip':
        raise RuntimeError('--output must end in .zip')
    side_hash = output.with_suffix(output.suffix + '.sha256')
    side_report = output.with_suffix(output.suffix + '.report.json')
    for path in (output, side_hash, side_report):
        if os.path.lexists(path):
            raise RuntimeError(f'Refusing to replace existing output: {path}')
    if not output.parent.is_dir():
        raise RuntimeError(f'Output directory does not exist: {output.parent}')
    if not clean_git(repository):
        raise RuntimeError('Source Git worktree must be clean, including untracked non-ignored files')
    staged = run(['git', 'ls-files', '--stage', '-z'], cwd=repository)
    if any(entry.startswith('120000 ') for entry in staged.split('\0')):
        raise RuntimeError('Tracked symlinks are not supported: dereferencing them would '
                           'make the packaged Git worktree dirty')
    head = run(['git', 'rev-parse', 'HEAD'], cwd=repository)
    branch = run(['git', 'branch', '--show-current'], cwd=repository) or '(detached HEAD)'
    handbook = (repository / args.documents_dir).resolve()
    if not within(handbook, repository) or not handbook.is_dir():
        raise RuntimeError('--documents-dir must be an existing directory within the repository')
    relative_handbook = handbook.relative_to(repository)
    renderers = {} if args.no_render else ensure_renderers()
    created: list[Path] = []
    try:
        with tempfile.TemporaryDirectory(prefix='arm-delivery-') as temporary:
            scratch = Path(temporary)
            package = scratch / 'package'
            package.mkdir()
            clone = package / 'git_repository'
            run(['git', 'clone', '--no-local', str(repository), str(clone)])
            if run(['git', 'rev-parse', 'HEAD'], cwd=clone) != head:
                raise RuntimeError('Source HEAD changed during cloning')
            # Removing origin also removes its tracking refs. Preserve every
            # advertised source branch as a local branch before that removal.
            tracking = run(['git', 'for-each-ref', '--format=%(refname) %(objectname)',
                            'refs/remotes/origin/'], cwd=clone)
            for entry in tracking.splitlines():
                ref, commit = entry.split()
                if ref == 'refs/remotes/origin/HEAD':
                    continue
                local = 'refs/heads/' + ref.removeprefix('refs/remotes/origin/')
                run(['git', 'update-ref', local, commit], cwd=clone)
            run(['git', 'remote', 'remove', 'origin'], cwd=clone)
            if (clone / '.git/objects/info/alternates').exists():
                raise RuntimeError('Clone uses external Git object alternates')
            run(['git', 'fsck', '--full'], cwd=clone)
            if not clean_git(clone) or run(['git', 'remote'], cwd=clone):
                raise RuntimeError('Independent clone failed clean-state or remote verification')
            source_docs = clone / relative_handbook
            if not (source_docs / 'README.md').is_file():
                raise RuntimeError('Documentation must include README.md')
            mains = sorted(path.name for path in source_docs.glob('*.md') if path.name != 'README.md')
            if len(mains) != 3:
                raise RuntimeError('Documentation must contain exactly three main Markdown files plus README.md')
            check_symlinks(source_docs)
            documents = package / 'documents'
            shutil.copytree(source_docs, documents, symlinks=False)
            for name in ['README.md', *mains]:
                original = source_docs / name
                rewritten = rewrite_links(original.read_text(encoding='utf-8'), original,
                                          source_docs.resolve(), clone.resolve())
                (documents / name).write_text(rewritten, encoding='utf-8')
            if args.include_mujoco:
                dependency = repository / MUJOCO
                if not (dependency / 'THIRD_PARTY_NOTICES').is_file():
                    raise RuntimeError('Complete local MuJoCo dependency with THIRD_PARTY_NOTICES is required')
                check_symlinks(dependency)
                shutil.copytree(dependency, clone / MUJOCO, symlinks=False)
            if not args.no_render:
                previous_tmp = os.environ.get('TMPDIR')
                os.environ['TMPDIR'] = str(scratch)
                try:
                    render_documents(documents, mains, scratch)
                finally:
                    if previous_tmp is None:
                        os.environ.pop('TMPDIR', None)
                    else:
                        os.environ['TMPDIR'] = previous_tmp
            if not clean_git(clone):
                raise RuntimeError('Bundled repository is not clean after packaging dependencies')
            check_symlinks(clone)
            manifest = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                        'source_git_head': head, 'source_git_branch': branch,
                        'options': {'documents_dir': relative_handbook.as_posix(),
                                    'include_mujoco': args.include_mujoco, 'render': not args.no_render},
                        'renderers': renderers,
                        'git': {'independent_clone': True, 'clone_mode': '--no-local',
                                'object_alternates': False, 'remotes': [], 'fsck_passed': True,
                                'source_clean': True, 'packaged_clean': True},
                        'main_documents': mains,
                        'manifest_scope': 'All packaged regular-file contents except this manifest; '
                                          'symlinks are dereferenced within their containing tree.'}
            (documents / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)
                                                    + '\n', encoding='utf-8')
            staged_zip = scratch / 'delivery.zip'
            count, _ = write_archive(package, staged_zip)
            if not clean_git(repository) or run(['git', 'rev-parse', 'HEAD'], cwd=repository) != head:
                raise RuntimeError('Source worktree changed while preparing the delivery')
            digest = sha256(staged_zip)
            report: dict[str, object] = {'output': str(output), 'sha256': digest,
                'bytes': staged_zip.stat().st_size, 'source_git_head': head,
                'source_git_branch': branch, 'files_verified': count,
                'zip_crc_passed': True, 'manifest_verified': True,
                'git_fsck_passed': True, 'independent_repository': True,
                'rendered': not args.no_render, 'include_mujoco': args.include_mujoco,
                'top_level_directories': ['git_repository', 'documents']}
            # Exclusive creation prevents accidental overwrite, including a race
            # after the early existence check. Roll back only files we created.
            with output.open('xb') as destination:
                created.append(output)
                with staged_zip.open('rb') as source:
                    shutil.copyfileobj(source, destination, length=1024 * 1024)
            if sha256(output) != digest:
                raise RuntimeError('Published ZIP checksum differs from verified staged ZIP')
            with side_hash.open('x', encoding='utf-8') as stream:
                created.append(side_hash)
                stream.write(f'{digest}  {output.name}\n')
            with side_report.open('x', encoding='utf-8') as stream:
                created.append(side_report)
                json.dump(report, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
            return report
    except BaseException:
        for path in reversed(created):
            path.unlink(missing_ok=True)
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, epilog=
        'Run from a clean Git worktree. Rendering requires pandoc, XeLaTeX, Fontconfig, '
        'Noto Serif CJK SC, Noto Sans Mono CJK SC, DejaVu Sans Mono, xeCJK, fvextra, '
        'xurl, etoolbox, lmodern and pzdr.tfm. '
        'No tools or fonts are downloaded. --no-render is diagnostic only.')
    parser.add_argument('--output', required=True, help='New output .zip path; parent must exist')
    parser.add_argument('--documents-dir', default='docs/handbook', help='Tracked handbook directory')
    parser.add_argument('--include-mujoco', action='store_true', help='Bundle local MuJoCo 3.3.7 and notices')
    parser.add_argument('--no-render', action='store_true', help='Diagnostic source-only documentation package')
    args = parser.parse_args()
    try:
        report = build(args)
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f'Packaging failed: {error}\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
