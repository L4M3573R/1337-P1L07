#!/usr/bin/env python3
"""What upstream StarPilot has that this branch doesn't, read live from GitHub.

  python3 scripts/upstream_status.py            # testing, stable and Dom vs HEAD
  python3 scripts/upstream_status.py --log      # also list testing's new commits

Asks GitHub for each branch's current tip (`git ls-remote`), fetches those branches by explicit refspec, and only then
counts commits. A plain `git fetch upstream` is not trusted: until 2026-11-23 `remote.upstream.fetch` covered only
`Dom`, so `upstream/testing` sat frozen at 11-19 and "no updates" was reported while 202 commits had landed. The
fetch refspec is also repaired to all branches, and a local ref that still differs from GitHub after the fetch fails.
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REMOTE = 'upstream'
BRANCHES = ('testing', 'stable', 'Dom')   # testing is what this fork follows; stable replaced StarPilot on 11-17
WILDCARD = f'+refs/heads/*:refs/remotes/{REMOTE}/*'


def git(*args: str, network: bool = False) -> str:
  # inside the Claude Desktop distrobox, network git runs on the host (credentials, as for push)
  prefix = ['distrobox-host-exec'] if network and os.environ.get('CONTAINER_ID') else []
  return subprocess.run(prefix + ['git', '-C', str(ROOT), *args], capture_output=True, text=True, check=True).stdout.strip()


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('--log', action='store_true', help="list testing's commits missing from HEAD")
  args = parser.parse_args()

  if git('config', '--get-all', f'remote.{REMOTE}.fetch').splitlines() != [WILDCARD]:
    git('config', '--replace-all', f'remote.{REMOTE}.fetch', WILDCARD)
    print(f'repaired remote.{REMOTE}.fetch -> {WILDCARD}')

  live = {}
  for line in git('ls-remote', '--heads', REMOTE, *BRANCHES, network=True).splitlines():
    sha, ref = line.split()
    live[ref.removeprefix('refs/heads/')] = sha
  missing = [b for b in BRANCHES if b not in live]
  if missing:
    print(f'not on GitHub any more: {", ".join(missing)}')
  git('fetch', '--quiet', REMOTE, *(f'+refs/heads/{b}:refs/remotes/{REMOTE}/{b}' for b in live), network=True)

  failed = False
  for branch, sha in live.items():
    local = git('rev-parse', f'{REMOTE}/{branch}')
    if local != sha:
      print(f'{branch}: local ref {local[:10]} still differs from GitHub {sha[:10]}')
      failed = True
      continue
    tip = git('log', '-1', '--format=%h %ci %s', sha)
    new = int(git('rev-list', '--count', f'HEAD..{sha}'))
    print(f'{branch:8s} {new:5d} commits not in HEAD | tip {tip}')
  if args.log and 'testing' in live:
    log = git('log', '--no-merges', '--reverse', '--format=  %ad %s', '--date=format:%m-%d', f'HEAD..{live["testing"]}')
    print(log or '  (none)')
  return 1 if failed else 0


if __name__ == '__main__':
  sys.exit(main())
