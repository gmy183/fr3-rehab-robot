#!/usr/bin/env bash
set -Eeuo pipefail

# ==========================================
# FR3 Rehab Robot - GitHub Automatic Backup
# Repository: gmy183/fr3-rehab-robot
# Branch: main
# ==========================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$SCRIPT_DIR/.." rev-parse --show-toplevel)"

cd "$ROOT"

EXPECTED_REMOTE="git@github.com:gmy183/fr3-rehab-robot.git"
BRANCH="main"
MAX_FILE_SIZE=$((25 * 1024 * 1024))

echo
echo "=========================================="
echo " FR3 REHAB ROBOT - GITHUB BACKUP"
echo "=========================================="

# 1. Check branch
CURRENT_BRANCH=$(git branch --show-current)

if [[ "$CURRENT_BRANCH" != "$BRANCH" ]]; then
    echo "[STOP] Expected branch: $BRANCH"
    exit 1
fi

# 2. Check remote
REMOTE_URL=$(git remote get-url origin)

if [[ "$REMOTE_URL" != "$EXPECTED_REMOTE" ]]; then
    echo "[STOP] Unexpected GitHub remote"
    exit 1
fi

echo "[PASS] Repository and branch verified"

# 3. Fetch remote
echo
echo "[1/6] Checking GitHub..."

git fetch --quiet origin main

REMOTE_BEFORE=$(git rev-parse origin/main)
LOCAL_BEFORE=$(git rev-parse HEAD)

# Never overwrite remote commits that are missing locally
if ! git merge-base --is-ancestor "$REMOTE_BEFORE" "$LOCAL_BEFORE"; then
    echo "[STOP] Remote contains commits not present locally."
    echo "[ACTION] Review and synchronize branches first."
    exit 1
fi

echo "[PASS] Remote history verified"

# 4. Stage project changes
echo
echo "[2/6] Collecting project changes..."

git add -A

# 5. Security and file-size checks
echo
echo "[3/6] Running safety checks..."

python3 - "$MAX_FILE_SIZE" <<'PY'
import os
import re
import subprocess
import sys

max_size = int(sys.argv[1])

def git_bytes(*args):
    return subprocess.check_output(["git", *args])

# Check all indexed paths for prohibited files
paths = git_bytes("ls-files", "-z").split(b"\0")

blocked_pattern = re.compile(
    rb"(^|/)(hardware_profile\.env|\.env|"
    rb"ENVIRONMENT_SNAPSHOT\.txt|id_rsa|id_ed25519|"
    rb"[^/]+\.(pem|key|p12|pfx))$",
    re.I
)

blocked = [
    os.fsdecode(p)
    for p in paths if p and blocked_pattern.search(p)
]

if blocked:
    print("[STOP] Prohibited files detected:")
    for p in blocked:
        print("  ", p)
    sys.exit(1)

# Check staged file sizes
staged = git_bytes(
    "diff", "--cached", "--name-only", "-z"
).split(b"\0")

large = []

for raw in staged:
    if not raw:
        continue

    path = os.fsdecode(raw)

    try:
        size = int(subprocess.check_output(
            ["git", "cat-file", "-s", ":" + path],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip())
    except subprocess.CalledProcessError:
        continue

    if size > max_size:
        large.append((path, size))

if large:
    print("[STOP] Files exceed 25 MB:")
    for path, size in large:
        print(f"  {path}: {size / 1024 / 1024:.2f} MB")
    sys.exit(1)

# Scan staged text changes for high-confidence credentials
diff = git_bytes(
    "diff", "--cached", "--no-ext-diff",
    "--unified=0", "--no-color"
)

patterns = [
    re.compile(rb"-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----"),
    re.compile(rb"\bgh[pousr]_[A-Za-z0-9]{30,}\b"),
    re.compile(rb"\bgithub_pat_[A-Za-z0-9_]{30,}\b"),
    re.compile(rb"\bAKIA[0-9A-Z]{16}\b"),
    re.compile(rb"(?i)\bBearer\s+[A-Za-z0-9._~+/=-]{24,}")
]

for line in diff.splitlines():
    if not line.startswith(b"+") or line.startswith(b"+++"):
        continue

    if any(p.search(line) for p in patterns):
        print("[STOP] Potential credential detected in staged changes.")
        print("[ACTION] Inspect staged files before proceeding.")
        sys.exit(1)

print("[PASS] Sensitive filename check")
print("[PASS] Large file check")
print("[PASS] Credential pattern check")
PY

# 6. Check patch formatting
git diff --cached --check

echo "[PASS] Git diff check"

# 7. Commit changes
echo
echo "[4/6] Creating Git commit..."

if ! git diff --cached --quiet; then
    MESSAGE="backup: $(date '+%Y-%m-%d %H:%M:%S')"
    git commit -m "$MESSAGE"
else
    echo "[INFO] No new file changes"
fi

# 8. Push to GitHub
echo
echo "[5/6] Synchronizing GitHub..."

LOCAL_NOW=$(git rev-parse HEAD)

if [[ "$LOCAL_NOW" != "$REMOTE_BEFORE" ]]; then
    git push origin HEAD:refs/heads/main
else
    echo "[INFO] No new commits to push"
fi

# 9. Verify
echo
echo "[6/6] Verifying remote..."

git fetch --quiet origin main

LOCAL_FINAL=$(git rev-parse HEAD)
REMOTE_FINAL=$(git rev-parse origin/main)

if [[ "$LOCAL_FINAL" != "$REMOTE_FINAL" ]]; then
    echo "[FAIL] Local and remote commits differ"
    exit 1
fi

echo
echo "=========================================="
echo " BACKUP SUCCESS"
echo "=========================================="
echo "Repository: $EXPECTED_REMOTE"
echo "Branch:     $BRANCH"
echo "Commit:     $LOCAL_FINAL"
echo
git status --short --branch
