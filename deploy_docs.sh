#!/usr/bin/env bash
set -e
set -o pipefail

# === CONFIGURATION ===
DOCS_DIR="../stepdance_documentation/html"   # Doxygen HTML output directory
TARGET_REPO="github.com/pixelmaid/stepdance_docs.git"  # CHANGE THIS to your target repository
TARGET_BRANCH="main"
SOURCE_BRANCH="main"
DOXYGEN_CONFIG="stepdance_config"

# === ENSURE CLEAN WORKING TREE ===
if [ -n "$(git status --porcelain)" ]; then
  echo "Please commit or stash your changes before deploying."
  exit 1
fi

# === BUILD DOXYGEN ===
echo  "Building Doxygen documentation..."
doxygen "$DOXYGEN_CONFIG"

if [ ! -d "$DOCS_DIR" ]; then
  echo "Doxygen output directory not found: $DOCS_DIR"
  exit 1
fi

# === STORE CURRENT BRANCH AND COMMIT HASH ===
current_branch=$(git rev-parse --abbrev-ref HEAD)
commit_hash=$(git rev-parse --short HEAD)

if [ "$current_branch" != "$SOURCE_BRANCH" ]; then
  echo "⚠️ You are on '$current_branch', not '$SOURCE_BRANCH'. Proceeding anyway..."
fi

# === CREATE TEMP DIRECTORY FOR TARGET REPO ===
echo "📂 Cloning target repository..."
TEMP_DIR=$(mktemp -d)
git clone "$TARGET_REPO" "$TEMP_DIR"
cd "$TEMP_DIR"

# === CHECKOUT OR CREATE TARGET BRANCH ===
if git show-ref --verify --quiet refs/remotes/origin/$TARGET_BRANCH; then
  git checkout $TARGET_BRANCH
else
  echo "📦 Creating new $TARGET_BRANCH branch..."
  git checkout --orphan $TARGET_BRANCH
  git reset --hard
fi

# === CLEAN AND COPY NEW DOCS ===
echo "🧹 Cleaning old files..."
rm -rf ./*

echo "📋 Copying new documentation..."
cp -r "$DOCS_DIR"/* .

# === COMMIT AND PUSH DOCS ===
git add --all
if git diff --staged --quiet; then
  echo "No changes to commit."
else
  git commit -m "Update Doxygen docs for commit $commit_hash"
  git push origin $TARGET_BRANCH
  echo "✅ Documentation successfully deployed to '$TARGET_REPO' ($TARGET_BRANCH branch)!"
fi

# === CLEANUP ===
cd -
rm -rf "$TEMP_DIR"
echo "🧹 Cleaned up temporary directory."
