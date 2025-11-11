#!/usr/bin/env bash
set -e
set -o pipefail

# === CONFIGURATION ===
DOCS_DIR="../stepdance_documentation/html"   # Doxygen HTML output directory
BRANCH="gh-pages"
MAIN_BRANCH="main"
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

if [ "$current_branch" != "$MAIN_BRANCH" ]; then
  echo "⚠️ You are on '$current_branch', not '$MAIN_BRANCH'. Proceeding anyway..."
fi

# === FETCH AND CHECKOUT GH-PAGES BRANCH ===
echo "📂 Switching to $BRANCH branch..."
git fetch origin $BRANCH || true

if git show-ref --verify --quiet refs/heads/$BRANCH; then
  git checkout $BRANCH
else
  echo "📦 Creating local $BRANCH branch..."
  git checkout --orphan $BRANCH
  git reset --hard
fi

# === CLEAN AND COPY NEW DOCS ===
echo "🧹 Cleaning old files..."
rm -rf ./*

echo "📋 Copying new documentation..."
cp -r "$DOCS_DIR"/* .

# === COMMIT AND PUSH DOCS ===
git add --all
git commit -m "Update Doxygen docs for commit $commit_hash" || echo "No changes to commit."
git push origin $BRANCH

# === SWITCH BACK TO MAIN BRANCH ===
echo "🔄 Switching back to $MAIN_BRANCH branch..."
git checkout $MAIN_BRANCH

echo "✅ Documentation successfully deployed to '$BRANCH' branch!"
