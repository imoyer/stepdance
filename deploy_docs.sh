#!/usr/bin/env bash
set -e
set -o pipefail

# === CONFIGURATION ===
# Resolve paths relative to this script's directory so later `cd` doesn't break them
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCS_DIR="$SCRIPT_DIR/../stepdance_docs/html"   # Doxygen HTML output directory (generated)
SOURCE_DOCS_DIR="$SCRIPT_DIR/doc"               # Raw source markdown + images to publish alongside HTML
LIB_DIR="$SCRIPT_DIR/lib"                       # Library source code to zip
ZIP_FILE="$SCRIPT_DIR/Stepdance.zip"            # Zipped library file
TARGET_REPO="git@github.com:pixelmaid/stepdance_docs.git"  # CHANGE THIS to your target repository
TARGET_BRANCH="main"
SOURCE_BRANCH="main"
DOXYGEN_CONFIG="doxygen/stepdance_config"

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

# === CREATE STEPDANCE.ZIP FROM LIB FOLDER ===
echo "📦 Creating Stepdance.zip from lib folder..."
if [ -d "$LIB_DIR" ]; then
  # Remove existing zip if it exists
  rm -f "$ZIP_FILE"
  
  # Create a temporary directory and copy lib folder as "Stepdance"
  TEMP_LIB_DIR=$(mktemp -d)
  cp -R "$LIB_DIR" "$TEMP_LIB_DIR/Stepdance"
  
  # Create zip from the renamed folder
  cd "$TEMP_LIB_DIR"
  zip -r "$ZIP_FILE" Stepdance -x "*.git*" -x "*__pycache__*" -x "*.DS_Store"
  
  # Clean up temp directory
  rm -rf "$TEMP_LIB_DIR"
  
  echo "✅ Stepdance.zip created successfully"
else
  echo "❌ Library directory not found: $LIB_DIR"
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
# 1. Generated HTML site contents into repo root
cp -R "$DOCS_DIR"/. .

# 2. Raw source documentation (markdown + images) into a separate folder to allow downstream editing/reference.
#    Using 'source_doc' to avoid collisions with any Doxygen output directories.
if [ -d "$SOURCE_DOCS_DIR" ]; then
  mkdir -p source_doc
  cp -R "$SOURCE_DOCS_DIR"/. source_doc/
else
  echo "⚠️ Source documentation directory not found: $SOURCE_DOCS_DIR (skipping copy)"
fi

# 3. Copy Stepdance.zip library file to repo root
if [ -f "$ZIP_FILE" ]; then
  cp "$ZIP_FILE" .
  echo "📦 Copied Stepdance.zip to documentation repository"
else
  echo "⚠️ Stepdance.zip not found: $ZIP_FILE (skipping copy)"
fi

# === COMMIT AND PUSH DOCS ===
git add --all
if git diff --staged --quiet; then
  echo "No changes to commit."
else
  git commit -m "Update Doxygen docs (HTML + source markdown + Stepdance.zip) for commit $commit_hash"
  git push origin $TARGET_BRANCH
  echo "✅ Documentation successfully deployed to '$TARGET_REPO' ($TARGET_BRANCH branch)!"
fi

# === CLEANUP ===
cd -
rm -rf "$TEMP_DIR"
echo "🧹 Cleaned up temporary directory."
