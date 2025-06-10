#!/bin/bash

# Bridge Inspector - GitHub Setup Script
echo "==================================="
echo "Bridge Inspector GitHub Setup"
echo "==================================="
echo ""

# Check if remote already exists
if git remote get-url origin &>/dev/null; then
    echo "❌ Remote 'origin' already exists:"
    git remote get-url origin
    echo ""
    read -p "Do you want to remove it and add a new one? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git remote remove origin
    else
        echo "Keeping existing remote. Exiting."
        exit 0
    fi
fi

# Get GitHub username
echo ""
read -p "Enter your GitHub username: " github_username
if [ -z "$github_username" ]; then
    echo "❌ GitHub username cannot be empty"
    exit 1
fi

# Get repository name
echo ""
echo "Enter repository name (press Enter for default: 'bridge-inspector'):"
read -p "> " repo_name
if [ -z "$repo_name" ]; then
    repo_name="bridge-inspector"
fi

# Choose connection method
echo ""
echo "Choose connection method:"
echo "1) HTTPS (recommended for beginners)"
echo "2) SSH (requires SSH key setup)"
read -p "Enter choice (1 or 2): " connection_choice

case $connection_choice in
    1)
        remote_url="https://github.com/${github_username}/${repo_name}.git"
        ;;
    2)
        remote_url="git@github.com:${github_username}/${repo_name}.git"
        ;;
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "📋 Summary:"
echo "  Username: $github_username"
echo "  Repository: $repo_name"
echo "  Remote URL: $remote_url"
echo ""

# Create repository on GitHub
echo "📌 Next steps:"
echo ""
echo "1. Create a new repository on GitHub:"
echo "   🔗 https://github.com/new"
echo ""
echo "   - Repository name: $repo_name"
echo "   - Description: 'Autonomous bridge/building inspection system for ArduCopter'"
echo "   - Keep it Public or Private (your choice)"
echo "   - ⚠️  DO NOT initialize with README, .gitignore, or license"
echo ""
echo "2. After creating the repository on GitHub, press Enter to continue..."
read -p ""

# Add remote and push
echo ""
echo "🚀 Setting up remote and pushing..."
echo ""

# Add remote
echo "→ Adding remote origin..."
git remote add origin "$remote_url"

# Rename branch to main (GitHub default)
echo "→ Renaming branch to main..."
git branch -M main

# Push to GitHub
echo "→ Pushing to GitHub..."
echo ""
if git push -u origin main; then
    echo ""
    echo "✅ Successfully pushed to GitHub!"
    echo ""
    echo "🎉 Your repository is now available at:"
    echo "   https://github.com/${github_username}/${repo_name}"
    echo ""
    echo "📊 Repository contents:"
    echo "   - 28 files"
    echo "   - Complete autonomous inspection system"
    echo "   - AI-powered defect detection"
    echo "   - 7 inspection patterns"
    echo "   - Safety monitoring"
    echo "   - Report generation"
    echo ""
    echo "🚁 To clone and test:"
    echo "   git clone $remote_url"
    echo "   cd $repo_name"
    echo "   pip install -r requirements.txt"
    echo "   python3 visual_demo.py"
    echo ""
else
    echo ""
    echo "❌ Push failed. Possible issues:"
    echo ""
    echo "1. Repository doesn't exist on GitHub yet"
    echo "   → Create it at: https://github.com/new"
    echo ""
    echo "2. Authentication failed"
    echo "   → For HTTPS: You may need a Personal Access Token"
    echo "     Create one at: https://github.com/settings/tokens"
    echo "   → For SSH: Make sure your SSH key is added to GitHub"
    echo "     Check at: https://github.com/settings/keys"
    echo ""
    echo "3. Repository already has content"
    echo "   → Make sure you didn't initialize with README"
    echo ""
    echo "After fixing the issue, you can try pushing again with:"
    echo "   git push -u origin main"
fi