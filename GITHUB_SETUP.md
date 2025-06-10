# GitHub Setup for Bridge Inspector

## Option 1: Automated Setup (Recommended)

Run the setup script:
```bash
./setup_github.sh
```

This will guide you through the entire process.

## Option 2: Manual Setup

### Step 1: Create Repository on GitHub

1. Go to: https://github.com/new
2. Fill in:
   - **Repository name**: `bridge-inspector`
   - **Description**: `Autonomous bridge/building inspection system for ArduCopter`
   - **Public/Private**: Your choice
   - ⚠️ **IMPORTANT**: Do NOT check any initialization options:
     - ❌ DO NOT "Add a README file"
     - ❌ DO NOT "Add .gitignore"
     - ❌ DO NOT "Choose a license"
3. Click **"Create repository"**

### Step 2: Push Your Code

After creating the empty repository, run these commands:

```bash
# Add your repository as remote (replace YOUR_USERNAME)
git remote add origin https://github.com/YOUR_USERNAME/bridge-inspector.git

# Rename branch to main
git branch -M main

# Push to GitHub
git push -u origin main
```

### Alternative: Using SSH

If you prefer SSH (requires SSH key setup):
```bash
git remote add origin git@github.com:YOUR_USERNAME/bridge-inspector.git
git branch -M main
git push -u origin main
```

## Troubleshooting

### Authentication Issues (HTTPS)

GitHub now requires Personal Access Tokens instead of passwords:

1. Go to: https://github.com/settings/tokens
2. Click "Generate new token (classic)"
3. Give it a name: "Bridge Inspector Push"
4. Select scopes:
   - ✅ repo (all)
5. Generate token and copy it
6. Use this token as your password when pushing

### Authentication Issues (SSH)

1. Check if you have an SSH key:
   ```bash
   ls -la ~/.ssh/id_*.pub
   ```

2. If not, generate one:
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

3. Add to GitHub:
   - Copy key: `cat ~/.ssh/id_ed25519.pub`
   - Go to: https://github.com/settings/keys
   - Click "New SSH key"
   - Paste and save

### Repository Already Exists Error

If you get an error about the repository already existing:
```bash
# Remove the remote
git remote remove origin

# Add it again
git remote add origin YOUR_REPOSITORY_URL

# Force push (⚠️ this will overwrite)
git push -u origin main --force
```

## After Successful Push

Your repository will be available at:
```
https://github.com/YOUR_USERNAME/bridge-inspector
```

### Repository Statistics:
- 📁 **Files**: 28
- 📄 **Code**: ~7,000 lines
- 🚁 **Features**: Complete autonomous inspection system
- 🤖 **AI**: Defect detection with CNN
- 📊 **Reports**: HTML/PDF generation
- 🛡️ **Safety**: Real-time monitoring

### Quick Test:
```bash
# Clone your repository
git clone https://github.com/YOUR_USERNAME/bridge-inspector.git test-bridge
cd test-bridge

# Install and test
pip install -r requirements.txt
python3 visual_demo.py
```

## Share Your Repository

Once pushed, you can share your repository URL with others:
- Public repo: Anyone can view and clone
- Private repo: You control access

Add a star ⭐ if you like the project!