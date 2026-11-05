# Onboarding: Windows (WSL2)

What works: everything the Linux (no NVIDIA) setup does, with Gazebo windows through WSLg. Run all
commands from a **WSL2 Ubuntu shell**, not PowerShell or Git Bash.

## Install (once)

1. In PowerShell (admin): `wsl --install -d Ubuntu-24.04`, then reboot.
2. Docker Desktop for Windows with *Use the WSL 2 based engine* and *WSL integration* enabled for
   your Ubuntu distribution. Give it at least 8 GB RAM (`.wslconfig` or Docker Desktop settings).
3. In the Ubuntu shell:
   ```bash
   sudo apt update && sudo apt install -y git git-lfs pipx && pipx install pre-commit && git lfs install
   curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin
   ```
4. Foxglove and/or QGroundControl for Windows (they reach the container through localhost).

## First run

**Clone inside the WSL filesystem** (`~/src/...`), not under `/mnt/c/...`: Windows-side checkouts
are slow through the bind mount and OneDrive-synced folders will try to sync build outputs.

```bash
mkdir -p ~/src && cd ~/src
git clone https://github.com/InventoryManagementVIP/CoreDrone.git && cd CoreDrone
just setup && just build && just test
just sim          # Gazebo opens as a WSLg window
```

## Tips

- `.gitattributes` forces LF line endings. If you edit from Windows tools, keep them LF.
- If no Gazebo window appears, check `echo $DISPLAY` in WSL (WSLg sets `:0`) and run `just down`
  then `just sim` so the container picks up the display.
- Git Bash on Windows also works for `just` (the justfile sets `MSYS_NO_PATHCONV`), but WSL2 is
  the supported path.
