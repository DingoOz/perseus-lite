#! /usr/bin/env bash
# WARNING: This script is intended to be used only through `nix run`.

# make script brittle - fail on any error
set -euo pipefail

# push dev shell environment
nix develop --profile roar-devenv -c true
cachix push perseus-lite roar-devenv

rm roar-devenv*
