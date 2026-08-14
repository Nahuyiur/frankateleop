#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAJECTORY="${ROOT}/episodes/droid_episode_088685/trajectory.npz"

source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate polymetis
cd "${ROOT}/.."

exec python "${ROOT}/replay_delta_eef.py" "${TRAJECTORY}" "$@"
