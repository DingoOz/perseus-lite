#!/usr/bin/env bash
# Container entrypoint: source the ROS overlay, sync the baked models into the
# models volume, optionally build the TensorRT engine, then exec the command.
#
# - Models: the image bakes them into /opt/perseus/models-image with a STAMP
#   (model hash + base image). Docker only seeds a named volume on first
#   creation, so we re-copy whenever the volume's recorded stamp differs —
#   ie after every image rebuild that changes the model or base image — and
#   drop any engine built from the old model.
# - Engine: OPT-IN via ISAAC_BUILD_ENGINE=1 (default off: YOLOv8 is not wired
#   into perseus_isaac.launch.py yet, and the build takes minutes on an Orin
#   Nano). When enabled it runs in the BACKGROUND so the AprilTag graph starts
#   immediately. A failed build leaves a marker keyed by the stamp, so it is
#   not retried on every container restart; set ISAAC_REBUILD_ENGINE=1 (or
#   delete the marker) to retry.
set -euo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/perseus/ws/install/setup.bash

image_model_dir="/opt/perseus/models-image"
model_dir="/opt/perseus/models"
onnx="${model_dir}/cube_detector_yolob8s.onnx"
engine="${model_dir}/cube_detector_yolob8s.plan"
failed_marker="${engine}.failed"
engine_log="${model_dir}/engine-build.log"

log() { echo "[entrypoint] $*" >&2; }

# --- Sync baked models into the volume when the image changed ---------------
image_stamp=""
if [[ -f "${image_model_dir}/STAMP" ]]; then
  image_stamp="$(cat "${image_model_dir}/STAMP")"
  volume_stamp="$(cat "${model_dir}/STAMP" 2>/dev/null || true)"
  if [[ "${image_stamp}" != "${volume_stamp}" ]]; then
    log "Model volume is stale or new (image: '${image_stamp}', volume: '${volume_stamp}'); syncing."
    mkdir -p "${model_dir}"
    cp -f "${image_model_dir}"/*.onnx "${model_dir}/"
    # Engines/markers built from the previous model or TensorRT are invalid.
    rm -f "${engine}" "${failed_marker}" "${engine_log}"
    cp -f "${image_model_dir}/STAMP" "${model_dir}/STAMP"
  fi
fi

# --- Optional TensorRT engine build (background, opt-in) --------------------
build_engine() {
  local trtexec tmp_engine="${engine}.tmp"
  if command -v trtexec >/dev/null 2>&1; then
    trtexec="trtexec"
  elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
    trtexec="/usr/src/tensorrt/bin/trtexec"
  else
    log "WARNING: trtexec not found; not building the engine."
    echo "${image_stamp} trtexec-missing" >"${failed_marker}"
    return 0
  fi
  log "Building TensorRT engine in the background (log: ${engine_log})..."
  if nice -n 10 "${trtexec}" --onnx="${onnx}" --saveEngine="${tmp_engine}" --fp16 \
    >"${engine_log}" 2>&1; then
    mv -f "${tmp_engine}" "${engine}"
    log "TensorRT engine ready: ${engine}"
  else
    rm -f "${tmp_engine}"
    echo "${image_stamp} build-failed" >"${failed_marker}"
    log "WARNING: engine build failed (see ${engine_log}); not retrying until ISAAC_REBUILD_ENGINE=1 or a new image."
  fi
}

if [[ "${ISAAC_BUILD_ENGINE:-0}" == "1" && -f "${onnx}" && ! -f "${engine}" ]]; then
  if [[ "${ISAAC_REBUILD_ENGINE:-0}" == "1" ]]; then
    rm -f "${failed_marker}"
  fi
  if [[ -f "${failed_marker}" ]]; then
    log "Skipping engine build: previous attempt failed ($(cat "${failed_marker}")). Set ISAAC_REBUILD_ENGINE=1 to retry."
  else
    build_engine &
  fi
fi

exec "$@"
