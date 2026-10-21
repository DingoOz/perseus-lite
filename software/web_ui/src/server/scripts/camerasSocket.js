// @ts-nocheck

import { spawn } from "child_process";

let signallingServer = null;

// Camera events are relayed between browsers and the pi-server(s), which turn
// them into gst-launch pipelines. Only well-formed events are relayed; the
// pi-server re-validates everything before spawning anything.
const ACTIONS = new Set([
  "group-description",
  "kill",
  "request-groups",
  "request-stream",
  "group-terminated",
  "device-disconnect",
]);
const TRANSFORMS = new Set([
  "none",
  "clockwise",
  "counterclockwise",
  "rotate-180",
  "horizontal-flip",
  "vertical-flip",
  "upper-left-diagonal",
  "upper-right-diagonal",
  "automatic",
]);
const DEVICE_RE = /^[A-Za-z0-9_.:+-]+$/; // /dev/v4l/by-id entry names
const FILE_NAME_RE = /^[A-Za-z0-9_.-]+$/; // recording basename, no path separators
const MAX_DEVICES = 64;
const MIN_DIMENSION = 16;
const MAX_DIMENSION = 4096;

const isDimension = (value) =>
  Number.isInteger(value) && value >= MIN_DIMENSION && value <= MAX_DIMENSION;

// Returns a clean copy of the event (unknown fields dropped), or null if invalid.
export const sanitizeCameraEvent = (event) => {
  if (!event || typeof event !== "object") return null;
  if (event.type !== "camera" || !ACTIONS.has(event.action)) return null;

  const clean = { type: "camera", action: event.action, data: {} };
  const data = event.data;
  if (data === undefined || data === null) return clean;
  if (typeof data !== "object") return null;

  if (data.devices !== undefined) {
    if (
      !Array.isArray(data.devices) ||
      data.devices.length > MAX_DEVICES ||
      !data.devices.every((d) => typeof d === "string" && DEVICE_RE.test(d))
    )
      return null;
    clean.data.devices = [...data.devices];
  }
  if (data.resolution !== undefined) {
    const { width, height } = data.resolution ?? {};
    if (!isDimension(width) || !isDimension(height)) return null;
    clean.data.resolution = { width, height };
  }
  if (data.transform !== undefined) {
    if (!TRANSFORMS.has(data.transform)) return null;
    clean.data.transform = data.transform;
  }
  if (data.file !== undefined && data.file !== null && data.file !== "") {
    if (
      typeof data.file !== "string" ||
      data.file.length > 128 ||
      !FILE_NAME_RE.test(data.file) ||
      data.file === "." ||
      data.file === ".."
    )
      return null;
    clean.data.file = data.file;
  } else if (data.file !== undefined) {
    clean.data.file = null;
  }
  for (const flag of ["convertFromJpeg", "forceRestart"]) {
    if (data[flag] !== undefined) {
      if (typeof data[flag] !== "boolean") return null;
      clean.data[flag] = data[flag];
    }
  }
  return clean;
};

export const cameraSocket = (socket, server) => {
  socket.on("message", (data) => {
    if (!data || data.type !== "camera") return;

    const event = sanitizeCameraEvent(data);
    if (event === null) {
      console.warn(`Dropped invalid camera event from ${socket.id}`);
      return;
    }

    // start gstreamer signalling server on the first camera event
    if (signallingServer === null) {
      console.log("Starting gstreamer signalling server");
      signallingServer = spawn("gst-webrtc-signalling-server");
      signallingServer.stdout.pipe(process.stdout);
      signallingServer.stderr.pipe(process.stderr);
    }
    server.emit("camera-event", event);
  });
};
