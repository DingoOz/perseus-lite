import { io, Socket } from "socket.io-client";
import config from "./config.json";
import fs, { watch } from "node:fs";
import os from "node:os";
import path from "node:path";
import { ChildProcessWithoutNullStreams, spawn } from "node:child_process";

type videoTransformType =
  | "none"
  | "clockwise"
  | "counterclockwise"
  | "rotate-180"
  | "horizontal-flip"
  | "vertical-flip"
  | "upper-left-diagonal"
  | "upper-right-diagonal"
  | "automatic";

interface CameraEventType {
  type: "camera";
  action:
    | "group-description"
    | "kill"
    | "request-groups"
    | "request-stream"
    | "group-terminated"
    | "device-disconnect";
  data: {
    devices?: string[];
    resolution?: { width: number; height: number };
    transform?: videoTransformType;
    file?: string | null;
    convertFromJpeg?: boolean;
    forceRestart?: boolean;
  };
}

const log = (message: string) => {
  console.log(`[Camera server] ${message}`);
};

// -------------------------------------
//         Request validation
// -------------------------------------
// Everything below ends up in a gst-launch-1.0 pipeline description, which
// gst-launch re-parses from its argv (so "!" or spaces inside one argument
// still create new elements). Every client-supplied value is therefore
// validated against a strict whitelist before it gets anywhere near spawn().

const VIDEO_TRANSFORMS: readonly videoTransformType[] = [
  "none",
  "clockwise",
  "counterclockwise",
  "rotate-180",
  "horizontal-flip",
  "vertical-flip",
  "upper-left-diagonal",
  "upper-right-diagonal",
  "automatic",
];
const MIN_DIMENSION = 16;
const MAX_WIDTH = 4096;
const MAX_HEIGHT = 4096;
const DEFAULT_RESOLUTION = { width: 320, height: 240 };
// Recording file names are plain basenames; they are always written under RECORDINGS_DIR.
const FILE_NAME_RE = /^[A-Za-z0-9_.-]+$/;
const MAX_FILE_NAME_LENGTH = 128;
// Characters that are safe to place unquoted in a gst-launch property value.
const SAFE_PATH_RE = /^[A-Za-z0-9_./-]+$/;
const RECORDINGS_DIR = path.join(os.homedir(), "perseus-recordings");

interface StreamRequest {
  device: string;
  resolution: { width: number; height: number };
  transform: videoTransformType;
  file: string | null; // absolute path under RECORDINGS_DIR, or null
  convertFromJpeg: boolean;
  forceRestart: boolean;
  // true when the client specified the stream preferences explicitly
  hasPreferences: boolean;
}

const isDimension = (value: unknown, max: number): value is number =>
  typeof value === "number" &&
  Number.isInteger(value) &&
  value >= MIN_DIMENSION &&
  value <= max;

// Returns the validated request, or a string describing why it was rejected.
const parseStreamRequest = (
  data: CameraEventType["data"] | undefined,
): StreamRequest | string => {
  if (!data || typeof data !== "object") return "missing data";

  const device = Array.isArray(data.devices) ? data.devices[0] : undefined;
  if (typeof device !== "string" || !videoDevices.includes(device))
    return "unknown device";

  let resolution = DEFAULT_RESOLUTION;
  if (data.resolution !== undefined) {
    const { width, height } = (data.resolution ?? {}) as {
      width?: unknown;
      height?: unknown;
    };
    if (!isDimension(width, MAX_WIDTH) || !isDimension(height, MAX_HEIGHT))
      return "invalid resolution";
    resolution = { width, height };
  }

  let transform: videoTransformType = "none";
  if (data.transform !== undefined) {
    if (!VIDEO_TRANSFORMS.includes(data.transform)) return "invalid transform";
    transform = data.transform;
  }

  let file: string | null = null;
  if (data.file !== undefined && data.file !== null && data.file !== "") {
    if (
      typeof data.file !== "string" ||
      data.file.length > MAX_FILE_NAME_LENGTH ||
      !FILE_NAME_RE.test(data.file) ||
      data.file === "." ||
      data.file === ".."
    )
      return "invalid file name (use letters, digits, '_', '.', '-' only)";
    file = path.join(RECORDINGS_DIR, data.file);
    if (path.dirname(file) !== RECORDINGS_DIR || !SAFE_PATH_RE.test(file))
      return "invalid recording path";
  }

  for (const flag of ["convertFromJpeg", "forceRestart"] as const) {
    if (data[flag] !== undefined && typeof data[flag] !== "boolean")
      return `invalid ${flag}`;
  }

  return {
    device,
    resolution,
    transform,
    file,
    convertFromJpeg: data.convertFromJpeg ?? false,
    forceRestart: data.forceRestart ?? false,
    hasPreferences:
      data.resolution !== undefined &&
      data.transform !== undefined &&
      data.convertFromJpeg !== undefined,
  };
};

log("Starting...");

// -------------------------------------
//        Web socket connection
// -------------------------------------

const socket: Socket = io(
  `http://${config.webserverIp}:${config.webserverPort}`,
  {
    reconnection: true,
    reconnectionDelay: 1000,
  },
);

socket.on("connect", () => {
  log("Connected to web server!");
});

let retryCount = 0;
socket.on("connect_error", (error) => {
  // retryCount++;
  // if (retryCount > 15) {
  // 	log(`Giving up. Please check the config.`);
  // 	process.exit(1);
  // }
  log(`Connection error, please check the config. Retrying...`);
});

// -------------------------------------
//           Get hardware info
// -------------------------------------

const formatDeviceName = (device: string): string =>
  // @ts-ignore
  device.replace("-video-index0", "").replace("usb-", "").replaceAll("_", " ");

// Get the list of video devices
let videoDevices: string[] = fs
  .readdirSync("/dev/v4l/by-id")
  .filter((file) => file.endsWith("-index0")); // index 0 is the video and index 1 is the device metadata
videoDevices.forEach((device: string) => {
  log(`Found video device: ${formatDeviceName(device)}`);
});

// watch for changes in the video devices
let fsWatcher = watch("/dev/v4l/by-id", (eventType, filename) => {
  if (
    eventType === "rename" &&
    filename !== null &&
    filename.endsWith("-index0")
  ) {
    if (videoDevices.includes(filename)) {
      log(`Video device removed: ${formatDeviceName(filename)}`);
      videoDevices = videoDevices.filter((device) => device !== filename);

      // remove from gstInstances
      let index = gstInstances.findIndex(
        (instance) => instance.device === filename,
      );
      if (index !== -1) {
        gstInstances[index].instance.kill();
        gstInstances.splice(index, 1);
      }

      socket.send({
        type: "camera",
        action: "device-disconnect",
        data: { devices: [filename] },
      });
    } else {
      log(`Video device added: ${formatDeviceName(filename)}`);
      videoDevices.push(filename);

      initMessage.data.devices = videoDevices;
      socket.send(initMessage);
    }
  }
});

// -------------------------------------
//         Send initial message
// -------------------------------------

let initMessage: CameraEventType = {
  type: "camera",
  action: "group-description",
  data: { devices: videoDevices },
};
socket.send(initMessage);

// -------------------------------------
//        Handle socket events
// -------------------------------------

// Keep track of gstreamer instances since this info cannot be retrieved from the signalling server
let gstInstances: {
  device: string;
  resolution?: { width: number; height: number };
  transform?: videoTransformType;
  file: string | null;
  convertFromJpeg?: boolean;
  instance: ChildProcessWithoutNullStreams;
}[] = [];

// Only ever called with a request that has passed parseStreamRequest().
const startStream = (request: StreamRequest) => {
  const { device, resolution, transform, file, convertFromJpeg } = request;
  let gstArgs = [
    "webrtcsink",
    "stun-server=NULL",
    "name=ws",
    `signaller::uri="ws://${config.webserverIp}:8443"`, // port is always default 8443
    `meta="meta,device=${device}"`,
  ];
  gstArgs.push("v4l2src", `device=/dev/v4l/by-id/${device}`);
  if (convertFromJpeg) {
    gstArgs.push("!", "jpegdec");
  }
  gstArgs.push(
    "!",
    `video/x-raw,width=${resolution.width},height=${resolution.height}`,
    "!",
    "videoconvert",
  );
  if (transform !== "none")
    gstArgs.push("!", `videoflip`, `method=${transform}`);
  // If the video should be saved:
  if (file) {
    fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
    gstArgs.push(
      "!",
      "tee",
      "name=t", // Split the pipeline (to every spot with 't.')
      "t.",
      "!",
      "queue",
      "leaky=2", // If the queue fills up, start dropping frames from the oldest first
      "!",
      "jpegenc",
      "!",
      "avimux",
      "!",
      "filesink",
      `location=${file}`, // Save frames as an avi file under RECORDINGS_DIR
      "t.",
      "!",
      "queue",
    ); // If the webrtcsink fills up the queue (it can't display the video as fast as it's coming in), it will slow down the video
  }
  gstArgs.push("!", "ws.");

  // argv array, no shell
  const instance = spawn("gst-launch-1.0", gstArgs);
  gstInstances.push({
    device: device,
    resolution: resolution,
    transform: transform,
    file: file,
    convertFromJpeg: convertFromJpeg,
    instance: instance,
  });

  log("gst-launch-1.0 " + gstArgs.join(" "));

  instance.stdout.pipe(process.stdout);
  instance.stderr.pipe(process.stderr);
};

socket.on("camera-event", (event: CameraEventType) => {
  // Return if the target device is not owned by this server
  if (!event || typeof event !== "object") return;
  if (
    event.data &&
    Array.isArray(event.data.devices) &&
    !videoDevices.includes(event.data.devices[0])
  )
    return;
  switch (event.action) {
    case "request-groups":
      initMessage.data.devices = videoDevices;
      socket.send(initMessage);
      break;
    case "request-stream": {
      const request = parseStreamRequest(event.data);
      if (typeof request === "string") {
        log(`Rejected stream request: ${request}`);
        break;
      }
      log(
        `Requesting stream for device: ${formatDeviceName(request.device)} @ ${request.resolution.width}x${request.resolution.height} with transform: ${request.transform} and file: ${request.file}`,
      );
      let index = gstInstances.findIndex(
        (instance) => instance.device === request.device,
      );

      if (request.forceRestart && index !== -1) {
        gstInstances[index].instance.kill();
        gstInstances.splice(index, 1);
        index = -1;
      }

      if (index !== -1) {
        const running = gstInstances[index];
        // device is already streaming and no preferences were set
        if (!request.hasPreferences) break;

        // If the stream is already running, check if we need to restart it
        if (
          request.resolution.width === running.resolution?.width &&
          request.resolution.height === running.resolution?.height &&
          request.transform === running.transform &&
          request.convertFromJpeg === running.convertFromJpeg &&
          request.file === running.file
        )
          break;

        // Restart the stream with new preferences
        running.instance.kill();
        gstInstances.splice(index, 1);
      }

      startStream(request);
      break;
    }
    case "group-description":
      // ignore self sent messages
      break;
    default:
      log(`Unknown action: ${event.action}`);
  }
});

// -------------------------------------
//    Gracefully shutdown the server
// -------------------------------------

process.on("SIGINT", () => {
  log("Shutting down...");

  initMessage.action = "group-terminated";
  socket.send(initMessage);
  gstInstances.forEach((gstInstance) => {
    gstInstance.instance.kill();
  });
  fsWatcher.close();
  socket.close();
});
