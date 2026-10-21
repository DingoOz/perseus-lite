import * as ROSLIB from "roslib";

let rosConnectionWrap = $state<ROSLIB.Ros | false>(false);
// The Ros instance we currently own (connected, connecting or waiting to retry).
// Events from any other (stale) instance are ignored, so there is only ever one
// live connection and one pending retry.
let currentRos: ROSLIB.Ros | null = null;
let retryCounter: number = 0;
let retryTimeoutHandle: ReturnType<typeof setTimeout> | null = null;

const MAX_RETRY_EXPONENT = 5; // 2^5 s = 32 s max between retries

export const getRosConnection = () => rosConnectionWrap;

const clearRetry = () => {
  if (retryTimeoutHandle) clearTimeout(retryTimeoutHandle);
  retryTimeoutHandle = null;
};

// Drop the current instance without triggering its auto-retry.
const releaseCurrent = () => {
  const previous = currentRos;
  currentRos = null;
  rosConnectionWrap = false;
  previous?.close();
};

export const connectRos = (address: string) => {
  clearRetry();
  releaseCurrent();

  const rosConnection = new ROSLIB.Ros({
    url: `ws://${address}:9090`,
  });
  currentRos = rosConnection;

  // print on connection, close and error
  rosConnection.on("connection", () => {
    if (currentRos !== rosConnection) {
      // superseded while connecting
      rosConnection.close();
      return;
    }
    console.log("rosbridge connected");
    clearRetry();
    retryCounter = 0;
    rosConnectionWrap = rosConnection;
  });
  rosConnection.on("error", (error) => {
    console.log("Error connecting to websocket server: ", error);
  });
  rosConnection.on("close", () => {
    // Ignore closes from instances we no longer own (manual disconnect or a
    // newer connectRos() call) so they can't schedule a competing retry.
    if (currentRos !== rosConnection) return;

    console.log("rosbridge disconnected");
    rosConnectionWrap = false;
    clearRetry();

    // Retry every 2^n seconds, up to 32 seconds
    const timeoutTime = Math.pow(2, retryCounter) * 1000;
    if (retryCounter < MAX_RETRY_EXPONENT) retryCounter++;
    console.log(`Retrying connection in ${timeoutTime / 1000} seconds...`);
    retryTimeoutHandle = setTimeout(() => {
      retryTimeoutHandle = null;
      if (currentRos === rosConnection) connectRos(address);
    }, timeoutTime);
  });
};

export const disconnectRos = () => {
  clearRetry();
  retryCounter = 0;
  releaseCurrent();
};
