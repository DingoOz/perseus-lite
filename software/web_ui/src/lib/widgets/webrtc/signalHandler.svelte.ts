export interface PeerType {
  sessionId: string;
  name: string;
  online: boolean;
  connection: RTCPeerConnection | null;
  track: MediaStream | null;
}

export let peerConnections = $state<Record<string, PeerType>>({});
let peerId = $state<string | undefined>(undefined);
let producerDeviceMap = $state<Record<string, string>>({});

export let ws: WebSocket | null = null;

interface SignalMessageType {
  type:
    | "welcome"
    | "setPeerStatus"
    | "list"
    | "peerStatusChanged"
    | "startSession"
    | "peer"
    | "sessionStarted"
    | "endSession";
  peerId?: string;
  sessionId?: string;
  roles?: string[];
  meta?: {
    name?: string;
    device?: string;
  };
  producers?: Array<{ id: string; meta: { device: string } }>;
  ice?: any;
  sdp?: any;
}

// Reconnect with exponential backoff: 500 ms, 1 s, 2 s, 4 s, then every 5 s.
const RECONNECT_BASE_MS = 500;
const RECONNECT_MAX_MS = 5000;
let reconnectAttempts = 0;
let reconnectHandle: ReturnType<typeof setTimeout> | null = null;
let shouldReconnect = false;

const clearReconnect = () => {
  if (reconnectHandle) clearTimeout(reconnectHandle);
  reconnectHandle = null;
};

// Close the current socket without letting its close event trigger a reconnect.
const closeSocket = () => {
  const old = ws;
  ws = null;
  if (old) {
    old.onopen = null;
    old.onerror = null;
    old.onclose = null;
    old.onmessage = null;
    old.close();
  }
};

export const disconnectFromSignallingServer = () => {
  shouldReconnect = false;
  clearReconnect();
  reconnectAttempts = 0;
  closeSocket();
};

export const connectToSignallingServer = (ip: string) => {
  shouldReconnect = true;
  clearReconnect();
  closeSocket();

  const socket = new WebSocket(`ws://${ip}:${8443}`);
  ws = socket;
  socket.onopen = () => {
    reconnectAttempts = 0;
    console.log("[WS] Connected to signalling server");
  };
  socket.onerror = (event) => {
    // an error is always followed by a close event, which schedules the reconnect
    console.log("[WS Error] -", event);
  };
  socket.onclose = () => {
    if (ws !== socket || !shouldReconnect) return;
    ws = null;
    const delay = Math.min(
      RECONNECT_BASE_MS * 2 ** reconnectAttempts,
      RECONNECT_MAX_MS,
    );
    reconnectAttempts++;
    clearReconnect();
    reconnectHandle = setTimeout(() => {
      reconnectHandle = null;
      if (shouldReconnect) connectToSignallingServer(ip);
    }, delay);
  };
  socket.onmessage = (event) => {
    let data: SignalMessageType;
    try {
      data = JSON.parse(event.data);
    } catch {
      console.warn("[WS] Ignoring non-JSON message");
      return;
    }
    switch (data.type) {
      case "welcome":
        // Sent when the signalling server registers a new connection
        peerId = data.peerId;
        // Tell the signalling server that we are a listener
        if (peerId) {
          wsSend({
            type: "setPeerStatus",
            roles: ["listener"],
            meta: {
              name: "web-client",
            },
          } as SignalMessageType);
        }
        break;
      case "peerStatusChanged":
        // Sent when a peer's status changes
        if (data.peerId === peerId && data.roles?.includes("listener")) {
          // If this device has been confirmed as a listener, request streams
          wsSend({ type: "list" });
        } else if (data.roles?.includes("producer")) {
          // If a camera produces comes online after start up, request its stream
          if (Object.keys(peerConnections).includes(data.meta?.device!)) {
            peerConnections[data.meta?.device!].online = true;
            producerDeviceMap[data.peerId!] = data.meta?.device!;
            wsSend({
              type: "startSession",
              peerId: data.peerId, // camera peerId
            });
          }
        }
        break;
      case "list":
        if (data.producers && data.producers.length > 0) {
          data.producers.forEach((producer) => {
            if (
              peerConnections[producer.meta.device] &&
              !peerConnections[producer.meta.device].online
            ) {
              // If the producer is already in the list, just set it to online
              peerConnections[producer.meta.device].online = true;
              producerDeviceMap[producer.id] = producer.meta.device;
              wsSend({
                type: "startSession",
                peerId: producer.id, // camera peerId
              });
            }
          });
        }
        break;
      case "sessionStarted":
        // When a session is started, we need to create a new RTCPeerConnection
        const targetDevice = producerDeviceMap[data.peerId!];

        peerConnections[targetDevice] = {
          sessionId: data.sessionId!,
          name: peerConnections[targetDevice].name,
          online: true,
          connection: new RTCPeerConnection(),
          track: null,
        };

        peerConnections[targetDevice].connection!.onicecandidate = (event) => {
          if (event.candidate && data.sessionId !== null) {
            wsSend({
              type: "peer",
              sessionId: data.sessionId,
              ice: event.candidate.toJSON(),
            });
          }
        };

        peerConnections[targetDevice].connection!.ontrack = (event) => {
          peerConnections[targetDevice].track = new MediaStream([event.track]);
        };
        break;
      case "peer":
        // Peer messages are for negotiating the connection

        // determine target device from the sessionId
        const peerDevice: string | undefined = Object.keys(
          peerConnections,
        ).find(
          (device) => peerConnections[device].sessionId === data.sessionId,
        );

        if (!peerDevice) {
          console.warn(
            "[WS] No peer device found for sessionId:",
            data.sessionId,
          );
          return;
        }

        if (peerConnections[peerDevice].connection === null) {
          console.error(
            "[WS] No connection found for peer device:",
            peerDevice,
          );
          return;
        }

        // acknowledge sdp and ice negotiation
        if (data.sdp) {
          peerConnections[peerDevice].connection
            .setRemoteDescription(data.sdp)
            .then(() => {
              return peerConnections[peerDevice].connection!.createAnswer();
            })
            .then((answer) => {
              return peerConnections[
                peerDevice
              ].connection!.setLocalDescription(answer);
            })
            .then(() => {
              wsSend({
                type: "peer",
                sessionId: data.sessionId,
                sdp: peerConnections[
                  peerDevice
                ].connection!.localDescription!.toJSON(),
              });
            });
        } else if (data.ice) {
          peerConnections[peerDevice].connection!.addIceCandidate(
            new RTCIceCandidate(data.ice),
          );
        } else {
          console.error("[ws] Unknown peer message:", data);
        }
        break;
      case "endSession":
        // Remove stream when session ends
        const deviceToRemove: string | undefined = Object.keys(
          peerConnections,
        ).find(
          (device) => peerConnections[device].sessionId === data.sessionId,
        );
        if (deviceToRemove) {
          peerConnections[deviceToRemove].track = null;
          peerConnections[deviceToRemove].connection?.close();
          peerConnections[deviceToRemove].connection = null;
          peerConnections[deviceToRemove].sessionId = "";
          peerConnections[deviceToRemove].online = false;
        }
        break;
      default:
        console.log("[WS] Unknown message type:", data);
    }
  };
};

export const newPeerConfigured = () => {
  wsSend({ type: "list" });
};

const wsSend = (data: SignalMessageType) => {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  // console.log('Sending:', data);
  ws.send(JSON.stringify(data));
};

export const getPeerId = () => {
  if (peerId) return peerId;
  return null;
};
