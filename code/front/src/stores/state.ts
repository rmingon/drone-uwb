/*
 * Live view of the anchor network.
 *
 * The server sends a full snapshot four times a second, so there is no state to
 * merge here: the last message is the truth.
 */

import { reactive, shallowReadonly } from "vue";
import type { Snapshot } from "../types";
import { demoSnapshot } from "./demo";

const RECONNECT_MS = 2000;

const serverUrl = () => {
  const configured = import.meta.env.VITE_SERVER_WS;
  if (configured) return configured as string;
  return `ws://${location.hostname || "localhost"}:1227`;
};

const state = reactive({
  connected: false,
  /** true when showing generated data because the query string asked for it */
  demo: false,
  snapshot: null as Snapshot | null,
  lastError: "",
});

let socket: WebSocket | null = null;
let retry: ReturnType<typeof setTimeout> | null = null;

const connect = () => {
  try {
    socket = new WebSocket(serverUrl());
  } catch (error) {
    state.lastError = String(error);
    scheduleRetry();
    return;
  }

  socket.onopen = () => {
    state.connected = true;
    state.lastError = "";
  };

  socket.onmessage = (event) => {
    try {
      const message = JSON.parse(event.data) as Snapshot;
      if (message.type === "state") state.snapshot = message;
    } catch {
      // a malformed frame is not worth tearing the view down for
    }
  };

  socket.onclose = () => {
    state.connected = false;
    scheduleRetry();
  };

  socket.onerror = () => {
    state.lastError = `cannot reach ${serverUrl()}`;
  };
};

const scheduleRetry = () => {
  if (retry) return;
  retry = setTimeout(() => {
    retry = null;
    connect();
  }, RECONNECT_MS);
};

export const startNetworkState = () => {
  /* With no hardware running there is nothing to look at, so ?demo=1 feeds the
     view a generated installation. */
  if (new URLSearchParams(location.search).has("demo")) {
    state.demo = true;
    state.connected = true;
    setInterval(() => {
      state.snapshot = demoSnapshot();
    }, 100);
    return;
  }
  connect();
};

/* shallow, so the snapshot keeps its own type rather than a deeply readonly one */
export const networkState = shallowReadonly(state);
export const serverAddress = serverUrl;
