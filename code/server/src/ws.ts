/*
 * Websocket feed for the web interface, and the drone's own connection.
 *
 * Browsers get a full snapshot on connect and then on every tick, which keeps
 * the client stateless. The volume is a few hundred bytes for an installation
 * of this size, so there is nothing to gain from diffing.
 */

import { Subject } from "rxjs";
import { buildSnapshot } from "./snapshot";

const WS_PORT = 1227;
const TOPIC = "state" as const;
const TICK_MS = 250;

/** Messages a drone sends over the socket, shaped like the anchors' UDP ones. */
const incoming = new Subject<any>();

const server = Bun.serve({
  port: WS_PORT,
  fetch(req, server) {
    if (server.upgrade(req)) return;
    return new Response("this endpoint speaks websocket", { status: 426 });
  },
  websocket: {
    open(ws) {
      ws.subscribe(TOPIC);
      ws.send(JSON.stringify(buildSnapshot()));
    },
    message(_ws, message) {
      try {
        incoming.next(JSON.parse(message.toString()));
      } catch {
        // a drone sending something unparseable should not take the server down
      }
    },
    close() {},
  },
});

setInterval(() => {
  server.publish(TOPIC, JSON.stringify(buildSnapshot()));
}, TICK_MS);

export const $wsIncoming = incoming;
export const wsPort = WS_PORT;
