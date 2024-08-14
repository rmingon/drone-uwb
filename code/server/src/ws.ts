import { Subject } from "rxjs";
import { drones } from ".";

const TOPIC = 'front' as const
const WS_PORT = 1227

const wsServer = new Subject<any>()
const wsPublish = new Subject<any>()

const server = Bun.serve({
    fetch(req, server) {
      if (server.upgrade(req)) {
        return;
      }
      return new Response("Upgrade failed", { status: 500 });
    },
    port: WS_PORT,
    websocket: {
      message(ws, message) {
        wsServer.next(JSON.stringify(message))
      },
      open(ws) {
        ws.subscribe(TOPIC);
        $wsPublish.next({type:'drones', data: [...drones.values()]})
      },
      close(ws, code, message) {},
      drain(ws) {}
    },
});

wsPublish.subscribe((msg: {}) => {
    server.publish(TOPIC, JSON.stringify(msg))
})

export const $wsServer = wsServer
export const $wsPublish = wsPublish