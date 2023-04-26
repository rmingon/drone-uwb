import {WebSocket, WebSocketServer} from 'ws';
import {Drone} from "./drone";

const wss = new WebSocketServer({ port: 8081 });

let drones = new Map<number, Drone>()

let user: WebSocket

interface UserCmd {
  cmd: "FLY" | "STOP" | "SPEED"
  params: string
}

wss.on('connection', function connection(ws, req) {
  ws.on('error', console.error);

  if (req.url === "/user" && !user)
    user = ws

  ws.on('message', function message(rawData) {
    const data = rawData.toString()
    if (req.url === "/drone") {
      if (data.substring(0, 1) === "D") {
        const id = parseInt(data.substring(1))
        drones.set(id, new Drone(id, ws))
        console.log("ADD DRONE", id)
      } else if (parseInt(data.substring(0, 1))) {
        const drone = drones.get(parseInt(data.substring(0, 1)))
        if (drone) {
          drone.parse(data.substring(1))
          user?.send(JSON.stringify(drone))
        }
      }
    }
    if (req.url === "/user") {
      if (data === "ping")
        return
      const user_cmd = JSON.parse(data) as UserCmd
      const drone = drones.get(1)
      if (drone) {
        drone.cmd(user_cmd.cmd, user_cmd.params)
      }
    }
  });
});