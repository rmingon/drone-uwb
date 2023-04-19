import {WebSocket} from "ws";


export class Drone {

  ws: WebSocket
  id: number
  constructor(id: number, ws: WebSocket) {
    this.id = id
    this.ws = ws
    ws.send("GO")
  }

  setMotorSpeed() {

  }



}