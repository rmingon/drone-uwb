import {WebSocket} from "ws";



export class Drone {
  id: string = "";

  x: number = 0
  y: number = 0
  altitude: number = 0

  constructor(id: string) {
    this.id = id
  }
}
