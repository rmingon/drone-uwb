import {WebSocket} from "ws";

export class Drone {

  #ws: WebSocket
  id: number

  x: number = 0
  y: number = 0
  altitude: number = 0

  constructor(id: number, ws: WebSocket) {
    this.id = id
    this.#ws = ws
  }

  setMotorMinSpeed(pwm : number) {
    this.#ws.send(`M${pwm}`)
  }

  stop() {
    this.#ws.send("S")
  }

  reboot() {
    this.#ws.send("R")
  }

  fly() {
    this.#ws.send('F')
  }

  direction(direction: number, acceleration: number) {
    if (direction > 15 || acceleration > 15)
      throw "this method can't take more than 15 in each parameter"
    const move = createBinaryString(direction) + createBinaryString(acceleration)
    const hex = parseInt(move, 2).toString(16);
    this.#ws.send(`D${hex}`)
  }

  parse(raw: string) {
    const cmd = raw.substring(0, 1)
    const data = raw.substring(1)
    if (cmd === "X") {
      this.x = parseInt(data)
    } else if (cmd === "Y") {
      this.y = parseInt(data)
    } else if (cmd === "A") {
      this.altitude = parseInt(data)
    }
  }
}

function createBinaryString(number: number, min_size = 4): string {
  let str = number.toString(2)
  const str_length = str.length
  let empty = ""
  for (let i = str_length; i < min_size; i++) {
    empty += "0"
  }
  return empty + str
}
