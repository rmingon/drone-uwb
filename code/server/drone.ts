import type { DronePosition } from ".";

const client = await Bun.udpSocket({});

export class Drone {
  id: string = "";
  ip: string = "";

  pitch: string = ""
  roll: string = ""
  yaw: string = ""
  throttle: number = 0

  constructor(id: string, ip: string) {
    this.id = id
    this.ip = ip
  }

  setPosition({pitch, roll, yaw, throttle}: DronePosition) {
    this.pitch = pitch
    this.roll = roll
    this.yaw = yaw
    this.throttle = throttle
  }

  reboot() {
    client.send(JSON.stringify({reboot: true}), 7051, this.ip);
  }
}
