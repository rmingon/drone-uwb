import { filter, interval, Subject } from "rxjs";
import { Anchor } from "./anchor";
import { Drone } from "./drone";

export interface DronePosition {
  pitch: string;
  roll: string;
  yaw: string;
  throttle: number;
}

interface DroneConnection {}

interface AnchorConnection {}

export interface AnchorRange {
  uniq: string; // drone uniq
  quality: string;
  power: string;
}

type DataType = DroneConnection | AnchorConnection | AnchorRange | DronePosition

interface Data<T> {
  uniq: string;
  data: T
  type: string
  ip: string
}

const listener = new Subject<any>();
await Bun.udpSocket({
  port: 7051,
  socket: {
    data(_socket, buf, _port, addr) {
      const data : Data<DataType> = JSON.parse(buf.toString())
      data.ip = addr
      listener.next(data);
    }
  }
})

const $anchorConnection = listener.pipe(filter<Data<AnchorConnection>>(data => data.type === "anchor"))
const $droneConnection = listener.pipe(filter<Data<DroneConnection>>(data => data.type === "drone"))
const $anchorRange = listener.pipe(filter<Data<AnchorRange>>(data => data.type === "range"))
const $dronePosition = listener.pipe(filter<Data<DronePosition>>(data => data.type === "position"))

const anchors = new Map<string, Anchor>()
const drones = new Map<string, Drone>()

$anchorConnection.subscribe(({uniq, ip}) => {
  anchors.set(uniq, new Anchor(uniq, ip))
  console.log(`anchors ${anchors.size}`)
})

$anchorRange.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (anchor) 
    anchor.setRange(data)
})

$droneConnection.subscribe(({uniq, ip}) => {
  drones.set(uniq, new Drone(uniq, ip))
  console.log(`drones ${drones.size}`)
})

$dronePosition.subscribe((position) => {
  const drone = drones.get(position.uniq)
  if (drone)
    drone.setPosition(position.data)
})

interval(1000).subscribe(() => {
  

})

console.log("GO")