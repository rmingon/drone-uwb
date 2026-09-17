import { filter, interval, Subject } from "rxjs";
import { Anchor } from "./anchor";
import { Drone } from "./drone";
import { getUsableRanges } from "./ranging";

export interface DronePosition {
  pitch: string;
  roll: string;
  yaw: string;
  throttle: number;
}

interface DroneConnection {
  /** EUI the anchors will report this drone under */
  eui: string;
}

interface AnchorConnection {
  /** short address of the anchor in the ranging chain */
  address: number;
  /** the main anchor is the one tags blink at */
  main: boolean;
}

/** Announced by the main anchor when it hands a short address to a tag. */
export interface AnchorTag {
  eui: string;
  address: number;
}

/** One completed two way ranging exchange, as measured by one anchor. */
export interface AnchorRange {
  address: number;   // short address of the tag
  range: number;     // meters, median filtered
  raw_range: number; // meters, this exchange alone
  rx_power: number;  // dBm
  fp_power: number;  // dBm, first path
  los: boolean;      // direct path looks clear
}

type DataType = DroneConnection | AnchorConnection | AnchorRange | AnchorTag | DronePosition

interface Data<T> {
  uniq: string;
  data: T
  type: string
  ip: string
}

const listener = new Subject<Data<DataType>>();

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

/* Narrows the shared stream down to one message type. */
const ofType = <T extends DataType>(type: string) =>
  filter((message: Data<DataType>): message is Data<T> => message.type === type)

const $anchorConnection = listener.pipe(ofType<AnchorConnection>("anchor"))
const $droneConnection = listener.pipe(ofType<DroneConnection>("drone"))
const $anchorTag = listener.pipe(ofType<AnchorTag>("tag"))
const $anchorRange = listener.pipe(ofType<AnchorRange>("range"))
const $dronePosition = listener.pipe(ofType<DronePosition>("position"))

const anchors = new Map<string, Anchor>()
const drones = new Map<string, Drone>()

/** EUI reported by the anchors -> the drone that announced it */
const dronesByEui = new Map<string, Drone>()

$anchorConnection.subscribe(({uniq, ip, data}) => {
  anchors.set(uniq, new Anchor(uniq, ip, data?.address, data?.main))
  console.log(`anchors ${anchors.size}`)
})

$anchorTag.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (anchor)
    anchor.registerTag(data)
})

$anchorRange.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (anchor)
    anchor.setRange(data)
})

$droneConnection.subscribe(({uniq, ip, data}) => {
  const drone = new Drone(uniq, ip)
  drones.set(uniq, drone)
  if (data?.eui)
    dronesByEui.set(data.eui, drone)
  console.log(`drones ${drones.size}`)
})

$dronePosition.subscribe((position) => {
  const drone = drones.get(position.uniq)
  if (drone)
    drone.setPosition(position.data)
})

/* A position needs at least three clear line of sight distances. Until the
   anchor coordinates are known, just report what we have. */
interval(1000).subscribe(() => {
  for (const [eui, drone] of dronesByEui) {
    const ranges = getUsableRanges(eui)
    if (ranges.length === 0) continue
    console.log(
      `${drone.id}: ` +
      ranges.map(r => `${r.anchor}=${r.distance.toFixed(2)}m`).join(" ")
    )
  }
})

export { anchors, drones, dronesByEui }

console.log("GO")
