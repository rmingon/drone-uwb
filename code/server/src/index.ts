import { filter, merge, Subject } from "rxjs";
import { Anchor } from "./anchor";
import { Drone } from "./drone";
import { registerAnchor, touchAnchor } from "./network";
import { $wsIncoming, wsPort } from "./ws";

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

const udp = new Subject<Data<DataType>>();

await Bun.udpSocket({
  port: 7051,
  socket: {
    data(_socket, buf, _port, addr) {
      try {
        const data : Data<DataType> = JSON.parse(buf.toString())
        data.ip = addr
        udp.next(data);
      } catch {
        // a malformed datagram should not take the server down
      }
    }
  }
})

/* Anchors speak UDP, drones speak websocket, both send the same envelope. */
const listener = merge(udp, $wsIncoming)

/* Narrows the shared stream down to one message type. */
const ofType = <T extends DataType>(type: string) =>
  filter((message: Data<DataType>): message is Data<T> => message.type === type)

const $anchorConnection = listener.pipe(ofType<AnchorConnection>("anchor"))
const $droneConnection = listener.pipe(ofType<DroneConnection>("drone"))
const $anchorTag = listener.pipe(ofType<AnchorTag>("tag"))
const $anchorRange = listener.pipe(ofType<AnchorRange>("range"))
const $anchorPeer = listener.pipe(ofType<AnchorRange>("peer"))
const $dronePosition = listener.pipe(ofType<DronePosition>("position"))

const anchors = new Map<string, Anchor>()
const drones = new Map<string, Drone>()

/** EUI reported by the anchors -> the drone that announced it */
const dronesByEui = new Map<string, Drone>()

$anchorConnection.subscribe(({uniq, ip, data}) => {
  const address = data?.address ?? 0
  const main = data?.main ?? false
  anchors.set(uniq, new Anchor(uniq, ip, address, main))
  registerAnchor({id: uniq, ip, address, main})
  console.log(`anchor ${uniq} is address ${address}${main ? " (main)" : ""}, ${anchors.size} known`)
})

$anchorTag.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (anchor)
    anchor.registerTag(data)
})

$anchorRange.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (!anchor) return
  touchAnchor(uniq)
  anchor.setRange(data)
})

/* A distance to another anchor describes the installation, not a drone. */
$anchorPeer.subscribe(({uniq, data}) => {
  const anchor = anchors.get(uniq)
  if (!anchor) return
  touchAnchor(uniq)
  anchor.setPeerRange(data)
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

export { anchors, drones, dronesByEui }

console.log(`listening for anchors on udp 7051, serving the interface on ws ${wsPort}`)
