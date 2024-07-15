import { filter, Subject } from "rxjs";
import { Anchor } from "./anchor";
import { Drone } from "./drone";

interface DronePosition {

}

interface DroneConnection {

}

interface AnchorConnection {

}

interface AnchorRange {

}

type typeOfData = AnchorConnection | DroneConnection | DronePosition | AnchorRange;

interface Data<T> {
  uniq: string;
  data: T
  type: string
}

const listener = new Subject<Data<typeOfData>>();
const server = await Bun.udpSocket({
  port: 7051,
  socket: {
    data(socket, buf, port, addr) {
      console.log(`message from ${addr}:${port}:`)
      listener.next(JSON.parse(buf.toString()));
    }
  }
})

const $anchorConnection = listener.pipe(filter<Data<AnchorConnection>>(data => data.type === "anchor"))
const $droneConnection = listener.pipe(filter<Data<DroneConnection>>(data => data.type === "drone"))
const $anchorRange = listener.pipe(filter<Data<AnchorRange>>(data => data.type === "range"))
const $dronePosition = listener.pipe(filter<Data<DronePosition>>(data => data.type === "position"))

const anchors = new Map<string, Anchor>()
const drones = new Map<string, Drone>()

$anchorConnection.subscribe(({uniq}) => {
  anchors.set(uniq, new Anchor(uniq))
})

$droneConnection.subscribe(({uniq}) => {
  drones.set(uniq, new Drone(uniq))
})