import type { AnchorRange, AnchorTag } from ".";
import { getAnchor, setPeerLink } from "./network";
import { registerTagAddress, resolveTagAddress, setRange } from "./ranging";

const UDP_PORT = 7051;

const client = await Bun.udpSocket({});

export class Anchor {
  id: string;
  ip: string;
  address: number;
  main: boolean;

  constructor(id: string, ip: string, address = 0, main = false) {
    this.id = id;
    this.ip = ip;
    this.address = address;
    this.main = main;
  }

  reboot() {
    this.udpSend({ reboot: true });
  }

  /**
   * Antenna delay is the dominant constant error in a two way ranging setup,
   * roughly 4.7 mm per unit. The anchor keeps the value in NVS.
   */
  setAntennaDelay(antennaDelay: number) {
    this.udpSend({ antenna_delay: antennaDelay });
  }

  /** Main anchor only: it just handed `address` to the tag with this EUI. */
  registerTag({ eui, address }: AnchorTag) {
    registerTagAddress(address, eui);
  }

  setRange(range: AnchorRange) {
    const tag = resolveTagAddress(range.address);
    if (!tag) {
      // the main anchor has not announced this address yet, nothing to attach
      // the measurement to
      return;
    }

    setRange({
      anchor: this.id,
      tag,
      distance: range.range,
      rawDistance: range.raw_range,
      rxPower: range.rx_power,
      fpPower: range.fp_power,
      los: range.los,
      at: Date.now(),
    });
  }

  /** One leg of the self survey, measured by this anchor against a peer. */
  setPeerRange(range: AnchorRange) {
    const self = getAnchor(this.id);
    if (!self) return;
    setPeerLink(self.address, range.address, range.range, range.los);
  }

  udpSend(data: {}) {
    client.send(JSON.stringify(data), UDP_PORT, this.ip);
  }
}
