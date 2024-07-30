import type { AnchorRange } from ".";
import { getAnchorRanging, setAnchorRanging } from "./ranging";

const client = await Bun.udpSocket({});

export class Anchor {
    id: string;
    ip: string;

    constructor(id: string, ip: string) {
        this.id = id.slice(0, 12)
        this.ip = ip
        console.log(ip)
    }

    reboot() {
        this.udpSend({reboot: true})
    }

    setRange({uniq, quality, power}: AnchorRange) {
        if (uniq[0] === "A") // ANCHOR
            setAnchorRanging(this.id, uniq.slice(1, 13), quality)
        if (uniq[0] === "D") // DRONE
            setAnchorRanging(this.id, uniq.slice(1, 13), quality)
        console.log(uniq)
        getAnchorRanging(this.id, uniq)
    }

    udpSend(data: {}) {
        client.send(JSON.stringify(data), 7051, this.ip);
    }

}