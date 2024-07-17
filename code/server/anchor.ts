import type { AnchorRange } from ".";

const client = await Bun.udpSocket({});

export class Anchor {
    id: string;
    ip: string;

    constructor(id: string, ip: string) {
        this.id = id
        this.ip = ip
    }

    reboot() {
        this.udpSend({reboot: true})
    }

    setRange({uniq, quality, power}: AnchorRange) {
        if (uniq[0] === "A") // ANCHOR
            anchorsRanging[uniq] =      
        if (uniq[0] === "D") // DRONE

        // console.log(data)
    }

    udpSend(data: {}) {
        client.send(JSON.stringify(data), 7051, this.ip);
    }

}