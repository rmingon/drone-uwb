export class Anchor {
    id: string;
    ip: string;

    constructor(id: string, ip: string) {
        this.id = id
        this.ip = ip
    }

    reboot() {

    }

    setRange() {
        throw new Error("Method not implemented.");
    }

}