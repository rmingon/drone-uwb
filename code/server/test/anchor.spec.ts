import { expect, test, describe } from "bun:test";
import { Anchor } from "../src/anchor";


describe("Anchor test", () => {
    const id = "1"
    const ip = "192.168.1.1"
    let anchor = new Anchor('1', ip)
    
    test("Should instance the class", () => {
        expect(anchor.ip).toBe(ip)
        expect(anchor.id).toBe(id)
    })

    test("Should call reboot", () => {
        
    })
})