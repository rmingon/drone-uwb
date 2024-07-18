import { expect, test, describe } from "bun:test";
import { Anchor } from "../src/anchor";


describe("Anchor test", () => {
    
    test("Should instance the class", () => {
        const id = "1"
        const ip = "192.168.1.1"
        const anchor = new Anchor('1', '192.168.1.1')
        expect(anchor.ip).toBe(ip)
        expect(anchor.id).toBe(id)
    })

    test("Should call reboot", () => {
        
    })
})