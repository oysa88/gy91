//% color=#FF4E2B weight=100 icon="\uf14e" block="GY91 - 9DOF"
namespace GY91 {

    const MPU = 0x68
    const HMC = 0x1E

    let gyroOffsetX = 0
    let gyroOffsetY = 0
    let gyroOffsetZ = 0

    let yaw = 0
    let lastTime = 0

    //% block="initialiser GY91"
    //% group="Oppsett"
    //% weight=100
    export function init(): void {
        // MPU6050 wake
        write8(MPU, 0x6B, 0x00)

        // Gyro ±250 dps
        write8(MPU, 0x1B, 0x00)
        // Acc ±2g
        write8(MPU, 0x1C, 0x00)

        // HMC5883L
        write8(HMC, 0x00, 0x70)
        write8(HMC, 0x01, 0x20)
        write8(HMC, 0x02, 0x00)

        lastTime = input.runningTime()
    }

    // ==================================================
    // AKSELEROMETER
    // ==================================================

    //% block="akselerasjon X"
    //% group="Akselerometer"
    export function accelX(): number {
        return read16(MPU, 0x3B)
    }

    //% block="akselerasjon Y"
    //% group="Akselerometer"
    export function accelY(): number {
        return read16(MPU, 0x3D)
    }

    //% block="akselerasjon Z"
    //% group="Akselerometer"
    export function accelZ(): number {
        return read16(MPU, 0x3F)
    }

    // ==================================================
    // GYROSKOP
    // ==================================================

    //% block="rotasjonshastighet X (°/s)"
    //% group="Gyroskop"
    export function gyroX(): number {
        return (read16(MPU, 0x43) - gyroOffsetX) / 131
    }

    //% block="rotasjonshastighet Y (°/s)"
    //% group="Gyroskop"
    export function gyroY(): number {
        return (read16(MPU, 0x45) - gyroOffsetY) / 131
    }

    //% block="rotasjonshastighet Z (°/s)"
    //% group="Gyroskop"
    export function gyroZ(): number {
        return (read16(MPU, 0x47) - gyroOffsetZ) / 131
    }

    //% block="kalibrer gyroskop"
    //% group="Gyroskop"
    //% weight=90
    export function calibrateGyro(): void {
        let sx = 0
        let sy = 0
        let sz = 0

        for (let i = 0; i < 50; i++) {
            sx += read16(MPU, 0x43)
            sy += read16(MPU, 0x45)
            sz += read16(MPU, 0x47)
            basic.pause(10)
        }

        gyroOffsetX = sx / 50
        gyroOffsetY = sy / 50
        gyroOffsetZ = sz / 50
    }

    // ==================================================
    // MAGNETOMETER
    // ==================================================

    //% block="magnetfelt X"
    //% group="Magnetometer"
    export function magX(): number {
        return read16(HMC, 0x03)
    }

    //% block="magnetfelt Y"
    //% group="Magnetometer"
    export function magY(): number {
        return read16(HMC, 0x07)
    }

    //% block="magnetfelt Z"
    //% group="Magnetometer"
    export function magZ(): number {
        return read16(HMC, 0x05)
    }

    //% block="kompassretning (grader)"
    //% group="Magnetometer"
    export function heading(): number {
        let x = magX()
        let y = magY()
        let angle = Math.atan2(y, x) * 180 / Math.PI
        if (angle < 0) angle += 360
        return angle
    }

    // ==================================================
    // ORIENTERING
    // ==================================================

    //% block="helning fremover/bakover (pitch)"
    //% group="Orientering"
    export function pitch(): number {
        let ax = accelX()
        let ay = accelY()
        let az = accelZ()
        return Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI
    }

    //% block="helning sideveis (roll)"
    //% group="Orientering"
    export function roll(): number {
        return Math.atan2(accelY(), accelZ()) * 180 / Math.PI
    }

    // ==================================================
    // ROTASJON
    // ==================================================

    //% block="rotasjonsvinkel rundt Z (yaw)"
    //% group="Rotasjon"
    export function yawAngle(): number {
        let now = input.runningTime()
        let dt = (now - lastTime) / 1000
        lastTime = now
        yaw += gyroZ() * dt
        return yaw
    }

    // ==================================================
    // I2C HJELP
    // ==================================================

    function write8(addr: number, reg: number, val: number): void {
        pins.i2cWriteBuffer(addr, Buffer.fromArray([reg, val]))
    }

    function read16(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE)
        let hi = pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
        let lo = pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
        let v = (hi << 8) | lo
        return v > 32767 ? v - 65536 : v
    }
}