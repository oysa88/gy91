//% color=#FF4E2B weight=35 icon="\uf14e" block="GY91 - 9DOF"
namespace GY91 {

    const MPU = 0x68
    const AK8963 = 0x0C
    const BMP280 = 0x76

    let gyroOffsetX = 0
    let gyroOffsetY = 0
    let gyroOffsetZ = 0

    let yaw = 0
    let lastTime = 0

    //% block="initialiser GY91"
    //% group="Oppsett"
    //% weight=100
    export function init(): void {
        // --- MPU9250 WAKE ---
        write8(MPU, 0x6B, 0x00)
        write8(MPU, 0x1B, 0x00) // gyro ±250
        write8(MPU, 0x1C, 0x00) // accel ±2g

        // --- AK8963 magnetometer enable ---
        write8(MPU, 0x37, 0x02) // bypass mode
        basic.pause(10)
        write8(AK8963, 0x0A, 0x16) // continuous measurement mode 2

        // --- BMP280 init ---
        write8(BMP280, 0xF4, 0x27) // temp + pressure on
        write8(BMP280, 0xF5, 0xA0) // config

        lastTime = input.runningTime()
    }

    // ==================================================
    // ACCEL
    // ==================================================

    //% block="akselerasjon X"
    export function accelX(): number { return read16(MPU, 0x3B) }

    //% block="akselerasjon Y"
    export function accelY(): number { return read16(MPU, 0x3D) }

    //% block="akselerasjon Z"
    export function accelZ(): number { return read16(MPU, 0x3F) }

    // ==================================================
    // GYRO
    // ==================================================

    //% block="rotasjonshastighet X (°/s)"
    export function gyroX(): number { return (read16(MPU, 0x43) - gyroOffsetX) / 131 }

    //% block="rotasjonshastighet Y (°/s)"
    export function gyroY(): number { return (read16(MPU, 0x45) - gyroOffsetY) / 131 }

    //% block="rotasjonshastighet Z (°/s)"
    export function gyroZ(): number { return (read16(MPU, 0x47) - gyroOffsetZ) / 131 }

    //% block="kalibrer gyroskop"
    export function calibrateGyro(): void {
        let sx = 0, sy = 0, sz = 0
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
    // MAGNETOMETER (AK8963)
    // ==================================================

    function mag16(reg: number): number {
        pins.i2cWriteNumber(AK8963, reg, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(AK8963, NumberFormat.Int16LE)
    }

    //% block="magnetfelt X"
    export function magX(): number { return mag16(0x03) }

    //% block="magnetfelt Y"
    export function magY(): number { return mag16(0x05) }

    //% block="magnetfelt Z"
    export function magZ(): number { return mag16(0x07) }

    //% block="kompassretning (grader)"
    export function heading(): number {
        let angle = Math.atan2(magY(), magX()) * 180 / Math.PI
        if (angle < 0) angle += 360
        return angle
    }

    // ==================================================
    // BMP280
    // ==================================================

    function read24(reg: number): number {
        pins.i2cWriteNumber(BMP280, reg, NumberFormat.UInt8BE, true)
        let msb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        let lsb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        let xlsb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        return (msb << 16) | (lsb << 8) | xlsb
    }

    //% block="temperatur (råverdi)"
    export function temperatureRaw(): number {
        return read24(0xFA) >> 4
    }

    //% block="trykk (råverdi)"
    export function pressureRaw(): number {
        return read24(0xF7) >> 4
    }

    // ==================================================
    // ORIENTATION
    // ==================================================

    //% block="helning fremover/bakover (pitch)"
    export function pitch(): number {
        let ax = accelX()
        let ay = accelY()
        let az = accelZ()
        return Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI
    }

    //% block="helning sideveis (roll)"
    export function roll(): number {
        return Math.atan2(accelY(), accelZ()) * 180 / Math.PI
    }

    //% block="rotasjonsvinkel rundt Z (yaw)"
    export function yawAngle(): number {
        let now = input.runningTime()
        let dt = (now - lastTime) / 1000
        lastTime = now
        yaw += gyroZ() * dt
        return yaw
    }

    // ==================================================
    // I2C HELPERS
    // ==================================================

    function write8(addr: number, reg: number, val: number): void {
        pins.i2cWriteBuffer(addr, Buffer.fromArray([reg, val]))
    }

    function read16(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(addr, NumberFormat.Int16BE)
    }
}
