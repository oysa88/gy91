//% color=#FF4E2B weight=35 icon="\uf14e" block="GY91 - 9DOF"
namespace GY91 {

    const MPU = 0x68
    const AK8963 = 0x0C
    const HMC5883 = 0x1E
    const BMP280 = 0x76

    const ACCEL_SCALE = 16384
    const GYRO_SCALE = 131
    const MAG_SCALE = 0.15

    let gyroOffsetX = 0
    let gyroOffsetY = 0
    let gyroOffsetZ = 0

    let yaw = 0
    let lastTime = 0

    let magAddr = 0
    let magType = 0 // 1 = AK8963, 2 = HMC5883L

    let dig_T1 = 0, dig_T2 = 0, dig_T3 = 0
    let dig_P1 = 0, dig_P2 = 0, dig_P3 = 0
    let dig_P4 = 0, dig_P5 = 0, dig_P6 = 0
    let dig_P7 = 0, dig_P8 = 0, dig_P9 = 0
    let tFine = 0

    function round2(v: number): number {
        return Math.round(v * 100) / 100
    }

    //% block="initialiser GY91"
    //% group="Oppsett"
    //% weight=100
    export function init(): void {
        write8(MPU, 0x6B, 0x00)
        write8(MPU, 0x1B, 0x00)
        write8(MPU, 0x1C, 0x00)

        write8(BMP280, 0xF4, 0x27)
        write8(BMP280, 0xF5, 0xA0)

        readCalibration()
        detectMagnetometer()
        lastTime = input.runningTime()
    }

    // ================= AKSELEROMETER =================

    //% block="akselerasjon X (g)"
    //% group="Akselerometer"
    export function accelX(): number { return round2(read16(MPU, 0x3B) / ACCEL_SCALE) }

    //% block="akselerasjon Y (g)"
    //% group="Akselerometer"
    export function accelY(): number { return round2(read16(MPU, 0x3D) / ACCEL_SCALE) }

    //% block="akselerasjon Z (g)"
    //% group="Akselerometer"
    export function accelZ(): number { return round2(read16(MPU, 0x3F) / ACCEL_SCALE) }

    //% block="total akselerasjon (g)"
    //% group="Akselerometer"
    export function totalAcceleration(): number {
        let x = accelX(), y = accelY(), z = accelZ()
        return round2(Math.sqrt(x * x + y * y + z * z))
    }

    // ================= GYROSKOP =================

    //% block="rotasjonshastighet X (°/s)"
    //% group="Gyroskop"
    export function gyroX(): number { return round2((read16(MPU, 0x43) - gyroOffsetX) / GYRO_SCALE) }

    //% block="rotasjonshastighet Y (°/s)"
    //% group="Gyroskop"
    export function gyroY(): number { return round2((read16(MPU, 0x45) - gyroOffsetY) / GYRO_SCALE) }

    //% block="rotasjonshastighet Z (°/s)"
    //% group="Gyroskop"
    export function gyroZ(): number { return round2((read16(MPU, 0x47) - gyroOffsetZ) / GYRO_SCALE) }

    //% block="kalibrer gyroskop"
    //% group="Gyroskop"
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

    // ================= MAGNETOMETER AUTO =================

    function detectMagnetometer(): void {
        write8(MPU, 0x37, 0x02)
        basic.pause(10)

        write8(AK8963, 0x0B, 0x01)
        basic.pause(10)
        write8(AK8963, 0x0A, 0x00)
        basic.pause(10)
        write8(AK8963, 0x0A, 0x16)
        basic.pause(10)

        pins.i2cWriteNumber(AK8963, 0x00, NumberFormat.UInt8BE, true)
        if (pins.i2cReadNumber(AK8963, NumberFormat.UInt8BE) == 0x48) {
            magAddr = AK8963
            magType = 1
            return
        }

        pins.i2cWriteNumber(HMC5883, 0x0A, NumberFormat.UInt8BE, true)
        if (pins.i2cReadNumber(HMC5883, NumberFormat.UInt8BE) == 0x48) {
            write8(HMC5883, 0x00, 0x70)
            write8(HMC5883, 0x01, 0x20)
            write8(HMC5883, 0x02, 0x00)
            magAddr = HMC5883
            magType = 2
        }
    }

    function magRawAK(reg: number): number {
        pins.i2cWriteNumber(magAddr, 0x02, NumberFormat.UInt8BE, true)
        if (!(pins.i2cReadNumber(magAddr, NumberFormat.UInt8BE) & 0x01)) return 0

        pins.i2cWriteNumber(magAddr, reg, NumberFormat.UInt8BE, true)
        let v = pins.i2cReadNumber(magAddr, NumberFormat.Int16LE)

        pins.i2cWriteNumber(magAddr, 0x09, NumberFormat.UInt8BE, true)
        pins.i2cReadNumber(magAddr, NumberFormat.UInt8BE)

        return v
    }

    function magRawHMC(reg: number): number {
        pins.i2cWriteNumber(magAddr, reg, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(magAddr, NumberFormat.Int16BE)
    }

    //% block="magnetfelt X (µT)"
    //% group="Magnetometer"
    export function magX(): number {
        if (magType == 1) return round2(magRawAK(0x03) * MAG_SCALE)
        if (magType == 2) return round2(magRawHMC(0x03) * 0.92)
        return 0
    }

    //% block="magnetfelt Y (µT)"
    //% group="Magnetometer"
    export function magY(): number {
        if (magType == 1) return round2(magRawAK(0x05) * MAG_SCALE)
        if (magType == 2) return round2(magRawHMC(0x07) * 0.92)
        return 0
    }

    //% block="magnetfelt Z (µT)"
    //% group="Magnetometer"
    export function magZ(): number {
        if (magType == 1) return round2(magRawAK(0x07) * MAG_SCALE)
        if (magType == 2) return round2(magRawHMC(0x05) * 0.92)
        return 0
    }

    //% block="kompassretning (grader)"
    //% group="Magnetometer"
    export function heading(): number {
        let angle = Math.atan2(magY(), magX()) * 180 / Math.PI
        if (angle < 0) angle += 360
        return round2(angle)
    }

    // ================= ORIENTERING =================

    //% block="helning fremover/bakover (pitch)"
    //% group="Orientering"
    export function pitch(): number {
        let ax = accelX(), ay = accelY(), az = accelZ()
        return round2(Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI)
    }

    //% block="helning sideveis (roll)"
    //% group="Orientering"
    export function roll(): number {
        return round2(Math.atan2(accelY(), accelZ()) * 180 / Math.PI)
    }

    // ================= ROTASJON =================

    //% block="rotasjonsvinkel rundt Z (yaw)"
    //% group="Rotasjon"
    export function yawAngle(): number {
        let now = input.runningTime()
        let dt = (now - lastTime) / 1000
        lastTime = now
        yaw += gyroZ() * dt
        return round2(yaw)
    }

    // ================= MILJØ =================

    function readCalibration(): void {
        pins.i2cWriteNumber(BMP280, 0x88, NumberFormat.UInt8BE, true)
        let buf = pins.i2cReadBuffer(BMP280, 24)
        dig_T1 = buf.getNumber(NumberFormat.UInt16LE, 0)
        dig_T2 = buf.getNumber(NumberFormat.Int16LE, 2)
        dig_T3 = buf.getNumber(NumberFormat.Int16LE, 4)
        dig_P1 = buf.getNumber(NumberFormat.UInt16LE, 6)
        dig_P2 = buf.getNumber(NumberFormat.Int16LE, 8)
        dig_P3 = buf.getNumber(NumberFormat.Int16LE, 10)
        dig_P4 = buf.getNumber(NumberFormat.Int16LE, 12)
        dig_P5 = buf.getNumber(NumberFormat.Int16LE, 14)
        dig_P6 = buf.getNumber(NumberFormat.Int16LE, 16)
        dig_P7 = buf.getNumber(NumberFormat.Int16LE, 18)
        dig_P8 = buf.getNumber(NumberFormat.Int16LE, 20)
        dig_P9 = buf.getNumber(NumberFormat.Int16LE, 22)
    }

    function read24(reg: number): number {
        pins.i2cWriteNumber(BMP280, reg, NumberFormat.UInt8BE, true)
        let msb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        let lsb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        let xlsb = pins.i2cReadNumber(BMP280, NumberFormat.UInt8BE)
        return (msb << 16) | (lsb << 8) | xlsb
    }

    //% block="temperatur (°C)"
    //% group="Miljø"
    export function temperatureC(): number {
        let adc_T = read24(0xFA) >> 4
        let var1 = (adc_T / 16384 - dig_T1 / 1024) * dig_T2
        let var2 = ((adc_T / 131072 - dig_T1 / 8192) ** 2) * dig_T3
        tFine = var1 + var2
        return round2(tFine / 5120)
    }

    //% block="lufttrykk (Pa)"
    //% group="Miljø"
    export function pressurePa(): number {
        temperatureC()
        let adc_P = read24(0xF7) >> 4
        let var1 = tFine / 2 - 64000
        let var2 = var1 * var1 * dig_P6 / 32768
        var2 += var1 * dig_P5 * 2
        var2 = var2 / 4 + dig_P4 * 65536
        var1 = (dig_P3 * var1 * var1 / 524288 + dig_P2 * var1) / 524288
        var1 = (1 + var1 / 32768) * dig_P1
        if (var1 == 0) return 0
        let p = 1048576 - adc_P
        p = (p - var2 / 4096) * 6250 / var1
        var1 = dig_P9 * p * p / 2147483648
        var2 = p * dig_P8 / 32768
        return round2(p + (var1 + var2 + dig_P7) / 16)
    }

    function write8(addr: number, reg: number, val: number): void {
        pins.i2cWriteBuffer(addr, Buffer.fromArray([reg, val]))
    }

    function read16(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(addr, NumberFormat.Int16BE)
    }
}