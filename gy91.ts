//% color=#FF4E2B weight=35 icon="\uf14e" block="GY91 - 9DOF"
namespace GY91 {

    const MPU = 0x68
    const BMP280 = 0x76
    const AK8963 = 0x0C
    const HMC5883 = 0x1E
    const QMC5883 = 0x0D

    const ACCEL_SCALE = 16384
    const GYRO_SCALE = 131
    const MAG_AK_SCALE = 0.15
    const MAG_HMC_SCALE = 0.92
    const MAG_QMC_SCALE = 0.083

    let gyroOffsetX = 0
    let gyroOffsetY = 0
    let gyroOffsetZ = 0
    let yaw = 0
    let lastTime = 0
    let magType = 0

    let dig_T1 = 0, dig_T2 = 0, dig_T3 = 0
    let dig_P1 = 0, dig_P2 = 0, dig_P3 = 0
    let dig_P4 = 0, dig_P5 = 0, dig_P6 = 0
    let dig_P7 = 0, dig_P8 = 0, dig_P9 = 0
    let tFine = 0

    //% block="initialiser GY91"
    //% group="Oppsett"
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

    export enum Axis { X, Y, Z }

    export enum Tilt {
        //% block="pitch (frem/bak)"
        Pitch,
        //% block="roll (sideveis)"
        Roll
    }

    function round2(v: number): number {
        return Math.round(v * 100) / 100
    }

    //% block="akselerasjon %axis (g)"
    //% group="Akselerometer"
    export function acceleration(axis: Axis): number {
        let raw = axis == Axis.X ? read16(MPU, 0x3B)
            : axis == Axis.Y ? read16(MPU, 0x3D)
                : read16(MPU, 0x3F)
        return round2(raw / ACCEL_SCALE)
    }

    //% block="rotasjonshastighet %axis (°/s)"
    //% group="Gyroskop"
    export function gyro(axis: Axis): number {
        let raw = axis == Axis.X ? read16(MPU, 0x43) - gyroOffsetX
            : axis == Axis.Y ? read16(MPU, 0x45) - gyroOffsetY
                : read16(MPU, 0x47) - gyroOffsetZ
        return round2(raw / GYRO_SCALE)
    }

    //% block="magnetfelt %axis (µT)"
    //% group="Magnetometer"
    export function magneticField(axis: Axis): number {
        if (magType == 1) {
            let reg = axis == Axis.X ? 0x03 : axis == Axis.Y ? 0x05 : 0x07
            return round2(magRawAK(reg) * MAG_AK_SCALE)
        }
        if (magType == 2) {
            let reg = axis == Axis.X ? 0x03 : axis == Axis.Y ? 0x07 : 0x05
            return round2(magRawHMC(reg) * MAG_HMC_SCALE)
        }
        if (magType == 3) {
            let x = read16(QMC5883, 0x00)
            let y = read16(QMC5883, 0x02)
            let z = read16(QMC5883, 0x04)
            let v = axis == Axis.X ? x : axis == Axis.Y ? y : z
            return round2(v * MAG_QMC_SCALE)
        }
        return 0
    }

    //% block="kompassretning (grader)"
    //% group="Magnetometer"
    export function heading(): number {
        let angle = Math.atan2(magneticField(Axis.Y), magneticField(Axis.X)) * 180 / Math.PI
        if (angle < 0) angle += 360
        return round2(angle)
    }

    //% block="magnetometer type"
    //% group="Magnetometer"
    export function magnetometerType(): string {
        if (magType == 1) return "AK8963 (MPU9250)"
        if (magType == 2) return "HMC5883L"
        if (magType == 3) return "QMC5883L (klone)"
        return "Ikke funnet"
    }

    //% block="helning %t (grader)"
    //% group="Orientering"
    export function tilt(t: Tilt): number {
        let ax = acceleration(Axis.X)
        let ay = acceleration(Axis.Y)
        let az = acceleration(Axis.Z)

        if (t == Tilt.Pitch)
            return round2(Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI)
        else
            return round2(Math.atan2(ay, az) * 180 / Math.PI)
    }

    //% block="rotasjonsvinkel rundt Z (yaw)"
    //% group="Rotasjon"
    export function yawAngle(): number {
        let now = input.runningTime()
        let dt = (now - lastTime) / 1000
        lastTime = now
        yaw += gyro(Axis.Z) * dt
        return round2(yaw)
    }

    //% block="temperatur (°C)"
    //% group="Miljø"
    export function temperature(): number {
        let adc_T = read24(BMP280, 0xFA) >> 4
        let var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11
        let var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14
        tFine = var1 + var2
        return round2(((tFine * 5 + 128) >> 8) / 100)
    }

    //% block="lufttrykk (Pa)"
    //% group="Miljø"
    export function pressure(): number {
        temperature()
        let adc_P = read24(BMP280, 0xF7) >> 4
        let var1 = tFine - 128000
        let var2 = var1 * var1 * dig_P6
        var2 = var2 + ((var1 * dig_P5) << 17)
        var2 = var2 + (dig_P4 << 35)
        var1 = ((var1 * var1 * dig_P3) >> 8) + ((var1 * dig_P2) << 12)
        var1 = (((1 << 47) + var1) * dig_P1) >> 33
        if (var1 == 0) return 0
        let p = 1048576 - adc_P
        p = (((p << 31) - var2) * 3125) / var1
        var1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (dig_P8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (dig_P7 << 4)
        return round2(p / 256)
    }

    function detectMagnetometer() {
        pins.i2cWriteNumber(AK8963, 0x00, NumberFormat.UInt8BE, true)
        if (pins.i2cReadNumber(AK8963, NumberFormat.UInt8BE, true) == 0x48) {
            write8(AK8963, 0x0A, 0x16)
            magType = 1
            return
        }

        pins.i2cWriteNumber(HMC5883, 0x0A, NumberFormat.UInt8BE, true)
        if (pins.i2cReadNumber(HMC5883, NumberFormat.UInt8BE, true) == 0x48) {
            write8(HMC5883, 0x00, 0x70)
            write8(HMC5883, 0x01, 0x20)
            write8(HMC5883, 0x02, 0x00)
            magType = 2
            return
        }

        pins.i2cWriteNumber(QMC5883, 0x0D, NumberFormat.UInt8BE, true)
        let id = pins.i2cReadNumber(QMC5883, NumberFormat.UInt8BE, true)
        if (id == 0xFF || id == 0x01) {
            write8(QMC5883, 0x0B, 0x01)
            write8(QMC5883, 0x09, 0x1D)
            magType = 3
        }
    }

    function magRawAK(reg: number): number {
        pins.i2cWriteNumber(AK8963, reg, NumberFormat.UInt8BE)
        let lo = pins.i2cReadNumber(AK8963, NumberFormat.UInt8BE)
        let hi = pins.i2cReadNumber(AK8963, NumberFormat.UInt8BE)
        let v = (hi << 8) | lo
        return v > 32767 ? v - 65536 : v
    }

    function magRawHMC(reg: number): number {
        return read16(HMC5883, reg)
    }

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

    function read24(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE)
        let msb = pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
        let lsb = pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
        let xlsb = pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
        return (msb << 16) | (lsb << 8) | xlsb
    }

    function readCalibration() {
        dig_T1 = read16(BMP280, 0x88)
        dig_T2 = read16(BMP280, 0x8A)
        dig_T3 = read16(BMP280, 0x8C)
        dig_P1 = read16(BMP280, 0x8E)
        dig_P2 = read16(BMP280, 0x90)
        dig_P3 = read16(BMP280, 0x92)
        dig_P4 = read16(BMP280, 0x94)
        dig_P5 = read16(BMP280, 0x96)
        dig_P6 = read16(BMP280, 0x98)
        dig_P7 = read16(BMP280, 0x9A)
        dig_P8 = read16(BMP280, 0x9C)
        dig_P9 = read16(BMP280, 0x9E)
    }
}