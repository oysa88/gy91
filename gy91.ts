//% color=#2E8B57 weight=35 icon="\uf14e" block="GY91 - 9DOF"
namespace GY91 {
    // ───────────────── I2C adresser ─────────────────
    let MPU = 0x68 // autodetekteres (0x68/0x69)
    const AK = 0x0C // AK8963
    const QMC = 0x0D // QMC5883L
    const HMC = 0x1E // HMC5883L
    let BMP = 0x76 // autodetekteres (0x76/0x77)

    // ───────────────── Enums (norske navn) ─────────────────
    export enum Akse {
        //% block="X"
        X = 0,
        //% block="Y"
        Y = 1,
        //% block="Z"
        Z = 2
    }

    export enum Helning {
        //% block="forover/bakover"
        ForoverBakover = 0, // Pitch
        //% block="sideveis"
        Sideveis = 1,       // Roll
        //% block="retning"
        Retning = 2         // Yaw (gyro-integrert)
    }

    // ───────────────── Status og innstillinger ─────────────────
    enum MagType { None, AK_Master, QMC_Master, HMC_Master }
    let magType: MagType = MagType.None
    let mpuWho = 0
    let initStarted = false
    let initDone = false

    // QMC ODR: 10Hz=0xD1, 50Hz=0xD5, 200Hz=0xD9 (OSR=128, RNG=8G, CONT)
    let _qmcOdr = 0xD5
    // I2C_MST_CTRL: WAIT_FOR_ES=1 (bit6) + masterklokke (0x0B ≈346k, 0x0D ≈400k)
    let _mstCtrl = 0x4B // default: WAIT_FOR_ES + treg masterklokke

    // ───────────────── Skalaer ─────────────────
    const ACCEL_SCALE = 16384       // ±2g
    const GYRO_SCALE = 131         // ±250 dps
    const MAG_AK_SCALE = 4912.0 / 32760.0 // ≈0.1499 µT/LSB
    const MAG_QMC_SCALE = 0.0667           // ~µT/LSB @8G
    const HMC_SCALE = 0.092            // ~µT/LSB @1.3Ga (CRB=0.20)

    // ───────────────── Gyro / orientering ─────────────────
    let gyroOffsetZ = 0
    let yaw = 0
    let lastTime = 0
    let _akLastX = 0, _akLastY = 0, _akLastZ = 0

    // ───────────────── MAG Hard-iron offset ─────────────────
    let _mOffX = 0, _mOffY = 0, _mOffZ = 0
    let _calRunning = false
    let _minX = 0, _maxX = 0, _minY = 0, _maxY = 0, _minZ = 0, _maxZ = 0

    // ───────────────── BMP280 kalibrering ─────────────────
    let dig_T1 = 0, dig_T2 = 0, dig_T3 = 0
    let dig_P1 = 0, dig_P2 = 0, dig_P3 = 0
    let dig_P4 = 0, dig_P5 = 0, dig_P6 = 0
    let dig_P7 = 0, dig_P8 = 0, dig_P9 = 0
    let tFine = 0
    let bmpPresent = false

    // ───────────────── Hjelpere ─────────────────
    function _feedMinMax(x: number, y: number, z: number) {
        if (!_calRunning) return
        if (x < _minX) _minX = x; if (x > _maxX) _maxX = x
        if (y < _minY) _minY = y; if (y > _maxY) _maxY = y
        if (z < _minZ) _minZ = z; if (z > _maxZ) _maxZ = z
    }

    function toHex2(n: number): string {
        n = n & 0xFF
        const D = "0123456789ABCDEF"
        return D.charAt((n >> 4) & 0xF) + D.charAt(n & 0xF)
    }
    function write8(a: number, r: number, v: number) {
        const b = pins.createBuffer(2)
        b.setNumber(NumberFormat.UInt8BE, 0, r)
        b.setNumber(NumberFormat.UInt8BE, 1, v)
        pins.i2cWriteBuffer(a, b)
    }
    function read8(a: number, r: number): number {
        pins.i2cWriteNumber(a, r, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(a, NumberFormat.UInt8BE)
    }
    function read16BE(a: number, r: number): number {
        pins.i2cWriteNumber(a, r, NumberFormat.UInt8BE, true)
        const hi = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        const lo = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        let v = (hi << 8) | lo
        return v > 32767 ? v - 65536 : v
    }
    function read24(a: number, r: number): number {
        pins.i2cWriteNumber(a, r, NumberFormat.UInt8BE, true)
        const b1 = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        const b2 = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        const b3 = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        return (b1 << 16) | (b2 << 8) | b3
    }

    // ───────────────── Autoinit ─────────────────
    function ensureInit() {
        if (initStarted) return
        initStarted = true
        control.inBackground(function () {
            fullInit()
            initDone = true
        })
    }
    function fullInit() {
        MPU = probeMPU()
        if (MPU == 0) return
        mpuEnableMaster(_mstCtrl)
        mpuSensorTiming()

        if (akMasterInit()) {
            magType = MagType.AK_Master
        } else if (qmcMasterInit()) {
            magType = MagType.QMC_Master
        } else if (hmcMasterInit()) {
            magType = MagType.HMC_Master
        } else {
            magType = MagType.None
        }

        if (probeBMP()) {
            bmpPresent = true
            bmpConfigure()
            bmpReadCoeffs()
        } else {
            bmpPresent = false
        }
        lastTime = input.runningTime()
    }
    function probeMPU(): number {
        basic.pause(400)
        let found = 0
        for (let a of [0x68, 0x69]) {
            for (let i = 0; i < 3; i++) {
                const who = readWhoAt(a)
                if (who == 0x71 || who == 0x73) { mpuWho = who; found = a; break }
                basic.pause(30)
            }
            if (found) break
        }
        return found
    }
    function readWhoAt(addr: number): number {
        pins.i2cWriteNumber(addr, 0x75, NumberFormat.UInt8BE, true)
        return pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
    }
    function mpuEnableMaster(mstCtrl: number) {
        write8(MPU, 0x37, 0x00)              // BYPASS_EN=0
        basic.pause(2)
        write8(MPU, 0x6A, 0x20)              // I2C_MST_EN=1
        basic.pause(2)
        write8(MPU, 0x24, mstCtrl & 0x7F)    // I2C_MST_CTRL (WAIT_FOR_ES|CLK)
        basic.pause(2)
        write8(MPU, 0x67, 0x81)              // SHADOW + SLV0_DLY_EN
        basic.pause(2)
    }
    function mpuSensorTiming() {
        write8(MPU, 0x1A, 0x03) // CONFIG: DLPF_CFG=3
        write8(MPU, 0x19, 0x04) // SMPLRT_DIV=4 (~200 Hz)
        basic.pause(2)
    }

    // ───────── SLV0/SLV4 (med SLV0‑pause rundt SLV4) ─────────
    function slv0Enable(enable: boolean, len: number = 0, addr7: number = 0, startReg: number = 0) {
        if (enable) {
            write8(MPU, 0x26, startReg)
            write8(MPU, 0x25, (addr7 & 0x7F) | 0x80) // read
            write8(MPU, 0x27, 0x80 | (len & 0x0F))
        } else {
            write8(MPU, 0x27, 0x00)
        }
        basic.pause(2)
    }
    function mpuSlv4Read(addr7: number, reg: number): number[] {
        const prevSlv0 = read8(MPU, 0x27)
        write8(MPU, 0x27, 0x00); basic.pause(1)
        write8(MPU, 0x32, reg)
        write8(MPU, 0x31, (addr7 & 0x7F) | 0x80) // read
        write8(MPU, 0x34, 0x80)
        let st = 0, ok = 0, nack = 0
        for (let i = 0; i < 60; i++) {
            st = read8(MPU, 0x36)
            if (st & 0x40) { ok = 1; break } // DONE
            basic.pause(2)
        }
        if (st & 0x10) nack = 1
        const di = read8(MPU, 0x35)
        write8(MPU, 0x34, 0x00)
        write8(MPU, 0x27, prevSlv0)
        basic.pause(1)
        ok = (ok == 1 && nack == 0) ? 1 : 0
        return [ok, nack, di & 0xFF]
    }
    function mpuSlv4Write(addr7: number, reg: number, val: number): boolean {
        const prevSlv0 = read8(MPU, 0x27)
        write8(MPU, 0x27, 0x00); basic.pause(1)
        write8(MPU, 0x32, reg)
        write8(MPU, 0x33, val & 0xFF)
        write8(MPU, 0x31, addr7 & 0x7F) // write
        write8(MPU, 0x34, 0x80)
        let done = false, nack = false
        for (let i = 0; i < 60; i++) {
            const st = read8(MPU, 0x36)
            if (st & 0x40) { done = true; nack = (st & 0x10) != 0; break }
            basic.pause(2)
        }
        write8(MPU, 0x34, 0x00)
        write8(MPU, 0x27, prevSlv0)
        basic.pause(1)
        return done && !nack
    }
    function mpuExtReadBurst(len: number): Buffer {
        const b = pins.createBuffer(len)
        for (let i = 0; i < len; i++) b.setNumber(NumberFormat.UInt8BE, i, read8(MPU, 0x49 + i))
        return b
    }

    // ───────── MAG: AK8963 via master (robust) ─────────
    function akVerifyCntl(v: number): boolean {
        const r = mpuSlv4Read(AK, 0x0A)
        return (r[0] == 1 && r[1] == 0 && r[2] == v)
    }
    function akSetMode(cntl: number): boolean {
        if (!mpuSlv4Write(AK, 0x0A, 0x00)) return false
        basic.pause(10)
        if (!mpuSlv4Write(AK, 0x0B, 0x01)) return false
        basic.pause(10)
        if (!mpuSlv4Write(AK, 0x0A, cntl)) return false
        basic.pause(10)
        return akVerifyCntl(cntl)
    }
    function akMasterInit(): boolean {
        // WHO=0x48
        let whoOK = false
        for (let i = 0; i < 3; i++) {
            const r = mpuSlv4Read(AK, 0x00)
            if (r[0] == 1 && r[2] == 0x48) { whoOK = true; break }
            basic.pause(5)
        }
        if (!whoOK) return false

        // 100 Hz (0x16) → fallback 8 Hz (0x12)
        let ok = false
        for (let n = 0; n < 2 && !ok; n++) ok = akSetMode(0x16)
        if (!ok) for (let n = 0; n < 2 && !ok; n++) ok = akSetMode(0x12)
        if (!ok) return false

        slv0Enable(true, 8, AK, 0x02) // ST1 + XYZ + ST2
        return true
    }
    function akMasterReadAll(): number[] {
        // Returnerer uavrundede verdier i µT (sticky siste gyldige)
        for (let tries = 0; tries < 8; tries++) {
            const d = mpuExtReadBurst(8)
            const st1 = d.getNumber(NumberFormat.UInt8BE, 0)
            if ((st1 & 0x01) == 0) { basic.pause(6); continue } // DRDY=0
            let x = (d.getNumber(NumberFormat.UInt8BE, 2) << 8) | d.getNumber(NumberFormat.UInt8BE, 1)
            let y = (d.getNumber(NumberFormat.UInt8BE, 4) << 8) | d.getNumber(NumberFormat.UInt8BE, 3)
            let z = (d.getNumber(NumberFormat.UInt8BE, 6) << 8) | d.getNumber(NumberFormat.UInt8BE, 5)
            if (x > 32767) x -= 65536
            if (y > 32767) y -= 65536
            if (z > 32767) z -= 65536
            const st2 = d.getNumber(NumberFormat.UInt8BE, 7)
            if (st2 & 0x08) { basic.pause(4); continue } // overflow
            if (x == 0 && y == 0 && z == 0) { basic.pause(6); continue }
            _akLastX = x; _akLastY = y; _akLastZ = z
            return [x * MAG_AK_SCALE, y * MAG_AK_SCALE, z * MAG_AK_SCALE]
        }
        return [_akLastX * MAG_AK_SCALE, _akLastY * MAG_AK_SCALE, _akLastZ * MAG_AK_SCALE]
    }

    // ───────── MAG: QMC5883L via master ─────────
    function qmcMasterInit(): boolean {
        if (!mpuSlv4Write(QMC, 0x0A, 0x80)) return false
        basic.pause(5)
        if (!mpuSlv4Write(QMC, 0x0A, 0x00)) return false
        basic.pause(5)
        if (!mpuSlv4Write(QMC, 0x0B, 0x01)) return false
        basic.pause(5)
        if (!mpuSlv4Write(QMC, 0x09, _qmcOdr)) return false // CTRL1
        basic.pause(10)
        const chk = mpuSlv4Read(QMC, 0x09)
        if (chk[0] == 0 || chk[1] == 1) return false
        if (chk[2] == 0x00 || chk[2] == 0xFF) return false
        slv0Enable(true, 6, QMC, 0x00)
        return true
    }
    function qmcMasterReadAll(): number[] {
        const d = mpuExtReadBurst(6)
        let x = (d.getNumber(NumberFormat.UInt8BE, 1) << 8) | d.getNumber(NumberFormat.UInt8BE, 0)
        let y = (d.getNumber(NumberFormat.UInt8BE, 3) << 8) | d.getNumber(NumberFormat.UInt8BE, 2)
        let z = (d.getNumber(NumberFormat.UInt8BE, 5) << 8) | d.getNumber(NumberFormat.UInt8BE, 4)
        if (x > 32767) x -= 65536
        if (y > 32767) y -= 65536
        if (z > 32767) z -= 65536
        return [x * MAG_QMC_SCALE, y * MAG_QMC_SCALE, z * MAG_QMC_SCALE]
    }

    // ───────── MAG: HMC5883L via master ─────────
    function hmcIdOk(): boolean {
        const ida = mpuSlv4Read(HMC, 0x0A) // 'H'
        const idb = mpuSlv4Read(HMC, 0x0B) // '4'
        const idc = mpuSlv4Read(HMC, 0x0C) // '3'
        return (ida[0] == 1 && idb[0] == 1 && idc[0] == 1 &&
            ida[2] == 0x48 && idb[2] == 0x34 && idc[2] == 0x33)
    }
    function hmcMasterInit(): boolean {
        if (!hmcIdOk()) return false
        if (!mpuSlv4Write(HMC, 0x00, 0x70)) return false // CRA
        basic.pause(5)
        if (!mpuSlv4Write(HMC, 0x01, 0x20)) return false // CRB
        basic.pause(5)
        if (!mpuSlv4Write(HMC, 0x02, 0x00)) return false // MODE=continuous
        basic.pause(10)
        slv0Enable(true, 6, HMC, 0x03) // X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
        return true
    }
    function hmcMasterReadAll(): number[] {
        const d = mpuExtReadBurst(6)
        let x = (d.getNumber(NumberFormat.UInt8BE, 0) << 8) | d.getNumber(NumberFormat.UInt8BE, 1)
        let z = (d.getNumber(NumberFormat.UInt8BE, 2) << 8) | d.getNumber(NumberFormat.UInt8BE, 3)
        let y = (d.getNumber(NumberFormat.UInt8BE, 4) << 8) | d.getNumber(NumberFormat.UInt8BE, 5)
        if (x > 32767) x -= 65536
        if (y > 32767) y -= 65536
        if (z > 32767) z -= 65536
        return [x * HMC_SCALE, y * HMC_SCALE, z * HMC_SCALE]
    }

    // ───────── BMP280 ─────────
    function probeBMP(): boolean {
        // Prøv 0x76 først (default), så 0x77
        for (let a of [0x76, 0x77]) {
            const id = safeRead8(a, 0xD0)
            if (id == 0x58 /* BMP280 */ || id == 0x60 /* BME280 */) {
                BMP = a
                return true
            }
        }
        return false
    }
    function safeRead8(a: number, r: number): number {
        let v = 0
        try { v = read8(a, r) } catch (e) { v = 0 }
        return v
    }
    function bmpConfigure() {
        // ctrl_meas: temp x2 (010), press x8 (100), mode normal (11) => 01010011 = 0x53
        write8(BMP, 0xF4, 0x53)
        // config: standby 500 ms (100), filter x4 (010), spi3w off (0) => 10010000 = 0x90
        write8(BMP, 0xF5, 0x90)
    }
    function bmpReadCoeffs() {
        dig_T1 = read16LEu(BMP, 0x88)
        dig_T2 = read16LEs(BMP, 0x8A)
        dig_T3 = read16LEs(BMP, 0x8C)
        dig_P1 = read16LEu(BMP, 0x8E)
        dig_P2 = read16LEs(BMP, 0x90)
        dig_P3 = read16LEs(BMP, 0x92)
        dig_P4 = read16LEs(BMP, 0x94)
        dig_P5 = read16LEs(BMP, 0x96)
        dig_P6 = read16LEs(BMP, 0x98)
        dig_P7 = read16LEs(BMP, 0x9A)
        dig_P8 = read16LEs(BMP, 0x9C)
        dig_P9 = read16LEs(BMP, 0x9E)
    }
    function read16LEu(a: number, r: number): number {
        pins.i2cWriteNumber(a, r, NumberFormat.UInt8BE, true)
        const lo = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        const hi = pins.i2cReadNumber(a, NumberFormat.UInt8BE)
        return (hi << 8) | lo
    }
    function read16LEs(a: number, r: number): number {
        const v = read16LEu(a, r)
        return v > 32767 ? v - 65536 : v
    }

    // ───────────────── Offentlige blokker (faste desimaler) ─────────────────

    //% block="akselerasjon %akse (g)"
    //% group="Måling"
    //% weight=100
    export function akselerasjon(akse: Akse): number {
        ensureInit()
        if (MPU == 0) return 0
        const r = akse == Akse.X ? 0x3B : akse == Akse.Y ? 0x3D : 0x3F
        const g = read16BE(MPU, r) / ACCEL_SCALE
        return Math.round(g * 100) / 100   // 2 desimaler
    }

    //% block="gyro %akse (°/s)"
    //% group="Måling"
    //% weight=99
    export function gyro(akse: Akse): number {
        ensureInit()
        if (MPU == 0) return 0
        const r = akse == Akse.X ? 0x43 : akse == Akse.Y ? 0x45 : 0x47
        let raw = read16BE(MPU, r)
        if (akse == Akse.Z) raw -= gyroOffsetZ
        const dps = raw / GYRO_SCALE
        return Math.round(dps * 10) / 10   // 1 desimal
    }

    //% block="kalibrer gyro (hold rolig)"
    //% group="Kalibrering"
    //% advanced=true
    //% weight=50
    export function kalibrerGyro(): void {
        ensureInit()
        if (MPU == 0) { gyroOffsetZ = 0; return }
        let sum = 0
        for (let i = 0; i < 60; i++) { sum += read16BE(MPU, 0x47); basic.pause(5) }
        gyroOffsetZ = Math.idiv(sum, 60)
    }

    function magScaledAll(): number[] {
        if (magType == MagType.AK_Master) return akMasterReadAll()
        if (magType == MagType.QMC_Master) return qmcMasterReadAll()
        if (magType == MagType.HMC_Master) return hmcMasterReadAll()
        return [0, 0, 0]
    }

    //% block="magnetfelt %akse (µT)"
    //% group="Måling"
    //% weight=98
    export function magnetfelt(akse: Akse): number {
        ensureInit()
        const [rx, ry, rz] = magScaledAll()
        const x = rx - _mOffX
        const y = ry - _mOffY
        const z = rz - _mOffZ
        if (_calRunning) _feedMinMax(x, y, z)
        const val = (akse == Akse.X ? x : akse == Akse.Y ? y : z)
        return Math.round(val * 10) / 10    // 1 desimal
    }

    //% block="magnetfelt total styrke (µT)"
    //% group="Orientering"
    //% weight=95
    export function magnetfeltTotal(): number {
        const x = magnetfelt(Akse.X)
        const y = magnetfelt(Akse.Y)
        const z = magnetfelt(Akse.Z)
        const b = Math.sqrt(x * x + y * y + z * z)
        return Math.round(b * 10) / 10      // 1 desimal
    }

    //% block="start magnetometer-kalibrering"
    //% group="Kalibrering"
    //% advanced=true
    export function startMagKal(): void {
        _calRunning = true
        const [x, y, z] = magScaledAll()
        _minX = x; _maxX = x
        _minY = y; _maxY = y
        _minZ = z; _maxZ = z
    }

    //% block="stopp magnetometer-kalibrering og lagre offset"
    //% group="Kalibrering"
    //% advanced=true
    export function stopMagKal(): void {
        _calRunning = false
        _mOffX = (_minX + _maxX) / 2
        _mOffY = (_minY + _maxY) / 2
        _mOffZ = (_minZ + _maxZ) / 2
    }

    //% block="sett magnetometer offset X %x Y %y Z %z (µT)"
    //% group="Kalibrering"
    //% advanced=true
    export function setMagOffset(x: number, y: number, z: number): void {
        _mOffX = x; _mOffY = y; _mOffZ = z
    }

    //% block="hent magnetometer offset (tekst)"
    //% group="Kalibrering"
    //% advanced=true
    export function getMagOffset(): string {
        return "offX=" + Math.round(_mOffX) + " offY=" + Math.round(_mOffY) + " offZ=" + Math.round(_mOffZ)
    }

    //% block="helning %h (grader)"
    //% group="Orientering"
    //% weight=90
    export function helning(h: Helning): number {
        const ax = akselerasjon(Akse.X) // 2 desimaler holder i trig
        const ay = akselerasjon(Akse.Y)
        const az = akselerasjon(Akse.Z)
        if (h == Helning.ForoverBakover) {
            const v = Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI
            return Math.round(v * 10) / 10  // 1 desimal
        }
        if (h == Helning.Sideveis) {
            const v = Math.atan2(ay, az) * 180 / Math.PI
            return Math.round(v * 10) / 10  // 1 desimal
        }
        // Retning (yaw) = gyro-integrasjon (guard første kall)
        const now = input.runningTime()
        if (lastTime == 0) { lastTime = now; return Math.round(yaw * 10) / 10 }
        const dt = (now - lastTime) / 1000
        lastTime = now
        yaw += gyro(Akse.Z) * dt
        return Math.round(yaw * 10) / 10     // 1 desimal
    }

    //% block="nullstill yaw"
    //% group="Kalibrering"
    //% advanced=true
    //% weight=45
    export function nullstillYaw(): void { yaw = 0; lastTime = input.runningTime() }

    //% block="kompassretning (grader)"
    //% group="Orientering"
    //% weight=88
    export function kompassretning(): number {
        const mx = magnetfelt(Akse.X)
        const my = magnetfelt(Akse.Y)
        const mz = magnetfelt(Akse.Z)
        const p = helning(Helning.ForoverBakover) * Math.PI / 180
        const r = helning(Helning.Sideveis) * Math.PI / 180
        const xh = mx * Math.cos(p) + mz * Math.sin(p)
        const yh = mx * Math.sin(r) * Math.sin(p) + my * Math.cos(r) - mz * Math.sin(r) * Math.cos(p)
        let hdg = Math.atan2(yh, xh) * 180 / Math.PI
        if (hdg < 0) hdg += 360
        return Math.round(hdg * 10) / 10     // 1 desimal
    }

    //% block="temperatur (°C)"
    //% group="Miljø"
    //% weight=70
    export function temperatur(): number {
        ensureInit()
        if (!bmpPresent) return 0
        let adc_T = read24(BMP, 0xFA) >> 4
        let var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11
        let tdiff = (adc_T >> 4) - dig_T1
        let tdiff2 = (tdiff * tdiff) >> 12
        let var2 = (tdiff2 * dig_T3) >> 14
        tFine = var1 + var2
        // Bosch: T(°C) = tFine/5120.0 = ((tFine*5+128)>>8)/100.0
        const c = ((tFine * 5 + 128) >> 8) / 100.0
        return Math.round(c * 10) / 10       // 1 desimal
    }

    //% block="trykk (Pa)"
    //% group="Miljø"
    //% weight=65
    export function trykk(): number {
        ensureInit()
        if (!bmpPresent) return 0

        // Sørg for oppdatert tFine (kaller temperatur())
        temperatur()

        // Les rå 20-bit trykk
        const adc_P = (read24(BMP, 0xF7) >> 4)

        // Bosch BMP280 float-kompensasjon
        let var1 = tFine / 2.0 - 64000.0
        let var2 = var1 * var1 * (dig_P6 as number) / 32768.0
        var2 = var2 + var1 * (dig_P5 as number) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4 as number) * 65536.0)
        var1 = ((dig_P3 as number) * var1 * var1 / 524288.0 + (dig_P2 as number) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1 as number)
        if (var1 == 0) return 0

        let p = 1048576.0 - adc_P
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9 as number) * p * p / 2147483648.0
        var2 = p * (dig_P8 as number) / 32768.0
        p = p + (var1 + var2 + (dig_P7 as number)) / 16.0

        return Math.round(p)                 // 0 desimaler (Pa)
    }

    // ───────── Avansert (debug/status) ─────────

    //% block="sensorstatus (tekst)"
    //% group="Avansert"
    //% advanced=true
    export function sensorStatus(): string {
        ensureInit()
        const mpuTxt = mpuWho ? ("0x" + toHex2(mpuWho)) : "—"
        const magTxt =
            magType == MagType.AK_Master ? "AK8963 via MPU-master" :
                magType == MagType.QMC_Master ? "QMC5883L via MPU-master" :
                    magType == MagType.HMC_Master ? "HMC5883L via MPU-master" : "none"
        const bmpTxt = bmpPresent ? ("BMP280 @0x" + toHex2(BMP)) : "—"
        return "MPU WHO=" + mpuTxt + "  MAG=" + magTxt + "  BMP=" + bmpTxt
    }

    //% block="debug dump (skriv til Serial)"
    //% group="Avansert"
    //% advanced=true
    export function debugDump(): void {
        ensureInit()
        if (MPU == 0) { serial.writeLine("Ingen MPU"); return }
        const user = read8(MPU, 0x6A)
        const intp = read8(MPU, 0x37)
        const pwr1 = read8(MPU, 0x6B)
        const mst = read8(MPU, 0x24)
        const dly = read8(MPU, 0x67)
        const slv0 = read8(MPU, 0x27)
        const slv4 = read8(MPU, 0x34)
        serial.writeLine("USER_CTRL=0x" + toHex2(user))
        serial.writeLine("INT_PIN_CFG=0x" + toHex2(intp))
        serial.writeLine("PWR_MGMT_1=0x" + toHex2(pwr1))
        serial.writeLine("I2C_MST_CTRL=0x" + toHex2(mst))
        serial.writeLine("I2C_MST_DELAY_CTRL=0x" + toHex2(dly))
        serial.writeLine("I2C_SLV0_CTRL=0x" + toHex2(slv0))
        serial.writeLine("I2C_SLV4_CTRL=0x" + toHex2(slv4))
    }

    //% block="reset MPU I2C-master (nødknep)"
    //% group="Avansert"
    //% advanced=true
    export function resetMaster(): void {
        ensureInit()
        if (MPU == 0) return
        write8(MPU, 0x6A, 0x22); basic.pause(5) // I2C_MST_RST + I2C_MST_EN
        write8(MPU, 0x6A, 0x20); basic.pause(2) // I2C_MST_EN
        write8(MPU, 0x24, _mstCtrl & 0x7F); basic.pause(2)
        write8(MPU, 0x67, 0x81); basic.pause(2)
        if (magType == MagType.AK_Master) slv0Enable(true, 8, AK, 0x02)
        else if (magType == MagType.QMC_Master) slv0Enable(true, 6, QMC, 0x00)
        else if (magType == MagType.HMC_Master) slv0Enable(true, 6, HMC, 0x03)
    }

    //% block="dump EXT_SENS_DATA (lengde %len) til Serial"
    //% group="Avansert"
    //% advanced=true
    //% len.min=1 len.max=24 len.defl=8
    export function dumpExt(len: number = 8): void {
        ensureInit()
        const b = mpuExtReadBurst(len)
        let line = "EXT RAW:"
        for (let i = 0; i < len; i++) line += " " + toHex2(b.getNumber(NumberFormat.UInt8BE, i))
        serial.writeLine(line)
    }

    //% block="BMP debug (skriv til Serial)"
    //% group="Avansert"
    //% advanced=true
    export function bmpDebug(): void {
        if (!bmpPresent) { serial.writeLine("BMP: ikke tilstede"); return }
        const id = safeRead8(BMP, 0xD0)
        serial.writeLine("BMP addr=0x" + toHex2(BMP) + " CHIPID=0x" + toHex2(id))
        const rawT = read24(BMP, 0xFA) >> 4
        const rawP = read24(BMP, 0xF7) >> 4
        serial.writeLine("ADC T(raw20)=" + rawT + "  P(raw20)=" + rawP)
    }

    //% block="sett BMP-adresse til %addr"
    //% group="Avansert"
    //% advanced=true
    //% addr.defl=0x76
    export function settBMPAdresse(addr: number): void {
        if (addr != 0x76 && addr != 0x77) return
        const id = safeRead8(addr, 0xD0)
        if (id == 0x58 || id == 0x60) {
            BMP = addr
            bmpPresent = true
            bmpConfigure()
            bmpReadCoeffs()
        }
    }
}