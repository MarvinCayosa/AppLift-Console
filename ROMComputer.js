/**
 * ROMComputer - Range of Motion calculator
 * 
 * Supports two ROM types (same calibration flow, different computation):
 * 
 * - ANGLE ROM (dumbbell exercises):
 *   Uses quaternion angular displacement. Sensor rotates with the dumbbell,
 *   so ROM = angular range (degrees) of the primary rotation axis.
 * 
 * - STROKE ROM (barbell / weight stack exercises):
 *   Sensor on the bar or flat on the weight stack — constrained to 1D vertical motion.
 *   1. Detect gravity axis: at rest, whichever accel axis reads ~9.8 m/s² is vertical.
 *   2. Subtract gravity from that axis → linear acceleration.
 *   3. Double-integrate per rep: accel → velocity → displacement (cm).
 *   4. ZUPT (Zero-Velocity Update) at top/bottom of each rep kills drift.
 *   ROM = peak-to-trough vertical displacement within a rep.
 * 
 * Calibration flow (identical for both types):
 * 1. User holds starting position for 3 seconds (baseline capture)
 * 2. User performs 3 full-ROM reps
 * 3. Average ROM of 3 reps becomes the target
 * 4. Each subsequent rep shows fulfillment % against target
 */

// Exercise code → ROM type mapping
// Dumbbell exercises (0,1) = angle, everything else = stroke
const EXERCISE_ROM_TYPE = {
  0: 'angle', // Concentration Curls
  1: 'angle', // Overhead Extension
  2: 'stroke', // Bench Press
  3: 'stroke', // Back Squats
  4: 'stroke', // Lateral Pulldown
  5: 'stroke', // Seated Leg Extension
};

// Equipment name → exercise code mapping for lookup
const EQUIPMENT_EXERCISE_MAP = {
  'dumbbell': { 'concentration curls': 0, 'overhead extension': 1 },
  'barbell': { 'bench press': 2, 'back squats': 3 },
  'weight stack': { 'lateral pulldown': 4, 'seated leg extension': 5, 'leg extension': 5 },
};

export class ROMComputer {
  constructor() {
    this.exerciseType = 0;
    this.repROMs = [];
    this.currentRepData = [];
    
    // Angle ROM state (quaternion-based)
    this.baselineQuat = null;       // {w,x,y,z} at neutral
    this.primaryAxis = null;        // auto-detected: 'roll','pitch','yaw'
    this.baselineAngle = null;      // Euler baseline
    
    // Stroke ROM state — quaternion gravity removal + trapezoidal integration + ZUPT
    this.velocity = 0;
    this.displacement = 0;
    this.lastTimestamp = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;
    this.baselineGravity = null;    // calibrated gravity vector {x,y,z} from baseline hold
    this.gravityMag = 9.81;         // calibrated gravity magnitude
    this.gyroBias = {x:0,y:0,z:0}; // calibrated gyro offset (native units)
    this.gyroInRadians = false;     // auto-detected: true if ESP32 outputs rad/s
    this.stillCounter = 0;          // consecutive near-zero samples
    this.NOISE_FLOOR = 0.06;        // m/s² — acceleration dead-zone (tighter)
    this.ZUPT_THRESHOLD = 0.12;     // m/s² — accel rest detection
    this.GYRO_STILL_RAD = 0.06;     // rad/s — gyro rest detection (~3.4°/s)
    this.ZUPT_SAMPLES = 2;          // consecutive still samples to trigger ZUPT (faster response)
    this.ZUPT_DECAY = 0.03;         // very aggressive velocity decay at rest
    this.MAX_DISPLACEMENT = 2.0;    // meters (200 cm) — hard clamp for safety
    this.MAX_VELOCITY = 2.0;        // m/s — velocity clamp to prevent runaway
    this.prevVertAccel = 0;         // for EMA smoothing
    this.DEG2RAD = Math.PI / 180;
    this.DEBUG_MODE = false;        // set to true for extra logging
    
    // Target ROM & Calibration
    this.targetROM = null;          // target ROM from calibration reps (degrees or cm)
    this.isCalibrationRep = false;  // true during calibration rep
    this.romCalibrated = false;     // true after target ROM is set
    this.calibrationROMs = [];      // ROMs from calibration reps (for averaging 3 reps)
    
    // Within-rep tracking (position-independent: max - min within the rep)
    this.repMinAngle = Infinity;    // tracks min angle within current rep
    this.repMaxAngle = -Infinity;   // tracks max angle within current rep
    
    // Live tracking
    this.liveAngleDeg = 0;          // current angle from baseline (degrees)
    this.liveDisplacementCm = 0;    // current displacement (cm)
    this.liveVelocity = 0;
    this.liveFulfillment = 0;       // current rep ROM / targetROM * 100
    this.liveRepROM = 0;            // current rep ROM (max-min so far)
    this.sampleHistory = [];        // last N samples for live chart
    this.maxHistorySize = 200;      // ~10s at 20Hz
  }
  
  // ---- ROM type detection ----
  getROMType(exerciseCode) {
    return EXERCISE_ROM_TYPE[exerciseCode] || 'angle';
  }
  
  /**
   * Set exercise from equipment/exercise name strings
   */
  setExerciseFromNames(equipmentName, exerciseName) {
    if (!equipmentName || !exerciseName) return;
    const eqKey = equipmentName.toLowerCase().trim();
    const exKey = exerciseName.toLowerCase().trim();
    
    for (const [eq, exercises] of Object.entries(EQUIPMENT_EXERCISE_MAP)) {
      if (eqKey.includes(eq)) {
        for (const [ex, code] of Object.entries(exercises)) {
          if (exKey.includes(ex) || ex.includes(exKey)) {
            this.setExercise(code);
            return;
          }
        }
      }
    }
    // Default to angle ROM for dumbbell
    if (eqKey.includes('dumbbell')) {
      this.setExercise(0);
    }
  }
  
  setExercise(exerciseCode) {
    this.exerciseType = exerciseCode;
    this.reset();
  }
  
  // ---- Quaternion math helpers (from index.html) ----
  quatConjugate(q) {
    return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
  }
  
  quatMultiply(a, b) {
    return {
      w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
      x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
      y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
      z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
  }
  
  quatToEuler(q) {
    const sinr = 2 * (q.w * q.x + q.y * q.z);
    const cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr, cosr) * (180 / Math.PI);
    
    let pitch;
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1) {
      pitch = Math.sign(sinp) * 90;
    } else {
      pitch = Math.asin(sinp) * (180 / Math.PI);
    }
    
    const siny = 2 * (q.w * q.z + q.x * q.y);
    const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny, cosy) * (180 / Math.PI);
    
    return { roll, pitch, yaw };
  }
  
  // Angle between two quaternions in degrees
  quatAngleDeg(q1, q2) {
    const dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
    const clamped = Math.min(1, Math.max(-1, Math.abs(dot)));
    return 2 * Math.acos(clamped) * (180 / Math.PI);
  }
  
  rotateVector(v, q) {
    const p = { w: 0, x: v.x, y: v.y, z: v.z };
    const r = this.quatMultiply(this.quatMultiply(q, p), this.quatConjugate(q));
    return { x: r.x, y: r.y, z: r.z };
  }
  
  // ---- Main sample entry ----
  addSample(data) {
    const { roll, pitch, yaw, qw, qx, qy, qz, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, timestamp } = data;
    const romType = this.getROMType(this.exerciseType);
    const q = { w: qw || 0, x: qx || 0, y: qy || 0, z: qz || 0 };
    
    // Skip if no valid quaternion data
    if (qw === undefined && qx === undefined) return;
    
    if (romType === 'angle') {
      this.addAngleSample(roll || 0, pitch || 0, yaw || 0, q, timestamp);
    } else {
      this.addStrokeSample(q, accelX || 0, accelY || 0, accelZ || 0, gyroX || 0, gyroY || 0, gyroZ || 0, timestamp);
    }
  }
  
  // ---- ANGLE ROM (Quaternion-based) - from index.html ----
  addAngleSample(roll, pitch, yaw, q, timestamp) {
    // Set baseline on first sample (auto)
    if (this.baselineQuat === null) {
      this.baselineQuat = { ...q };
      this.baselineAngle = { roll, pitch, yaw };
    }
    
    // Total angular displacement from baseline (single scalar, degrees)
    this.liveAngleDeg = this.quatAngleDeg(this.baselineQuat, q);
    
    // Per-axis delta for auto-axis detection
    const deltaRoll = roll - this.baselineAngle.roll;
    const deltaPitch = pitch - this.baselineAngle.pitch;
    const deltaYaw = yaw - this.baselineAngle.yaw;
    
    // Track within-rep range using primary axis if known
    let trackValue = this.liveAngleDeg;
    if (this.primaryAxis) {
      const axisMap = { roll: deltaRoll, pitch: deltaPitch, yaw: deltaYaw };
      trackValue = axisMap[this.primaryAxis];
    }
    this.repMinAngle = Math.min(this.repMinAngle, trackValue);
    this.repMaxAngle = Math.max(this.repMaxAngle, trackValue);
    this.liveRepROM = this.repMaxAngle - this.repMinAngle;
    
    // Update fulfillment
    if (this.targetROM && this.targetROM > 0) {
      this.liveFulfillment = Math.min(150, (this.liveRepROM / this.targetROM) * 100);
    }
    
    this.currentRepData.push({
      roll: deltaRoll,
      pitch: deltaPitch,
      yaw: deltaYaw,
      totalAngle: this.liveAngleDeg,
      timestamp
    });
    
    // Update live chart history
    this.sampleHistory.push({ t: timestamp, v: this.liveRepROM });
    if (this.sampleHistory.length > this.maxHistorySize) this.sampleHistory.shift();
  }
  
  // ---- STROKE ROM (vertical displacement via quaternion gravity removal) ----
  // Improved approach using ESP32's onboard quaternion for gravity removal:
  // 1. Rotate measured accel to world frame using quaternion
  // 2. Subtract calibrated gravity magnitude from world-Z
  // 3. ZUPT when still (combined accel+gyro detection)
  // 4. Trapezoidal integration: accel → velocity → displacement
  addStrokeSample(q, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, timestamp) {
    // === First sample: initialize gravity magnitude ===
    if (this.baselineGravity === null) {
      this.baselineGravity = { x: accelX, y: accelY, z: accelZ };
      this.gravityMag = Math.sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
      this.lastTimestamp = timestamp;
      console.log(`🔍 [ROM DEBUG] Gravity mag=${this.gravityMag.toFixed(4)} from first sample`);
      return;
    }
    
    if (this.lastTimestamp === 0) { this.lastTimestamp = timestamp; return; }
    const dt = (timestamp - this.lastTimestamp) / 1000;
    if (dt <= 0 || dt >= 0.5) { this.lastTimestamp = timestamp; return; }
    this.lastTimestamp = timestamp;
    
    // === STEP 1: Quaternion-based gravity removal ===
    // Rotate measured accel from sensor frame to world frame
    const accelWorld = this.rotateVector({ x: accelX, y: accelY, z: accelZ }, q);
    // Vertical linear acceleration = world-Z component minus calibrated gravity
    const rawVertAccel = accelWorld.z - this.gravityMag;
    
    // Light EMA smoothing (reduces noise spikes, preserves motion signal)
    const vertAccel = 0.25 * this.prevVertAccel + 0.75 * rawVertAccel;
    this.prevVertAccel = vertAccel;
    
    // === STEP 2: Dead-zone (sensor noise) ===
    const accelInput = Math.abs(vertAccel) < this.NOISE_FLOOR ? 0 : vertAccel;
    
    // === STEP 3: ZUPT — combined accel + gyro stillness detection ===
    const accelMagDev = Math.abs(Math.sqrt(accelX*accelX+accelY*accelY+accelZ*accelZ) - this.gravityMag);
    const gyroMagRaw = Math.sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
    // Convert to rad/s for comparison if gyro is in deg/s
    const gyroMagRad = this.gyroInRadians ? gyroMagRaw : gyroMagRaw * this.DEG2RAD;
    const isStill = accelMagDev < this.ZUPT_THRESHOLD && gyroMagRad < this.GYRO_STILL_RAD;
    
    if (isStill) {
      this.stillCounter++;
    } else {
      this.stillCounter = 0;
    }
    
    if (this.stillCounter >= this.ZUPT_SAMPLES) {
      const oldVel = this.velocity;
      this.velocity *= this.ZUPT_DECAY;
      if (Math.abs(this.velocity) < 0.001) this.velocity = 0;
      if (this.DEBUG_MODE && oldVel !== 0 && this.velocity === 0) {
        console.log(`🛑 [ROM DEBUG] ZUPT triggered - velocity zeroed (was ${oldVel.toFixed(4)})`);
      }
    }
    
    // === STEP 4: Trapezoidal integration ===
    let newVelocity = this.velocity + accelInput * dt;
    // Clamp velocity to prevent runaway integration drift
    newVelocity = Math.max(-this.MAX_VELOCITY, Math.min(this.MAX_VELOCITY, newVelocity));
    this.displacement += (this.velocity + newVelocity) / 2 * dt;
    this.velocity = newVelocity;
    
    this.displacement = Math.max(-this.MAX_DISPLACEMENT, Math.min(this.MAX_DISPLACEMENT, this.displacement));
    
    if (this.DEBUG_MODE && this.currentRepData.length % 10 === 0) {
      console.log(`📏 [ROM] VertAccel:${vertAccel.toFixed(3)} Vel:${this.velocity.toFixed(4)} Disp:${(this.displacement*100).toFixed(1)}cm Still:${this.stillCounter}`);
    }
    
    this.peakDisplacement = Math.max(this.peakDisplacement, this.displacement);
    this.minDisplacement = Math.min(this.minDisplacement, this.displacement);
    
    // Convert to cm for display
    this.liveDisplacementCm = this.displacement * 100;
    this.liveVelocity = this.velocity;
    
    // Track within-rep min/max displacement (cm) for ROM = peak-to-trough
    this.repMinAngle = Math.min(this.repMinAngle, this.liveDisplacementCm);
    this.repMaxAngle = Math.max(this.repMaxAngle, this.liveDisplacementCm);
    this.liveRepROM = this.repMaxAngle - this.repMinAngle;
    
    // Update fulfillment %
    if (this.targetROM && this.targetROM > 0) {
      this.liveFulfillment = Math.min(150, (this.liveRepROM / this.targetROM) * 100);
    }
    
    // Store RAW sensor data for retrospective correction (retroCorrect)
    this.currentRepData.push({
      // Raw sensor data (needed for retroCorrect)
      ax: accelX,
      ay: accelY,
      az: accelZ,
      gx: gyroX,
      gy: gyroY,
      gz: gyroZ,
      qw: q.w,
      qx: q.x,
      qy: q.y,
      qz: q.z,
      ts: timestamp,
      // Live integration (for display only)
      displacement: this.liveDisplacementCm,
      velocity: this.velocity,
      vertAccel: vertAccel
    });
    
    this.sampleHistory.push({ t: timestamp, v: this.liveRepROM });
    if (this.sampleHistory.length > this.maxHistorySize) this.sampleHistory.shift();
  }
  
  // ---- Start calibration rep ----
  startCalibrationRep() {
    this.isCalibrationRep = true;
    this.currentRepData = [];
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    this.liveFulfillment = 0;
    // Reset stroke state for clean integration
    this.velocity = 0;
    this.displacement = 0;
    this.stillCounter = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;    this.prevVertAccel = 0;    console.log('[ROMComputer] Calibration rep started — do one full-range rep');
  }
  
  // ---- Finish calibration rep and return ROM value ----
  finishCalibrationRep() {
    if (this.currentRepData.length < 5) {
      console.log('[ROMComputer] Not enough data for calibration rep');
      this.isCalibrationRep = false;
      return null;
    }
    
    const romType = this.getROMType(this.exerciseType);
    let romValue;
    
    if (romType === 'angle') {
      romValue = this.computeAngleROM();
      // For angle ROM, also use within-rep tracking as alternative (take max)
      const repRangeROM = this.repMaxAngle - this.repMinAngle;
      romValue = Math.max(romValue, repRangeROM);
    } else {
      // For stroke ROM, use ONLY retroCorrect - do NOT override with live min/max
      // The live min/max (repMaxAngle/repMinAngle) drifts significantly;
      // retroCorrect uses forward-backward integration which cancels drift.
      romValue = this.computeStrokeROM();
    }
    
    // Clamp
    romValue = romType === 'angle' ? Math.min(romValue, 360) : Math.min(romValue, 300);
    
    this.isCalibrationRep = false;
    
    // Reset for next rep
    this.currentRepData = [];
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    
    // Reset stroke state
    if (romType === 'stroke') {
      this.velocity = 0;
      this.displacement = 0;
      this.stillCounter = 0;
      this.peakDisplacement = 0;
      this.minDisplacement = 0;
      this.prevVertAccel = 0;
    }
    
    const unit = romType === 'angle' ? '°' : ' cm';
    console.log(`[ROMComputer] Calibration rep ROM: ${romValue.toFixed(1)}${unit}`);
    return romValue;
  }
  
  /**
   * Set the target ROM from calibration (average of 3 reps)
   */
  setTargetFromCalibration(romValues) {
    if (!romValues || romValues.length === 0) return null;
    
    const avg = romValues.reduce((a, b) => a + b, 0) / romValues.length;
    this.targetROM = avg;
    this.romCalibrated = true;
    this.calibrationROMs = [...romValues];
    
    const romType = this.getROMType(this.exerciseType);
    const unit = romType === 'angle' ? '°' : ' cm';
    console.log(`[ROMComputer] Target ROM set from ${romValues.length} reps: ${avg.toFixed(1)}${unit}`);
    
    return avg;
  }
  
  // ---- Rep completion (called by RepCounter) ----
  completeRep() {
    if (this.currentRepData.length < 3) return null;
    
    const romType = this.getROMType(this.exerciseType);
    let romValue = 0;
    
    if (romType === 'angle') {
      romValue = this.computeAngleROM();
      // For angle ROM, also use within-rep (max-min) tracking
      const repRangeROM = this.repMaxAngle - this.repMinAngle;
      if (isFinite(repRangeROM)) {
        romValue = Math.max(romValue, repRangeROM);
      }
    } else {
      // For stroke ROM, use ONLY retroCorrect (live min/max drifts)
      romValue = this.computeStrokeROM();
    }
    
    // Clamp unrealistic values
    romValue = romType === 'angle' ? Math.min(romValue, 360) : Math.min(romValue, 200);
    
    const repROM = {
      repIndex: this.repROMs.length + 1,
      romValue: romValue,
      romType: romType,
      unit: romType === 'angle' ? 'deg' : 'cm',
      fulfillment: this.targetROM ? Math.min(150, (romValue / this.targetROM) * 100) : null
    };
    this.repROMs.push(repROM);
    
    // Zero-velocity reset at rep boundary for stroke
    if (romType === 'stroke') {
      this.velocity = 0;
      this.displacement = 0;
      this.peakDisplacement = 0;
      this.minDisplacement = 0;
      this.stillCounter = 0;
      this.prevVertAccel = 0;
    }
    this.currentRepData = [];
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    this.liveFulfillment = 0;
    
    return repROM;
  }
  
  computeAngleROM() {
    if (this.currentRepData.length < 3) return 0;
    
    // Option 1: Use total quaternion angular displacement
    const totalAngles = this.currentRepData.map(d => d.totalAngle);
    const quatROM = Math.max(...totalAngles) - Math.min(...totalAngles);
    
    // Option 2: Auto-detect primary Euler axis
    if (this.primaryAxis === null) {
      const ranges = {
        roll: Math.max(...this.currentRepData.map(d => d.roll)) - Math.min(...this.currentRepData.map(d => d.roll)),
        pitch: Math.max(...this.currentRepData.map(d => d.pitch)) - Math.min(...this.currentRepData.map(d => d.pitch)),
        yaw: Math.max(...this.currentRepData.map(d => d.yaw)) - Math.min(...this.currentRepData.map(d => d.yaw))
      };
      this.primaryAxis = Object.entries(ranges).sort((a, b) => b[1] - a[1])[0][0];
      console.log(`[ROMComputer] Auto-detected primary axis: ${this.primaryAxis} (range: ${ranges[this.primaryAxis].toFixed(1)}°)`);
    }
    
    const axisValues = this.currentRepData.map(d => d[this.primaryAxis]);
    const axisROM = Math.max(...axisValues) - Math.min(...axisValues);
    
    return Math.max(quatROM, axisROM);
  }
  
  computeStrokeROM() {
    if (this.currentRepData.length < 3) return 0;
    
    // Use retrospective forward-backward integration for accurate ROM
    // This eliminates accumulated drift that live integration suffers from
    const corrected = this.retroCorrect(this.currentRepData);
    
    if (corrected && corrected.rom > 0) {
      console.log(`📏 [ROM] RetroCorrect: ROM=${corrected.rom.toFixed(1)}cm peak=${corrected.peak.toFixed(1)}cm`);
      return corrected.rom;
    }
    
    // Fallback to live integration if retroCorrect fails
    const disp = this.currentRepData.map(d => d.displacement);
    return Math.max(...disp) - Math.min(...disp);
  }
  
  // ========== RETROSPECTIVE CORRECTION (Forward-Backward Integration) ==========
  // After a rep ends, re-process all samples with:
  // 1. Quaternion-based vertical acceleration extraction
  // 2. Acceleration bias estimation & removal from rest segments
  // 3. Light smoothing to reduce noise amplification
  // 4. Forward-backward velocity integration (drift cancellation)
  // 5. Velocity detrending for complete reps
  // 6. Rest-segment velocity zeroing (gyro-assisted)
  // 7. Position integration with linear detrending
  // This approach is speed-invariant and eliminates accumulated drift.
  retroCorrect(samples, isComplete = true) {
    const g = this.gravityMag;
    const n = samples.length;
    if (n < 5) return null;
    
    // ====== STEP 1: Extract raw vertical acceleration ======
    const rawAcc = new Float64Array(n);
    const dts = new Float64Array(n);
    
    for (let i = 0; i < n; i++) {
      const s = samples[i];
      if (i > 0) {
        const dt = (s.ts - samples[i - 1].ts) / 1000;
        dts[i] = (dt > 0 && dt < 0.5) ? dt : 0;
      }
      const q = { w: s.qw, x: s.qx, y: s.qy, z: s.qz };
      const aw = this.rotateVector({ x: s.ax, y: s.ay, z: s.az }, q);
      rawAcc[i] = aw.z - g;
    }
    
    // ====== STEP 2: Detect rest/still segments (accel + gyro) ======
    const STILL_ACCEL = 0.25;  // m/s² (relaxed for better rest detection with hand vibration)
    const STILL_GYRO = 0.10;   // rad/s (relaxed to catch more rest samples)
    const isStill = new Uint8Array(n);
    
    for (let i = 0; i < n; i++) {
      const s = samples[i];
      const aMagDev = Math.abs(Math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2) - g);
      const gMag = Math.sqrt(s.gx ** 2 + s.gy ** 2 + s.gz ** 2);
      const gMagRad = this.gyroInRadians ? gMag : gMag * this.DEG2RAD;
      isStill[i] = (aMagDev < STILL_ACCEL && gMagRad < STILL_GYRO) ? 1 : 0;
    }
    
    const restSegs = [];
    let rStart = -1;
    for (let i = 0; i < n; i++) {
      if (isStill[i]) {
        if (rStart < 0) rStart = i;
      } else {
        if (rStart >= 0 && (i - rStart) >= 2) restSegs.push([rStart, i - 1]);
        rStart = -1;
      }
    }
    if (rStart >= 0 && (n - rStart) >= 2) restSegs.push([rStart, n - 1]);
    
    // ====== STEP 3: Estimate & remove acceleration bias ======
    let biasSum = 0, biasN = 0;
    restSegs.forEach(([s, e]) => {
      for (let i = s; i <= e; i++) { biasSum += rawAcc[i]; biasN++; }
    });
    let accBias;
    if (biasN > 5) {
      accBias = biasSum / biasN;
    } else {
      let fullSum = 0;
      for (let i = 0; i < n; i++) fullSum += rawAcc[i];
      accBias = fullSum / n;
    }
    
    // ====== STEP 4: Bias removal + light smoothing + noise floor ======
    const acc = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let a = rawAcc[i] - accBias;
      if (n > 10 && i > 0 && i < n - 1) {
        a = ((rawAcc[i - 1] - accBias) + 2 * a + (rawAcc[i + 1] - accBias)) / 4;
      }
      if (isStill[i]) {
        a = 0;
      } else if (Math.abs(a) < 0.05) {
        a = 0;
      }
      acc[i] = a;
    }
    
    // ====== STEP 5: Forward-backward velocity integration ======
    const vFwd = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      vFwd[i] = vFwd[i - 1] + (acc[i - 1] + acc[i]) / 2 * dts[i];
    }
    const vBwd = new Float64Array(n);
    for (let i = n - 2; i >= 0; i--) {
      vBwd[i] = vBwd[i + 1] - (acc[i] + acc[i + 1]) / 2 * dts[i + 1];
    }
    const vel = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      vel[i] = (vFwd[i] + vBwd[i]) / 2;
    }
    
    // ====== STEP 5b: Velocity detrending (complete reps only) ======
    if (isComplete && n > 10) {
      const edgeSamples = Math.min(5, Math.floor(n / 4));
      let vStartSum = 0, vEndSum = 0;
      for (let i = 0; i < edgeSamples; i++) vStartSum += vel[i];
      for (let i = n - edgeSamples; i < n; i++) vEndSum += vel[i];
      const vStart = vStartSum / edgeSamples;
      const vEnd = vEndSum / edgeSamples;
      const vSlope = (vEnd - vStart) / (n - 1);
      for (let i = 0; i < n; i++) {
        vel[i] -= (vStart + vSlope * i);
      }
    }
    
    // Force velocity to 0 in rest segments
    restSegs.forEach(([s, e]) => {
      for (let i = s; i <= e; i++) vel[i] = 0;
    });
    
    // ====== STEP 6: Integrate position ======
    const pos = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      pos[i] = pos[i - 1] + (vel[i - 1] + vel[i]) / 2 * dts[i];
    }
    
    // ====== STEP 7: Position detrending ======
    if (restSegs.length >= 2) {
      const fMid = Math.round((restSegs[0][0] + restSegs[0][1]) / 2);
      const lMid = Math.round((restSegs[restSegs.length - 1][0] + restSegs[restSegs.length - 1][1]) / 2);
      if (lMid > fMid) {
        const p0 = pos[fMid], p1 = pos[lMid];
        const slope = (p1 - p0) / (lMid - fMid);
        for (let i = 0; i < n; i++) {
          pos[i] -= (p0 + slope * (i - fMid));
        }
      }
    } else if (restSegs.length === 1) {
      const mid = Math.round((restSegs[0][0] + restSegs[0][1]) / 2);
      if (isComplete) {
        const restIsNearStart = mid < n / 2;
        const anchor1 = restIsNearStart ? mid : 0;
        const anchor2 = restIsNearStart ? n - 1 : mid;
        const p0 = pos[anchor1], p1 = pos[anchor2];
        const span = anchor2 - anchor1;
        if (span > 0) {
          const slope = (p1 - p0) / span;
          for (let i = 0; i < n; i++) {
            pos[i] -= (p0 + slope * (i - anchor1));
          }
        }
      } else {
        const offset = pos[mid];
        for (let i = 0; i < n; i++) pos[i] -= offset;
      }
    } else {
      console.log('⚠️ [RetroCorrect] No rest segments detected — using linear detrend fallback');
      const p0 = pos[0], p1 = pos[n - 1];
      const slope = (p1 - p0) / (n - 1);
      for (let i = 0; i < n; i++) {
        pos[i] -= (p0 + slope * i);
      }
    }
    
    // ====== STEP 8: Extract peak & ROM ======
    let maxD = -Infinity, minD = Infinity;
    for (let i = 0; i < n; i++) {
      if (pos[i] > maxD) maxD = pos[i];
      if (pos[i] < minD) minD = pos[i];
    }
    
    const peakAbs = Math.max(Math.abs(maxD), Math.abs(minD));
    const rom = maxD - minD;
    
    if (this.DEBUG_MODE) {
      console.log(`📏 [RetroCorrect] bias=${(accBias).toFixed(4)}m/s² restSegs=${restSegs.length} peak=${(peakAbs*100).toFixed(1)}cm rom=${(rom*100).toFixed(1)}cm complete=${isComplete}`);
    }
    
    return {
      peak: peakAbs * 100,
      rom: rom * 100,
      maxDisp: maxD * 100,
      minDisp: minD * 100
    };
  }
  
  getSetStats() {
    if (this.repROMs.length === 0) return null;
    const values = this.repROMs.map(r => r.romValue);
    const avg = values.reduce((a, b) => a + b, 0) / values.length;
    const max = Math.max(...values);
    const stdDev = Math.sqrt(values.reduce((sum, v) => sum + Math.pow(v - avg, 2), 0) / values.length);
    const consistency = avg > 0 ? ((1 - stdDev / avg) * 100) : 100;
    
    const fulfillments = this.repROMs.filter(r => r.fulfillment !== null).map(r => r.fulfillment);
    const avgFulfillment = fulfillments.length > 0 ? fulfillments.reduce((a, b) => a + b, 0) / fulfillments.length : null;
    
    return {
      avgROM: avg,
      maxROM: max,
      romConsistencyPercent: Math.max(0, Math.min(100, consistency)),
      romType: this.getROMType(this.exerciseType),
      unit: this.getROMType(this.exerciseType) === 'angle' ? 'deg' : 'cm',
      repCount: this.repROMs.length,
      targetROM: this.targetROM,
      avgFulfillment: avgFulfillment
    };
  }
  
  getROMForRep(repNumber) {
    const rom = this.repROMs.find(r => r.repIndex === repNumber);
    return rom ? rom.romValue : 0;
  }
  
  getROMLabel() {
    const romType = this.getROMType(this.exerciseType);
    if (romType === 'angle') return 'Equipment ROM';
    if (this.exerciseType <= 3) return 'Vertical Displacement';
    return 'Stack Displacement';
  }
  
  getUnit() {
    return this.getROMType(this.exerciseType) === 'angle' ? '°' : ' cm';
  }
  
  /**
   * Enable debug logging and testing modes
   */
  enableDebugMode() {
    this.DEBUG_MODE = true;
    console.log('🔧 [ROM DEBUG] Debug mode enabled');
  }
  
  disableDebugMode() {
    this.DEBUG_MODE = false;
    console.log('🔧 [ROM DEBUG] Debug mode disabled');
  }
  
  bypassHighPassFilter(bypass = true) {
    // No-op: HP filter removed in favor of quaternion gravity removal + adaptive ZUPT
    console.log(`🔧 [ROM DEBUG] HP filter no longer used (quaternion gravity removal handles drift)`);
  }
  
  /**
   * Force reset displacement to zero (when user returns to starting position)
   * Call this when you know the sensor is back at the baseline position
   */
  resetDisplacementToZero() {
    this.displacement = 0;
    this.velocity = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;
    this.stillCounter = 0;    this.prevVertAccel = 0;    console.log('🔄 [ROM DEBUG] Displacement manually reset to zero');
  }
  
  /**
   * Set baseline from calibration samples (hold-still calibration)
   * Averages quaternion/accel samples for accurate baseline
   */
  setBaselineFromSamples(samples) {
    if (!samples || samples.length < 5) return false;
    
    let sumW = 0, sumX = 0, sumY = 0, sumZ = 0;
    let sumRoll = 0, sumPitch = 0, sumYaw = 0;
    let sumAx = 0, sumAy = 0, sumAz = 0;
    
    // Ensure consistent hemisphere
    const ref = samples[0];
    samples.forEach(s => {
      const dot = ref.qw*s.qw + ref.qx*s.qx + ref.qy*s.qy + ref.qz*s.qz;
      const sign = dot < 0 ? -1 : 1;
      sumW += s.qw * sign;
      sumX += s.qx * sign;
      sumY += s.qy * sign;
      sumZ += s.qz * sign;
      sumRoll += s.roll || 0;
      sumPitch += s.pitch || 0;
      sumYaw += s.yaw || 0;
      sumAx += s.accelX || 0;
      sumAy += s.accelY || 0;
      sumAz += s.accelZ || 0;
    });
    
    const n = samples.length;
    const avgQ = { w: sumW/n, x: sumX/n, y: sumY/n, z: sumZ/n };
    const norm = Math.sqrt(avgQ.w**2 + avgQ.x**2 + avgQ.y**2 + avgQ.z**2);
    avgQ.w /= norm; avgQ.x /= norm; avgQ.y /= norm; avgQ.z /= norm;
    
    this.baselineQuat = avgQ;
    this.baselineAngle = { roll: sumRoll/n, pitch: sumPitch/n, yaw: sumYaw/n };
    this.baselineGravity = { x: sumAx/n, y: sumAy/n, z: sumAz/n };
    
    // Set gravity magnitude for quaternion-based approach
    this.gravityMag = Math.sqrt(this.baselineGravity.x**2 + this.baselineGravity.y**2 + this.baselineGravity.z**2);
    
    // Compute gyro bias from rest samples (if gyro data available)
    let sumGx=0, sumGy=0, sumGz=0;
    samples.forEach(s => {
      sumGx += s.gyroX || 0;
      sumGy += s.gyroY || 0;
      sumGz += s.gyroZ || 0;
    });
    this.gyroBias = { x: sumGx/n, y: sumGy/n, z: sumGz/n };
    
    // Auto-detect gyro units: if bias magnitude is small (<0.3), it's likely radians/sec
    const gyroBiasMag = Math.abs(this.gyroBias.x) + Math.abs(this.gyroBias.y) + Math.abs(this.gyroBias.z);
    this.gyroInRadians = gyroBiasMag < 0.3;
    
    const grav = this.baselineGravity;
    const gyroUnits = this.gyroInRadians ? 'rad/s' : 'deg/s';
    console.log(`🔍 [ROM DEBUG] Baseline gravity: [${grav.x.toFixed(3)}, ${grav.y.toFixed(3)}, ${grav.z.toFixed(3)}] mag=${this.gravityMag.toFixed(4)}`);
    console.log(`🎯 [ROM DEBUG] Gyro bias: [${this.gyroBias.x.toFixed(4)}, ${this.gyroBias.y.toFixed(4)}, ${this.gyroBias.z.toFixed(4)}] ${gyroUnits} (auto-detected)`);
    
    // Reset tracking state
    this.currentRepData = [];
    this.sampleHistory = [];
    this.velocity = 0;
    this.displacement = 0;
    this.stillCounter = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;
    this.prevVertAccel = 0;
    this.liveAngleDeg = 0;
    this.liveDisplacementCm = 0;
    this.repROMs = [];
    this.primaryAxis = null;
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    this.liveFulfillment = 0;
    
    console.log(`[ROMComputer] Baseline set from ${n} samples. Q: [${avgQ.w.toFixed(3)}, ${avgQ.x.toFixed(3)}, ${avgQ.y.toFixed(3)}, ${avgQ.z.toFixed(3)}]`);
    return true;
  }
  
  calibrateBaseline() {
    this.baselineQuat = null;
    this.baselineAngle = null;
    this.baselineGravity = null;
    this.gravityMag = 9.81;
    this.gyroBias = {x:0,y:0,z:0};
    this.gyroInRadians = false;
    this.currentRepData = [];
    this.sampleHistory = [];
    this.velocity = 0;
    this.displacement = 0;
    this.stillCounter = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;
    this.prevVertAccel = 0;
    this.liveAngleDeg = 0;
    this.liveDisplacementCm = 0;
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    this.liveFulfillment = 0;
    console.log('[ROMComputer] Baseline recalibrated to current position');
  }
  
  reset() {
    this.baselineQuat = null;
    this.baselineAngle = null;
    this.baselineGravity = null;
    this.gravityMag = 9.81;
    this.gyroBias = {x:0,y:0,z:0};
    this.gyroInRadians = false;
    this.primaryAxis = null;
    this.repROMs = [];
    this.currentRepData = [];
    this.velocity = 0;
    this.displacement = 0;
    this.lastTimestamp = 0;
    this.peakDisplacement = 0;
    this.minDisplacement = 0;
    this.stillCounter = 0;
    this.prevVertAccel = 0;
    this.liveAngleDeg = 0;
    this.liveDisplacementCm = 0;
    this.liveVelocity = 0;
    this.sampleHistory = [];
    this.targetROM = null;
    this.isCalibrationRep = false;
    this.romCalibrated = false;
    this.calibrationROMs = [];
    this.repMinAngle = Infinity;
    this.repMaxAngle = -Infinity;
    this.liveRepROM = 0;
    this.liveFulfillment = 0;
    this.DEBUG_MODE = false;
  }
}

// Singleton instance for global access
let _instance = null;

export function getROMComputer() {
  if (!_instance) {
    _instance = new ROMComputer();
  }
  return _instance;
}

export function resetROMComputer() {
  if (_instance) {
    _instance.reset();
  }
  _instance = new ROMComputer();
  return _instance;
}

// Global access for debugging in browser console
if (typeof window !== 'undefined') {
  window.getROMComputer = getROMComputer;
  window.debugROM = () => {
    const rom = getROMComputer();
    rom.enableDebugMode();
    console.log('🔧 Use rom.bypassHighPassFilter(true), rom.resetDisplacementToZero(), etc.');
    return rom;
  };
}
