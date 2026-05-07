import { useRef, useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { Modal } from './common/Modal';
import { wsSend } from '../lib/wsSend';
import figure8Image from '../assets/arrow.svg';
import type { IMUData, MagCalStatusData, SensorRangeData, KinematicsData, OdomParamData } from '../lib/wsProtocol';

// ─── Constants ───────────────────────────────────────────────────────────────

const RANGE_STATUS_LABEL = ['Valid', 'Out of range', 'Sensor error', 'Not installed'];
const RANGE_STATUS_COLOR = [
  'text-emerald-400',
  'text-white/30',
  'text-rose-400',
  'text-amber-400',
];
const SENSOR_TYPE_LABEL = ['Ultrasonic', 'Lidar'];

// ─── Helpers ─────────────────────────────────────────────────────────────────

function quatToEulerDeg(w: number, x: number, y: number, z: number) {
  const roll  = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw   = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  const toDeg = (r: number) => (r * 180) / Math.PI;
  return { roll: toDeg(roll), pitch: toDeg(pitch), yaw: toDeg(yaw) };
}

function fmt(v: number, decimals = 1) {
  return (v >= 0 ? '+' : '') + v.toFixed(decimals);
}

// Build rotation matrix from quaternion (body → world)
function quatToR(w: number, x: number, y: number, z: number): number[][] {
  return [
    [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)  ],
    [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)  ],
    [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
  ];
}

function rotVec(R: number[][], v: number[]): number[] {
  return [
    R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
    R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
    R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2],
  ];
}

// Right-hand isometric projection (camera at front-right-top, i.e. +X, -Y, +Z).
// World X → lower-left (forward/away), World Y → lower-right (left side toward viewer), World Z → up.
// Swapping (wx-wy) → (wy-wx) fixes handedness: 2D cross(X,Y) is now negative = Z out of screen.
function proj(wx: number, wy: number, wz: number, cx: number, cy: number, scale: number) {
  return {
    x: cx + (wy - wx) * 0.866 * scale,
    y: cy + (wx + wy) * 0.5  * scale - wz * scale,
  };
}

function drawArrow(
  ctx: CanvasRenderingContext2D,
  x0: number, y0: number,
  x1: number, y1: number,
  color: string,
  lineWidth: number,
  headLen = 9,
  headW = 4,
) {
  const dx = x1 - x0, dy = y1 - y0;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 2) return;
  const nx = dx / len, ny = dy / len;

  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - nx * headLen + ny * headW, y1 - ny * headLen - nx * headW);
  ctx.lineTo(x1 - nx * headLen - ny * headW, y1 - ny * headLen + nx * headW);
  ctx.closePath();
  ctx.fill();
}

// ─── 3D Robot Frame View ─────────────────────────────────────────────────────

function IMU3DView({ imu }: { imu: IMUData }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const W   = canvas.clientWidth;
    const H   = canvas.clientHeight;
    if (!W || !H) return;
    canvas.width  = W * dpr;
    canvas.height = H * dpr;
    const ctx = canvas.getContext('2d')!;
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, W, H);

    const cx    = W / 2;
    const cy    = H / 2;
    const scale = Math.min(W, H) * 0.30;

    const R = quatToR(imu.quatW, imu.quatX, imu.quatY, imu.quatZ);
    const p = (v: number[]) => {
      const w = rotVec(R, v);
      return proj(w[0], w[1], w[2], cx, cy, scale);
    };
    const O = proj(0, 0, 0, cx, cy, scale);

    // Ground reference ellipse (XY plane at Z=0)
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.07)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i <= 64; i++) {
      const a = (i / 64) * Math.PI * 2;
      const pt = proj(Math.cos(a) * 0.85, Math.sin(a) * 0.85, 0, cx, cy, scale);
      i === 0 ? ctx.moveTo(pt.x, pt.y) : ctx.lineTo(pt.x, pt.y);
    }
    ctx.stroke();
    ctx.restore();

    // Robot box — slightly larger, filled faces for a "solid" look
    const BL = 0.52, BW = 0.52, BH = 0.52;
    const C: number[][] = [
      [ BL/2,-BW/2,-BH/2], [ BL/2, BW/2,-BH/2], [-BL/2, BW/2,-BH/2], [-BL/2,-BW/2,-BH/2],  // 0-3 bottom
      [ BL/2,-BW/2, BH/2], [ BL/2, BW/2, BH/2], [-BL/2, BW/2, BH/2], [-BL/2,-BW/2, BH/2],  // 4-7 top
    ];
    // Camera direction from scene (right-hand, front-right-top camera at [1,-1,1]/√3)
    const CAM = [1/Math.SQRT2, -1/Math.SQRT2, 1/Math.SQRT2]; // simplified to 45°

    // Face definitions: [corner indices, outward normal, fill color]
    const faces: [number[], number[], string][] = [
      [[3,2,6,7], [-1, 0, 0], 'rgba(100,120,160,0.06)'],  // back   (−X)
      [[1,0,4,5], [ 1, 0, 0], 'rgba(248,113,113,0.09)'],  // front  (+X, red hint)
      [[2,3,7,6], [ 0, 1, 0], 'rgba(100,120,160,0.06)'],  // left   (+Y body-left)
      [[0,1,5,4], [ 0,-1, 0], 'rgba(120,180,140,0.08)'],  // right  (−Y body-right, green hint)
      [[0,3,2,1], [ 0, 0,-1], 'rgba(100,120,160,0.04)'],  // bottom (−Z)
      [[4,5,6,7], [ 0, 0, 1], 'rgba(96,165,250,0.10)'],   // top    (+Z, blue hint)
    ];

    // Sort back-to-front by dot(rotated_center, cam_dir)
    const sortedFaces = faces
      .map(([idxs, norm, color]) => {
        const rn = rotVec(R, norm);
        const vis = rn[0]*CAM[0] + rn[1]*CAM[1] + rn[2]*CAM[2];
        // center of face in body frame
        const bc = idxs.reduce((acc, i) => [acc[0]+C[i][0], acc[1]+C[i][1], acc[2]+C[i][2]], [0,0,0]).map(v => v/idxs.length);
        const wc = rotVec(R, bc);
        const depth = wc[0]*CAM[0] + wc[1]*CAM[1] + wc[2]*CAM[2];
        return { idxs, color, vis, depth };
      })
      .sort((a, b) => a.depth - b.depth); // painter's algo: far first

    // Draw filled faces
    for (const { idxs, color, vis } of sortedFaces) {
      if (vis <= 0) continue; // back-face cull
      const pts = idxs.map(i => p(C[i]));
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(pts[0].x, pts[0].y);
      for (let k = 1; k < pts.length; k++) ctx.lineTo(pts[k].x, pts[k].y);
      ctx.closePath();
      ctx.fill();
    }

    // Draw edges on top
    const edges = [
      [0,1],[1,2],[2,3],[3,0],
      [4,5],[5,6],[6,7],[7,4],
      [0,4],[1,5],[2,6],[3,7],
    ];
    ctx.strokeStyle = 'rgba(255,255,255,0.30)';
    ctx.lineWidth = 1;
    for (const [a, b] of edges) {
      const pa = p(C[a]), pb = p(C[b]);
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }

    // Forward-direction marker: small arrow on front face (+X side center)
    // p() takes body-frame coordinates and rotates internally — do NOT pre-rotate.
    const faceCenter = p([BL/2, 0, 0]);
    const faceTop    = p([BL/2, 0,  BH/3]);
    const faceBot    = p([BL/2, 0, -BH/3]);
    const frontVis   = rotVec(R, [1,0,0]);
    if (frontVis[0]*CAM[0] + frontVis[1]*CAM[1] + frontVis[2]*CAM[2] > 0.1) {
      ctx.strokeStyle = 'rgba(248,113,113,0.8)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(faceBot.x, faceBot.y);
      ctx.lineTo(faceTop.x, faceTop.y);
      ctx.stroke();
      // arrowhead
      const dx = faceTop.x - faceCenter.x, dy = faceTop.y - faceCenter.y;
      const len = Math.sqrt(dx*dx+dy*dy) || 1;
      const nx = dx/len, ny = dy/len;
      ctx.fillStyle = 'rgba(248,113,113,0.9)';
      ctx.beginPath();
      ctx.moveTo(faceTop.x, faceTop.y);
      ctx.lineTo(faceTop.x - nx*6 + ny*3, faceTop.y - ny*6 - nx*3);
      ctx.lineTo(faceTop.x - nx*6 - ny*3, faceTop.y - ny*6 + nx*3);
      ctx.closePath();
      ctx.fill();
    }

    // Body axes (X=red, Y=green, Z=blue)
    const AXES = [
      { v: [1,0,0], color: '#f87171', label: 'X' },
      { v: [0,1,0], color: '#4ade80', label: 'Y' },
      { v: [0,0,1], color: '#60a5fa', label: 'Z' },
    ];
    ctx.font = 'bold 11px sans-serif';
    ctx.textBaseline = 'middle';
    for (const { v, color, label } of AXES) {
      const w = rotVec(R, v);
      const tip = proj(w[0], w[1], w[2], cx, cy, scale);
      drawArrow(ctx, O.x, O.y, tip.x, tip.y, color, 2.5);
      const dx = tip.x - O.x, dy = tip.y - O.y;
      const len = Math.sqrt(dx*dx+dy*dy);
      if (len > 0) {
        ctx.fillStyle = color;
        ctx.fillText(label, tip.x + (dx/len)*6, tip.y + (dy/len)*6);
      }
    }

    // Earth-frame acceleration vector (amber, dashed)
    const AS   = 0.55; // scale factor per g
    const aW   = [imu.earthAccX * AS, imu.earthAccY * AS, imu.earthAccZ * AS];
    // earthAcc is already in world/earth frame → project directly without rotation
    const aTip = proj(aW[0], aW[1], aW[2], cx, cy, scale);
    const adx  = aTip.x - O.x, ady = aTip.y - O.y;
    const aLen = Math.sqrt(adx*adx + ady*ady);
    if (aLen > 3) {
      ctx.save();
      ctx.setLineDash([5, 3]);
      drawArrow(ctx, O.x, O.y, aTip.x, aTip.y, '#fbbf24', 1.8, 8, 3.5);
      ctx.restore();
      ctx.fillStyle = '#fbbf24';
      ctx.font = '10px sans-serif';
      ctx.fillText('a', aTip.x + (adx/aLen)*5, aTip.y + (ady/aLen)*5);
    }

  }, [imu]);

  return <canvas ref={canvasRef} className="w-full rounded-lg" style={{ height: '160px' }} />;
}

// ─── 2D Compass View ─────────────────────────────────────────────────────────

function IMUCompass({ imu }: { imu: IMUData }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const W   = canvas.clientWidth;
    const H   = canvas.clientHeight;
    if (!W || !H) return;
    canvas.width  = W * dpr;
    canvas.height = H * dpr;
    const ctx = canvas.getContext('2d')!;
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, W, H);

    const cx = W / 2, cy = H / 2;
    const r  = Math.min(cx, cy) * 0.78;

    // Background circle
    const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, r);
    grad.addColorStop(0, 'rgba(255,255,255,0.08)');
    grad.addColorStop(1, 'rgba(255,255,255,0.02)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = 'rgba(255,255,255,0.18)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // Inner ring
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r * 0.6, 0, Math.PI * 2);
    ctx.stroke();

    // Tick marks
    for (let i = 0; i < 36; i++) {
      const angle  = (i / 36) * Math.PI * 2 - Math.PI / 2;
      const major  = i % 9 === 0;
      const tr     = major ? r * 0.83 : r * 0.90;
      ctx.strokeStyle = major ? 'rgba(255,255,255,0.45)' : 'rgba(255,255,255,0.15)';
      ctx.lineWidth   = major ? 1.5 : 0.75;
      ctx.beginPath();
      ctx.moveTo(cx + tr * Math.cos(angle), cy + tr * Math.sin(angle));
      ctx.lineTo(cx + r  * Math.cos(angle), cy + r  * Math.sin(angle));
      ctx.stroke();
    }

    // Cardinal labels (fixed, N at top)
    const cardinals: [string, number][] = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    for (const [label, deg] of cardinals) {
      const angle = (deg * Math.PI / 180) - Math.PI / 2;
      const lr    = r * 0.70;
      ctx.font      = label === 'N' ? 'bold 12px sans-serif' : '11px sans-serif';
      ctx.fillStyle = label === 'N' ? '#f87171' : 'rgba(255,255,255,0.65)';
      ctx.fillText(label, cx + lr * Math.cos(angle), cy + lr * Math.sin(angle));
    }

    // Tilt-compensated heading using roll/pitch from quaternion
    const { roll, pitch } = quatToEulerDeg(imu.quatW, imu.quatX, imu.quatY, imu.quatZ);
    const cr = Math.cos(roll  * Math.PI / 180);
    const sr = Math.sin(roll  * Math.PI / 180);
    const cp = Math.cos(pitch * Math.PI / 180);
    const sp = Math.sin(pitch * Math.PI / 180);
    const mx = imu.magX, my = imu.magY, mz = imu.magZ;
    const Xh =  mx * cp + mz * sp;
    const Yh =  mx * sp * sr + my * cr - mz * sr * cp;
    const heading = Math.atan2(Yh, Xh);  // radians; 0 = mag X direction

    // Needle: red half toward heading, white half opposite
    const nLen      = r * 0.60;
    const sLen      = r * 0.32;
    const halfW     = 5.5;
    const needleAngle = heading - Math.PI / 2; // canvas angle (−π/2 = top)

    const nx = Math.cos(needleAngle), ny = Math.sin(needleAngle);
    const px = Math.cos(needleAngle + Math.PI / 2), py = Math.sin(needleAngle + Math.PI / 2);

    // North half
    ctx.fillStyle = '#f87171';
    ctx.beginPath();
    ctx.moveTo(cx + nLen * nx,  cy + nLen * ny);
    ctx.lineTo(cx + halfW * px, cy + halfW * py);
    ctx.lineTo(cx,              cy);
    ctx.lineTo(cx - halfW * px, cy - halfW * py);
    ctx.closePath();
    ctx.fill();

    // South half
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.beginPath();
    ctx.moveTo(cx - sLen * nx,  cy - sLen * ny);
    ctx.lineTo(cx + halfW * px, cy + halfW * py);
    ctx.lineTo(cx,              cy);
    ctx.lineTo(cx - halfW * px, cy - halfW * py);
    ctx.closePath();
    ctx.fill();

    // Center cap
    ctx.fillStyle   = 'rgba(255,255,255,0.90)';
    ctx.strokeStyle = 'rgba(0,0,0,0.3)';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Heading readout
    const headingDeg = ((heading * 180 / Math.PI) + 360) % 360;
    ctx.fillStyle    = 'rgba(255,255,255,0.45)';
    ctx.font         = '10px monospace';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(`${headingDeg.toFixed(1)}°`, cx, cy + r + 14);

  }, [imu]);

  return <canvas ref={canvasRef} className="w-full rounded-lg" style={{ height: '168px' }} />;
}

// ─── IMU card ────────────────────────────────────────────────────────────────

function CompactRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between items-baseline gap-2">
      <span className="text-xs text-white/50 shrink-0">{label}</span>
      <span className="text-xs font-mono text-white text-right">{value}</span>
    </div>
  );
}

const IMU_TABS = ['3D View', 'Compass'] as const;
type IMUTab = typeof IMU_TABS[number];
type CalibrationPhase = 'preparing' | 'starting' | 'sampling' | 'complete' | 'failed';

function IMUCard({
  imu,
  magCal,
  imuConfigured,
  imuReady,
  onCalibrate,
}: {
  imu: IMUData | null
  magCal: MagCalStatusData | null
  imuConfigured: boolean
  imuReady: boolean
  onCalibrate: () => void
}) {
  const [tab, setTab] = useState<IMUTab>('3D View');
  const tabIndex = IMU_TABS.indexOf(tab);

  if (!imu) {
    return (
      <Card title="IMU (ICM-20948)">
        <div className="space-y-1">
          <div className="flex justify-between items-baseline">
            <span className="text-xs text-white/50">Fusion mode</span>
            <span className="text-lg font-mono font-semibold text-white/25">—</span>
          </div>
          <div className="flex justify-between items-baseline">
            <span className="text-xs text-white/50">Status</span>
            <span className={`text-xs font-mono ${
              imuConfigured && !imuReady ? 'text-amber-400' : 'text-white/30'
            }`}>
              {imuConfigured && !imuReady ? 'Not installed' : 'Waiting for telemetry'}
            </span>
          </div>
        </div>
      </Card>
    );
  }

  const { roll, pitch, yaw } = quatToEulerDeg(imu.quatW, imu.quatX, imu.quatY, imu.quatZ);
  const gyroX = imu.rawGyroX / 10;
  const gyroY = imu.rawGyroY / 10;
  const gyroZ = imu.rawGyroZ / 10;
  const isCalibrated = imu.magCalibrated === 1;
  const sampleCount = magCal?.sampleCount ?? 0;

  return (
    <Card title="IMU (ICM-20948)">
      <div className="mb-3 rounded-xl border border-white/10 bg-white/5 p-3">
        <div className="flex items-center justify-between gap-3">
          <div>
            <div className="flex items-center gap-2">
              <span
                className={`rounded-full px-2 py-0.5 text-[11px] font-semibold ${
                  isCalibrated
                    ? 'bg-emerald-500/20 text-emerald-300 border border-emerald-400/20'
                    : 'bg-amber-500/20 text-amber-300 border border-amber-400/20'
                }`}
              >
                {isCalibrated ? 'Calibrated' : 'Uncalibrated'}
              </span>
              <span className="text-[11px] text-white/40">
                {isCalibrated ? '9-DoF heading active' : '6-DoF fallback, yaw will drift'}
              </span>
            </div>
            {!isCalibrated && (
              <p className="mt-1 text-xs text-white/55">
                Run magnetometer calibration once so yaw is corrected with magnetic heading.
              </p>
            )}
          </div>
          <button
            onClick={onCalibrate}
            className="shrink-0 rounded-xl border border-cyan-400/40 bg-cyan-500/20 px-3 py-2 text-xs font-semibold text-white transition-all hover:bg-cyan-500/30"
          >
            {isCalibrated ? 'Recalibrate' : 'Calibrate IMU'}
          </button>
        </div>
        {!isCalibrated && sampleCount > 0 && (
          <p className="mt-2 text-[11px] text-white/40">
            Last mag calibration sample count: {sampleCount}
          </p>
        )}
      </div>

      {/* Pill tab switcher */}
      <div className="relative flex rounded-full p-1 bg-white/10 border border-white/15 mb-3">
        <div
          className="absolute top-1 bottom-1 rounded-full bg-white/20 border border-white/30 shadow-inner transition-all duration-200 ease-in-out"
          style={{
            width: 'calc((100% - 8px) / 2)',
            left: `calc(4px + ${tabIndex} * (100% - 8px) / 2)`,
          }}
        />
        {IMU_TABS.map((t) => (
          <button
            key={t}
            onClick={() => setTab(t)}
            className={`relative flex-1 py-1.5 text-xs font-semibold z-10 transition-colors duration-150 ${
              tab === t ? 'text-white' : 'text-white/40 hover:text-white/70'
            }`}
          >
            {t}
          </button>
        ))}
      </div>

      {/* Visualization canvas */}
      {tab === '3D View' ? <IMU3DView imu={imu} /> : <IMUCompass imu={imu} />}

      {/* Compact raw data */}
      <div className="mt-3 pt-3 border-t border-white/10 space-y-1">
        <CompactRow label="Fusion mode" value={isCalibrated ? '9-DoF' : '6-DoF'} />
        <CompactRow label="Orientation (R, P, Y)" value={`${fmt(roll)}°, ${fmt(pitch)}°, ${fmt(yaw)}°`} />
        <CompactRow label="Accel earth (X, Y, Z)" value={`${fmt(imu.earthAccX, 3)}, ${fmt(imu.earthAccY, 3)}, ${fmt(imu.earthAccZ, 3)} g`} />
        <CompactRow label="Gyro (X, Y, Z)"        value={`${fmt(gyroX, 1)}, ${fmt(gyroY, 1)}, ${fmt(gyroZ, 1)} dps`} />
        <CompactRow label="Mag (X, Y, Z)"          value={`${imu.magX}, ${imu.magY}, ${imu.magZ} µT`} />
      </div>
    </Card>
  );
}

function IMUCalibrationModal({
  open,
  phase,
  magCal,
  errorMessage,
  onCancel,
  onClose,
}: {
  open: boolean
  phase: CalibrationPhase
  magCal: MagCalStatusData | null
  errorMessage: string | null
  onCancel: () => void
  onClose: () => void
}) {
  const spanX = magCal ? Math.max(0, magCal.maxX - magCal.minX) : 0
  const spanY = magCal ? Math.max(0, magCal.maxY - magCal.minY) : 0
  const spanZ = magCal ? Math.max(0, magCal.maxZ - magCal.minZ) : 0
  const progress = magCal?.bridgeProgress ?? 0
  const bridgeReady = magCal?.bridgeReady ?? false
  const bridgeFallbackReady = magCal?.bridgeFallbackReady ?? false
  const fitQuality = magCal?.bridgeBestStdRatio

  let title = 'IMU Calibration'
  let subtitle = 'Collecting magnetometer data for heading correction.'
  if (phase === 'preparing') {
    subtitle = 'Stopping the robot and waiting for the firmware to reach IDLE.'
  } else if (phase === 'starting') {
    subtitle = 'Starting calibration sampling on the Arduino.'
  } else if (phase === 'complete') {
    subtitle = 'Calibration saved. The IMU will now use 9-DoF heading correction.'
  } else if (phase === 'failed') {
    subtitle = 'Calibration did not finish successfully.'
  }

  return (
    <Modal open={open} onClose={phase === 'failed' || phase === 'complete' ? onClose : onCancel} title={title} subtitle={subtitle} maxWidth="max-w-2xl">
      <div className="space-y-4">
        {phase === 'preparing' && (
          <p className="text-sm text-white/70">
            The IMU calibration must start from <span className="font-semibold text-white">IDLE</span>. The UI has already requested a stop and will begin calibration as soon as the firmware confirms IDLE.
          </p>
        )}

        {(phase === 'starting' || phase === 'sampling') && (
          <>
            <div className="rounded-2xl border border-white/10 bg-white/5 p-4">
              <p className="text-sm font-semibold text-white">How to move the robot</p>
              <p className="mt-2 text-sm text-white/70">
                Move the robot in a slow figure-8 while also rotating and tilting it through as many orientations as possible. Lift it and include roll and pitch, not only flat yaw motion.
              </p>
              <img
                src={figure8Image}
                alt="Figure-8 motion for IMU calibration"
                className="mt-4 w-56 max-w-full mx-auto"
              />
            </div>

            <div className="rounded-2xl border border-white/10 bg-white/5 p-4">
              <div className="flex items-center justify-between text-xs text-white/55">
                <span>Bridge fit progress</span>
                <span>{progress}%</span>
              </div>
              <div className="mt-2 h-2 overflow-hidden rounded-full bg-white/10">
                <div
                  className="h-full rounded-full bg-gradient-to-r from-cyan-400 to-emerald-400 transition-all"
                  style={{ width: `${progress}%` }}
                />
              </div>
              <div className="mt-3 grid grid-cols-2 gap-3 text-xs text-white/65">
                <div>Samples: <span className="font-mono text-white">{magCal?.sampleCount ?? 0}</span></div>
                <div>Span X/Y/Z: <span className="font-mono text-white">{spanX.toFixed(1)} / {spanY.toFixed(1)} / {spanZ.toFixed(1)} µT</span></div>
                <div>Bridge readiness: <span className="font-mono text-white">{bridgeReady ? 'ready' : 'collecting'}</span></div>
                <div>Fallback save: <span className="font-mono text-white">{bridgeFallbackReady ? 'available' : 'not yet'}</span></div>
                <div>Fit quality: <span className="font-mono text-white">{fitQuality != null ? fitQuality.toFixed(3) : '—'}</span></div>
              </div>
              <p className="mt-3 text-xs text-white/45">
                The bridge will save automatically when the backend decides the collected data is good enough.
              </p>
            </div>
          </>
        )}

        {phase === 'complete' && (
          <div className="rounded-2xl border border-emerald-400/20 bg-emerald-500/10 p-4 text-sm text-emerald-100">
            Magnetometer calibration saved successfully.
          </div>
        )}

        {phase === 'failed' && (
          <div className="rounded-2xl border border-rose-400/20 bg-rose-500/10 p-4 text-sm text-rose-100">
            {errorMessage ?? 'Calibration failed. Try again with slower figure-8 motion and more tilt/roll coverage.'}
          </div>
        )}

        <div className="flex justify-end gap-3">
          {phase === 'failed' || phase === 'complete' ? (
            <button
              onClick={onClose}
              className="rounded-xl border border-white/20 bg-white/10 px-4 py-2 text-sm font-semibold text-white transition-all hover:bg-white/20"
            >
              OK
            </button>
          ) : (
            <button
              onClick={onCancel}
              className="rounded-xl border border-white/20 bg-white/10 px-4 py-2 text-sm font-semibold text-white transition-all hover:bg-white/20"
            >
              Cancel
            </button>
          )}
        </div>
      </div>
    </Modal>
  )
}

// ─── Range sensor card ───────────────────────────────────────────────────────

function RangeCard({ sensor }: { sensor: SensorRangeData }) {
  const isValid   = sensor.status === 0;
  const typeLabel  = SENSOR_TYPE_LABEL[sensor.sensorType] ?? `Type ${sensor.sensorType}`;
  const statusLabel = RANGE_STATUS_LABEL[sensor.status] ?? `Status ${sensor.status}`;
  const statusColor = RANGE_STATUS_COLOR[sensor.status] ?? 'text-white/50';

  return (
    <Card title={`${typeLabel} #${sensor.sensorId}`}>
      <div className="space-y-1">
        <div className="flex justify-between items-baseline">
          <span className="text-xs text-white/50">Distance</span>
          <span className={`text-lg font-mono font-semibold ${isValid ? 'text-white' : 'text-white/25'}`}>
            {isValid ? `${sensor.distanceMm} mm` : '— mm'}
          </span>
        </div>
        <div className="flex justify-between items-baseline">
          <span className="text-xs text-white/50">Status</span>
          <span className={`text-xs font-mono ${statusColor}`}>{statusLabel}</span>
        </div>
      </div>
    </Card>
  );
}

function RangePlaceholderCard({ sensorType, sensorId }: { sensorType: number; sensorId: number }) {
  const typeLabel = SENSOR_TYPE_LABEL[sensorType] ?? `Type ${sensorType}`;

  return (
    <Card title={`${typeLabel} #${sensorId}`}>
      <div className="space-y-1">
        <div className="flex justify-between items-baseline">
          <span className="text-xs text-white/50">Distance</span>
          <span className="text-lg font-mono font-semibold text-white/25">— mm</span>
        </div>
        <div className="flex justify-between items-baseline">
          <span className="text-xs text-white/50">Status</span>
          <span className="text-xs font-mono text-white/30">Waiting for telemetry</span>
        </div>
      </div>
    </Card>
  );
}

// ─── Odometry card ───────────────────────────────────────────────────────────

const MM_TO_IN = 1 / 25.4;

function OdometryCard({ kinematics, odomParams }: { kinematics: KinematicsData; odomParams: OdomParamData | null }) {
  const [unit, setUnit] = useState<'mm' | 'in'>('mm');

  const distFmt = (mm: number)  => unit === 'in'
    ? `${(mm * MM_TO_IN).toFixed(3)} in`
    : `${mm.toFixed(1)} mm`;
  const spdFmt  = (mms: number) => unit === 'in'
    ? `${(mms * MM_TO_IN).toFixed(3)} in/s`
    : `${mms.toFixed(1)} mm/s`;

  const handleReset = () => wsSend('sys_odom_reset', {});

  const thetaDeg = kinematics.theta * 180 / Math.PI;

  return (
    <Card title="Odometry">
      {/* Unit toggle */}
      <div className="flex justify-end mb-2">
        <div className="flex rounded-full bg-white/10 border border-white/15 overflow-hidden">
          {(['mm', 'in'] as const).map((u) => (
            <button
              key={u}
              onClick={() => setUnit(u)}
              className={`px-3 py-1 text-xs font-semibold transition-colors ${
                unit === u ? 'bg-white/20 text-white' : 'text-white/40 hover:text-white/70'
              }`}
            >
              {u}
            </button>
          ))}
        </div>
      </div>

      <div className="space-y-1">
        <CompactRow label="Position (X, Y, θ)" value={`${distFmt(kinematics.x)},  ${distFmt(kinematics.y)},  ${thetaDeg.toFixed(1)}°`} />
        <CompactRow label="Velocity (Vx, Vy)"  value={`${spdFmt(kinematics.vx)},  ${spdFmt(kinematics.vy)}`} />
        {odomParams ? (
          <>
            <div className="my-2 h-px bg-white/10" />
            <CompactRow label="Wheel Ø / Base"  value={`${distFmt(odomParams.wheelDiameterMm)}  /  ${distFmt(odomParams.wheelBaseMm)}`} />
            <CompactRow label="Reset θ"         value={`${odomParams.initialThetaDeg.toFixed(1)}°`} />
            <CompactRow label="Left / Right"    value={`M${odomParams.leftMotorNumber}${odomParams.leftMotorDirInverted ? ' (inv)' : ''}  /  M${odomParams.rightMotorNumber}${odomParams.rightMotorDirInverted ? ' (inv)' : ''}`} />
          </>
        ) : (
          <>
            <div className="my-2 h-px bg-white/10" />
            <div className="text-xs text-white/40">Waiting for odometry parameters</div>
          </>
        )}
      </div>

      <button
        onClick={handleReset}
        className="mt-3 w-full rounded-xl border border-white/20 bg-white/5 px-3 py-2 text-xs font-semibold text-white/70 hover:bg-white/10 hover:text-white transition-all"
      >
        Reset Odometry
      </button>
    </Card>
  );
}

// ─── Card shell ──────────────────────────────────────────────────────────────

function Card({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-2xl" />
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />
      <div className="relative">
        <h4 className="text-sm font-semibold text-white mb-3">{title}</h4>
        {children}
      </div>
    </div>
  );
}

// ─── Placeholder card ─────────────────────────────────────────────────────────

function PlaceholderCard({ label }: { label: string }) {
  return (
    <div className="relative rounded-2xl p-5 backdrop-blur-2xl bg-white/5 border border-dashed border-white/20 shadow-xl">
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />
      <div className="relative text-center space-y-1">
        <p className="text-xs font-semibold text-white/40">{label} Sensors</p>
        <p className="text-xs text-white/25">
          Sensor data will appear here when sensors are attached
        </p>
      </div>
    </div>
  );
}

// ─── Main export ─────────────────────────────────────────────────────────────

interface SensorSectionProps {
  source: 'arduino' | 'rpi';
}

export function SensorSection({ source }: SensorSectionProps) {
  const imu          = useRobotStore((s) => s.imu);
  const magCal       = useRobotStore((s) => s.magCal);
  const system       = useRobotStore((s) => s.system);
  const rangeSensors = useRobotStore((s) => s.rangeSensors);
  const kinematics   = useRobotStore((s) => s.kinematics);
  const odomParams   = useRobotStore((s) => s.odomParams);
  const [calibrationOpen, setCalibrationOpen] = useState(false);
  const [calibrationPhase, setCalibrationPhase] = useState<CalibrationPhase>('preparing');
  const [calibrationError, setCalibrationError] = useState<string | null>(null);
  const sawSamplingRef = useRef(false);
  const completionTimerRef = useRef<number | null>(null);

  const attachedSensors = system?.attachedSensors ?? 0;
  const imuConfigured = (attachedSensors & 0x01) !== 0;
  const imuReady = ((system?.runtimeFlags ?? 0) & 0x10) !== 0;
  const hasIMU = imuConfigured || imuReady;
  const hasUltrasonic = ((attachedSensors & 0x04) !== 0) || rangeSensors.length > 0;
  const hasOdometry = kinematics !== null;
  const hasAny = hasIMU || hasUltrasonic || hasOdometry;

  const resetCalibrationModal = () => {
    if (completionTimerRef.current) {
      clearTimeout(completionTimerRef.current);
      completionTimerRef.current = null;
    }
    sawSamplingRef.current = false;
    setCalibrationError(null);
    setCalibrationOpen(false);
  };

  const beginCalibration = () => {
    sawSamplingRef.current = false;
    setCalibrationError(null);
    setCalibrationOpen(true);

    if (!system) {
      setCalibrationPhase('failed');
      setCalibrationError('System status is not available yet. Wait for telemetry, then try again.');
      return;
    }

    if (system.state === 2) {
      setCalibrationPhase('preparing');
      wsSend('sys_cmd', { command: 2 });
      return;
    }

    if (system.state === 1) {
      setCalibrationPhase('starting');
      wsSend('sensor_mag_cal_cmd', { command: 1 });
      return;
    }

    setCalibrationPhase('failed');
    setCalibrationError('Calibration can only start from IDLE. Reset or wait for the firmware to reach IDLE first.');
  };

  const cancelCalibration = () => {
    wsSend('sensor_mag_cal_cmd', { command: 2 });
    resetCalibrationModal();
  };

  useEffect(() => {
    if (source === 'rpi') return;
    if (!calibrationOpen || calibrationPhase !== 'preparing') return;
    if (system?.state === 1) {
      setCalibrationPhase('starting');
      wsSend('sensor_mag_cal_cmd', { command: 1 });
    }
  }, [source, calibrationOpen, calibrationPhase, system?.state]);

  useEffect(() => {
    if (source === 'rpi') return;
    if (!calibrationOpen) return;

    if (magCal?.state === 1) {
      sawSamplingRef.current = true;
      setCalibrationPhase('sampling');
      return;
    }

    if (sawSamplingRef.current && (imu?.magCalibrated === 1 || magCal?.state === 3)) {
      setCalibrationPhase('complete');
      if (!completionTimerRef.current) {
        completionTimerRef.current = window.setTimeout(() => {
          resetCalibrationModal();
        }, 1200);
      }
      return;
    }

    if (sawSamplingRef.current && magCal?.state === 4) {
      setCalibrationPhase('failed');
      setCalibrationError('Calibration data was insufficient. Try again with slower figure-8 motion and more tilt/roll coverage.');
      return;
    }

    if (sawSamplingRef.current && magCal?.state === 0) {
      setCalibrationPhase('failed');
      setCalibrationError('Calibration stopped before a valid fit was saved. Try again and include more 3D tilt coverage.');
    }
  }, [source, calibrationOpen, imu?.magCalibrated, magCal?.state]);

  if (!hasAny) {
    return <PlaceholderCard label="Arduino" />;
  }

  return (
    <div className="space-y-3">
      {hasIMU && (
        <IMUCard
          imu={imu}
          magCal={magCal}
          imuConfigured={imuConfigured}
          imuReady={imuReady}
          onCalibrate={beginCalibration}
        />
      )}
      {hasOdometry && kinematics && <OdometryCard kinematics={kinematics} odomParams={odomParams} />}
      {rangeSensors.map((sensor) => (
        <RangeCard key={sensor.sensorId} sensor={sensor} />
      ))}
      {hasUltrasonic && rangeSensors.length === 0 && (
        <RangePlaceholderCard sensorType={0} sensorId={0} />
      )}
      <IMUCalibrationModal
        open={calibrationOpen}
        phase={calibrationPhase}
        magCal={magCal}
        errorMessage={calibrationError}
        onCancel={cancelCalibration}
        onClose={resetCalibrationModal}
      />
    </div>
  );
}
