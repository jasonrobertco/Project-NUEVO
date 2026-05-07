/**
 * GpsStatusCard — shows current GPS / ArUco tag detection status.
 */
import { useRobotStore } from '../store/robotStore'

export function GpsStatusCard() {
  const gps = useRobotStore((s) => s.gpsStatus)

  const detected = gps?.is_detected ?? false

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-3 space-y-2">
      <div className="flex items-center justify-between">
        <span className="text-xs font-semibold text-white/70">GPS (ArUco)</span>
        <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${
          detected
            ? 'bg-emerald-500/20 text-emerald-400'
            : 'bg-rose-500/20 text-rose-400'
        }`}>
          {detected ? 'Detected' : 'Not detected'}
        </span>
      </div>

      <div className={`space-y-1 transition-opacity ${detected ? 'opacity-100' : 'opacity-30'}`}>
        <div className="flex items-center justify-between">
          <span className="text-xs text-white/50">Tag ID</span>
          <span className="text-xs font-mono text-white">{gps?.tag_id ?? '—'}</span>
        </div>
        <div className="flex items-center justify-between">
          <span className="text-xs text-white/50">X</span>
          <span className="text-xs font-mono text-white">
            {gps ? `${gps.x.toFixed(1)} mm` : '—'}
          </span>
        </div>
        <div className="flex items-center justify-between">
          <span className="text-xs text-white/50">Y</span>
          <span className="text-xs font-mono text-white">
            {gps ? `${gps.y.toFixed(1)} mm` : '—'}
          </span>
        </div>
      </div>

      {!gps && (
        <p className="text-xs text-white/30 italic">Waiting for GPS data…</p>
      )}
    </div>
  )
}
