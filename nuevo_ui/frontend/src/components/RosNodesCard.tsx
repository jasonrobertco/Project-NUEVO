/**
 * RosNodesCard — collapsible list of running ROS2 nodes with pub/sub details.
 */
import { useState } from 'react'
import { ChevronDown, ChevronRight } from 'lucide-react'
import { useRobotStore } from '../store/robotStore'
import type { RosNodeEntry } from '../lib/wsProtocol'

function NodeRow({ node }: { node: RosNodeEntry }) {
  const [open, setOpen] = useState(false)

  return (
    <div className="rounded-lg bg-white/5 border border-white/10 overflow-hidden">
      <button
        onClick={() => setOpen((o) => !o)}
        className="w-full flex items-center gap-1.5 px-2 py-1.5 text-left hover:bg-white/5 transition-colors"
      >
        {open
          ? <ChevronDown className="size-3 text-white/50 shrink-0" />
          : <ChevronRight className="size-3 text-white/50 shrink-0" />
        }
        <span className="text-xs font-mono text-white/80 truncate">{node.name}</span>
        <span className="ml-auto text-xs text-white/30 shrink-0">
          {node.publishers.length}↑ {node.subscribers.length}↓
        </span>
      </button>

      {open && (
        <div className="px-3 pb-2 space-y-1.5">
          {node.publishers.length > 0 && (
            <div>
              <p className="text-xs text-emerald-400/70 font-medium mb-0.5">Publishes</p>
              {node.publishers.map((t) => (
                <p key={t} className="text-xs font-mono text-white/50 pl-2 truncate">{t}</p>
              ))}
            </div>
          )}
          {node.subscribers.length > 0 && (
            <div>
              <p className="text-xs text-cyan-400/70 font-medium mb-0.5">Subscribes</p>
              {node.subscribers.map((t) => (
                <p key={t} className="text-xs font-mono text-white/50 pl-2 truncate">{t}</p>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  )
}

export function RosNodesCard() {
  const nodes = useRobotStore((s) => s.rosNodes)
  const [open, setOpen] = useState(false)

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-3 space-y-2">
      <button
        onClick={() => setOpen((o) => !o)}
        className="w-full flex items-center justify-between"
      >
        <span className="text-xs font-semibold text-white/70">ROS Nodes</span>
        <div className="flex items-center gap-1.5">
          {nodes.length > 0 && (
            <span className="text-xs text-white/40">{nodes.length} nodes</span>
          )}
          {open
            ? <ChevronDown className="size-3 text-white/50" />
            : <ChevronRight className="size-3 text-white/50" />
          }
        </div>
      </button>

      {open && (
        <div className="space-y-1">
          {nodes.length === 0 ? (
            <p className="text-xs text-white/30 italic">No data yet — waiting for bridge…</p>
          ) : (
            nodes.map((n) => <NodeRow key={n.name} node={n} />)
          )}
        </div>
      )}
    </div>
  )
}
