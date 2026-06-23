import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { JOINT_NAMES, HOME_ANGLES } from '../state/useStore'
import { useRobotStore } from '../state/context'
import {
  applyAnglesToRobot,
  readAnglesFromRobot,
  computeGrabPose,
  solveCCDIK,
  lerpAngles,
  easeInOutCubic,
  clampAllJoints,
} from '../ik/ikSolver'

// Seconds per motion segment
const SPEED = {
  moving_to_start: 2.0,
  grabbing:        0.5,
  moving_to_end:   2.0,
  releasing:       0.5,
  returning:       1.6,
}

const SEQUENCE = {
  moving_to_start: 'grabbing',
  grabbing:        'moving_to_end',
  moving_to_end:   'releasing',
  releasing:       'returning',
  returning:       'idle',
}

// Distance from the platform's base centre to the target object — chosen so
// the arm can reach the target naturally without being either cramped or
// fully extended.
const PLATFORM_STANDOFF = 1.1

/* Pick where the platform should park to grab the target.
 *
 *   parkingRef = 'origin' (default, main demo) — park STANDOFF metres before
 *                the target along the line from WORLD ORIGIN to the target.
 *   parkingRef = 'self'   (warehouse)         — park STANDOFF metres before
 *                the target along the line from the robot's CURRENT platform
 *                position to the target.  This way each roaming robot
 *                approaches from the side it's already on.
 */
function computePlatformPoseFor(targetWorldPos, currentPlatformPos, parkingRef) {
  const tx = targetWorldPos[0]
  const tz = targetWorldPos[2]
  const refX = parkingRef === 'self' ? currentPlatformPos[0] : 0
  const refZ = parkingRef === 'self' ? currentPlatformPos[2] : 0
  const dx = tx - refX
  const dz = tz - refZ
  const dist = Math.hypot(dx, dz) || 1e-6
  const px = tx - (dx / dist) * PLATFORM_STANDOFF
  const pz = tz - (dz / dist) * PLATFORM_STANDOFF
  return { position: [px, 0, pz], rotation: [0, 0, 0] }
}

// Half the AGV's footprint plus a little buffer — restricted zones are
// expanded by this much before we test against them, so the platform body
// (not just its centre point) stays clear of them.
const AGV_CLEARANCE = 0.65

// Default floor <Grid cellSize> — a "grid line" here is the same line the
// player sees drawn on the floor.  The active cell/origin are read from the
// store each frame (gridCell/gridOrigin) so a scene with a different grid
// unit (e.g. the site planner's gridSizeCm) snaps to its own lines; these
// constants are just the fallbacks that reproduce the original 1 m grid.
const GRID_CELL = 1
const GRID_ORIGIN = [0, 0]

function snapToGrid(v, cell = GRID_CELL, origin = 0) {
  return Math.round((v - origin) / cell) * cell + origin
}

/* Snaps `v` to a grid line, but biased towards `other` instead of to the
 * NEAREST line.  Plain nearest-rounding is what caused the "robot hops
 * backward then forward" glitch: if a coordinate's fractional part was
 * just past the halfway mark in the "wrong" direction, snapToGrid would
 * round it behind where it started, so the very first hop of the trip
 * briefly went backwards before the rest of the path corrected forward.
 * Rounding towards `other` instead guarantees the snapped value always
 * lies between v and other — i.e. the hop is always progress, never a
 * step back. */
function snapBiased(v, other, cell = GRID_CELL, origin = 0) {
  if (Math.abs(other - v) < 1e-9) return snapToGrid(v, cell, origin)
  const n = (v - origin) / cell
  return (other > v ? Math.ceil(n) : Math.floor(n)) * cell + origin
}

/* Does the segment p0→p1 (2D, [x,z]) cross the zone's rectangle, expanded
 * by `margin` on every side?  Standard slab/AABB segment test. */
function segmentHitsZone(p0, p1, zone, margin) {
  const lo = [zone.minX - margin, zone.minZ - margin]
  const hi = [zone.maxX + margin, zone.maxZ + margin]
  const d = [p1[0] - p0[0], p1[1] - p0[1]]
  let tmin = 0, tmax = 1
  for (let axis = 0; axis < 2; axis++) {
    const o = p0[axis]
    if (Math.abs(d[axis]) < 1e-9) {
      if (o < lo[axis] || o > hi[axis]) return false
    } else {
      let t0 = (lo[axis] - o) / d[axis]
      let t1 = (hi[axis] - o) / d[axis]
      if (t0 > t1) { const tmp = t0; t0 = t1; t1 = tmp }
      tmin = Math.max(tmin, t0)
      tmax = Math.min(tmax, t1)
      if (tmin > tmax) return false
    }
  }
  return true
}

function findBlockingZone(p0, p1, zones, margin = AGV_CLEARANCE) {
  for (const z of zones) {
    if (segmentHitsZone(p0, p1, z, margin)) return z
  }
  return null
}

function cornerCandidates(blocker, margin) {
  return [
    [blocker.minX - margin, blocker.minZ - margin],
    [blocker.minX - margin, blocker.maxZ + margin],
    [blocker.maxX + margin, blocker.minZ - margin],
    [blocker.maxX + margin, blocker.maxZ + margin],
  ]
}

/* Picks the cheapest corner of `blocker` (expanded by `margin`) such that a
 * straight a2→corner→b2 detour doesn't re-cross any zone in `zones`. Falls
 * back to a corner that at least clears `blocker` itself, then to the
 * cheapest corner outright — resolveZoneCrossings below re-checks the
 * result and will detour again around whatever it still hits. */
function pickDetourCorner(a2, b2, blocker, zones, margin) {
  const candidates = cornerCandidates(blocker, margin)
  const cost = (c) => Math.hypot(c[0] - a2[0], c[1] - a2[1]) + Math.hypot(b2[0] - c[0], b2[1] - c[1])
  const cheapestClearing = (against) => {
    let best = null, bestCost = Infinity
    for (const c of candidates) {
      if (findBlockingZone(a2, c, against) || findBlockingZone(c, b2, against)) continue
      const cst = cost(c)
      if (cst < bestCost) { bestCost = cst; best = c }
    }
    return best
  }
  return cheapestClearing(zones) || cheapestClearing([blocker])
    || candidates.reduce((best, c) => (best === null || cost(c) < cost(best) ? c : best), null)
}

/* Makes a single leg a→b axis-aligned: if it's already parallel to X or Z
 * (or zero-length), it's left as one leg; otherwise a bend is inserted
 * (move in X to b's X, then in Z to b's Z) so the AGV is never asked to
 * drive diagonally. */
function axisAlignLeg(a, b) {
  if (Math.abs(a[0] - b[0]) < 1e-6 || Math.abs(a[2] - b[2]) < 1e-6) return [b]
  return [[b[0], a[1], a[2]], b]
}

/* Runs axisAlignLeg over an entire waypoint list, so no matter how the
 * waypoints were produced (grid-snapping, a zone detour corner, …) the
 * resulting path is guaranteed to have no diagonal legs. */
function axisAlignPath(waypoints) {
  const out = [waypoints[0]]
  for (let i = 0; i < waypoints.length - 1; i++) {
    for (const p of axisAlignLeg(out[out.length - 1], waypoints[i + 1])) out.push(p)
  }
  return out
}

/* Repeatedly finds the first leg of `path` that cuts through a restricted
 * zone and reroutes it around that zone's nearest clear corner, until no
 * leg crosses any zone. This is what guarantees the AGV never drives
 * through a zone — it's checked on the FINAL path (after grid-snapping),
 * not just the original straight line, so a detour the grid-snap step
 * introduces gets caught and fixed too. A capped number of passes keeps a
 * pathological zone layout (e.g. several overlapping zones) from looping
 * forever; in practice a handful of user-drawn zones resolves in 1-2 passes.
 * Each detour is re-axis-aligned immediately (when gridMovement is on) so
 * the new corner never introduces a diagonal hop either. */
function resolveZoneCrossings(path, zones, gridMovement, cell = GRID_CELL, origin = GRID_ORIGIN) {
  if (!zones || zones.length === 0) return path
  let result = path
  for (let pass = 0; pass < 12; pass++) {
    let fixedSomething = false
    for (let i = 0; i < result.length - 1; i++) {
      const a = result[i], b = result[i + 1]
      const a2 = [a[0], a[2]], b2 = [b[0], b[2]]
      const blocker = findBlockingZone(a2, b2, zones)
      if (!blocker) continue
      // Extra margin when grid-snapping the corner below, so rounding it onto
      // the nearest grid line can't pull it back inside the AGV's clearance.
      const margin = gridMovement ? AGV_CLEARANCE + cell : AGV_CLEARANCE
      let corner = pickDetourCorner(a2, b2, blocker, zones, margin)
      if (gridMovement) corner = [snapToGrid(corner[0], cell, origin[0]), snapToGrid(corner[1], cell, origin[1])]
      const mid = [corner[0], a[1], corner[1]]
      result = [...result.slice(0, i + 1), mid, ...result.slice(i + 1)]
      if (gridMovement) result = axisAlignPath(result)
      fixedSomething = true
      break
    }
    if (!fixedSomething) break
  }
  return result
}

function lerpRotation(a, b, t) {
  return [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
  ]
}

function legLength(a, b) {
  return Math.hypot(b[0] - a[0], b[2] - a[2])
}

function legPosition(a, b, u) {
  const y = a[1] + (b[1] - a[1]) * u
  return [a[0] + (b[0] - a[0]) * u, y, a[2] + (b[2] - a[2]) * u]
}

/* True grid-cell pathfinding (BFS) between two ON-GRID points, routing
 * around any zone.  Replaces the old "bounce off one corner" heuristic,
 * which could only represent a single simple detour — with the floor
 * already bent into an L-shaped grid route, a zone sitting near the elbow
 * could block BOTH legs at once, and the corner-bounce had no way to find
 * a real way around that, producing convoluted or backtracking paths.
 * BFS on the actual grid always finds a genuine shortest route, so the
 * result is both correct (never enters a zone) and visually sane (no
 * needless zig-zagging). Returns null if no route exists in the searched
 * area (caller falls back to the simple corner-bounce in that case). */
function bfsGridPath(start2, goal2, zones, cell = GRID_CELL) {
  const margin = AGV_CLEARANCE
  const blocked = (x, z) => zones.some((zo) =>
    x >= zo.minX - margin && x <= zo.maxX + margin && z >= zo.minZ - margin && z <= zo.maxZ + margin)

  const xs = [start2[0], goal2[0]], zs = [start2[1], goal2[1]]
  for (const zo of zones) { xs.push(zo.minX, zo.maxX); zs.push(zo.minZ, zo.maxZ) }
  const pad = 3
  const minX = Math.floor(Math.min(...xs)) - pad, maxX = Math.ceil(Math.max(...xs)) + pad
  const minZ = Math.floor(Math.min(...zs)) - pad, maxZ = Math.ceil(Math.max(...zs)) + pad
  // Safety cap so a pathological zone spread can't search forever.
  if (((maxX - minX) / cell + 1) * ((maxZ - minZ) / cell + 1) > 4000) return null
  if (blocked(start2[0], start2[1]) || blocked(goal2[0], goal2[1])) return null

  const key = (x, z) => `${x},${z}`
  const goalKey = key(goal2[0], goal2[1])
  const cameFrom = new Map([[key(start2[0], start2[1]), null]])
  const queue = [start2]
  const DIRS = [[cell, 0], [-cell, 0], [0, cell], [0, -cell]]

  for (let qi = 0; qi < queue.length; qi++) {
    const [x, z] = queue[qi]
    if (key(x, z) === goalKey) break
    for (const [dx, dz] of DIRS) {
      const nx = x + dx, nz = z + dz
      if (nx < minX || nx > maxX || nz < minZ || nz > maxZ) continue
      const nk = key(nx, nz)
      if (cameFrom.has(nk) || blocked(nx, nz)) continue
      cameFrom.set(nk, [x, z])
      queue.push([nx, nz])
    }
  }
  if (!cameFrom.has(goalKey)) return null

  const cells = []
  for (let cur = goal2; cur; cur = cameFrom.get(key(cur[0], cur[1]))) cells.push(cur)
  cells.reverse()

  // Merge consecutive cells that continue in the same direction into one leg.
  const out = [cells[0]]
  let prevDir = null
  for (let i = 1; i < cells.length; i++) {
    const dir = [Math.sign(cells[i][0] - cells[i - 1][0]), Math.sign(cells[i][1] - cells[i - 1][1])]
    if (prevDir && dir[0] === prevDir[0] && dir[1] === prevDir[1]) out[out.length - 1] = cells[i]
    else out.push(cells[i])
    prevDir = dir
  }
  return out
}

/* Builds the AGV's travel path from fromPos to toPos.  Off the grid it's
 * just a zone-safe straight/bounced line (resolveZoneCrossings).  On the
 * grid: snap the endpoints onto the grid (biased so the snap is always
 * progress, never backwards — see snapBiased), BFS the safe route between
 * those grid points, then hook the unavoidable short off-grid stub at each
 * end back up — resolveZoneCrossings runs last as a safety net for those
 * two stubs only, since the BFS portion is already provably zone-clear. */
function buildSafePath(fromPos, toPos, zones, gridMovement, cell = GRID_CELL, origin = GRID_ORIGIN) {
  if (!gridMovement) return resolveZoneCrossings([fromPos, toPos], zones, false)

  const y = fromPos[1]
  const [ox, oz] = origin
  const aSnap2 = [snapBiased(fromPos[0], toPos[0], cell, ox), snapBiased(fromPos[2], toPos[2], cell, oz)]
  const bSnap2 = [snapBiased(toPos[0], fromPos[0], cell, ox), snapBiased(toPos[2], fromPos[2], cell, oz)]
  const core2 = (zones && zones.length > 0 && (aSnap2[0] !== bSnap2[0] || aSnap2[1] !== bSnap2[1]))
    ? bfsGridPath(aSnap2, bSnap2, zones, cell)
    : null
  const corePoints2 = core2 || [aSnap2, bSnap2]

  let path = [fromPos, ...corePoints2.map((p) => [p[0], y, p[1]]), toPos]
  path = dedupePath(path)
  path = axisAlignPath(path)
  return resolveZoneCrossings(path, zones, true, cell, origin)
}

/* Drops consecutive points that are (almost) the same — happens whenever
 * fromPos/toPos already sit on the grid, so their snapped point coincides
 * with them and would otherwise leave a useless zero-length leg. */
function dedupePath(path) {
  const out = [path[0]]
  for (let i = 1; i < path.length; i++) {
    const prev = out[out.length - 1], p = path[i]
    if (Math.hypot(p[0] - prev[0], p[2] - prev[2]) > 1e-4) out.push(p)
  }
  return out
}

/* Walks a multi-waypoint path (as built by buildSafePath) to the world
 * position/rotation at overall progress `t`. */
function poseAlongPath(path, rotA, rotB, t) {
  const rotation = lerpRotation(rotA, rotB, t)
  if (path.length < 2) return { position: [...path[0]], rotation }

  const legLens = []
  for (let i = 0; i < path.length - 1; i++) legLens.push(legLength(path[i], path[i + 1]))
  const total = legLens.reduce((a, b) => a + b, 0)
  if (total < 1e-6) return { position: [...path[path.length - 1]], rotation }

  let remaining = t * total
  for (let i = 0; i < legLens.length; i++) {
    const len = legLens[i]
    if (remaining <= len || i === legLens.length - 1) {
      const u = len > 1e-6 ? Math.min(1, remaining / len) : 1
      return { position: legPosition(path[i], path[i + 1], u), rotation }
    }
    remaining -= len
  }
  return { position: [...path[path.length - 1]], rotation }
}

/**
 * Headless R3F component — drives the joint animation and (in mobile mode)
 * the AGV platform pose.  Pick-and-place sequence:
 *   moving_to_start → grabbing → moving_to_end → releasing → returning → idle
 *
 * Binds to the per-robot store from `RobotStoreContext`; with no provider it
 * uses the lib singleton, which is exactly the main demo's behaviour.
 */
export default function AnimationController() {
  const progressRef = useRef(0)
  const prevStateRef = useRef('idle')
  const lastFollowKeyRef = useRef('')
  const useStore = useRobotStore()

  useFrame((_, delta) => {
    const store = useStore.getState()
    const {
      animState, robotRef, fromAngles, toAngles, fromPlatform, toPlatform,
      followTarget, startObject, endObject, mobileMode,
      platformPose, parkingRef, platformPath,
    } = store

    // ── Follow mode: live IK as the user drags the target ──────────────────
    if (animState === 'idle' && followTarget && robotRef) {
      const obj = followTarget === 'start' ? startObject : endObject
      const key = `${obj.position.join(',')}|${obj.rotation.join(',')}|${obj.grabVector.join(',')}|${mobileMode}`
      if (key !== lastFollowKeyRef.current) {
        lastFollowKeyRef.current = key

        if (mobileMode) {
          const platPose = computePlatformPoseFor(obj.position, platformPose.position, parkingRef)
          useStore.setState({ platformPose: platPose })
          _setGroupPose(store.platformGroupRef, platPose)
        }

        const { faceCenter, toolZ } = computeGrabPose(obj.position, obj.rotation, obj.grabVector)
        const solved = solveCCDIK(robotRef, faceCenter, toolZ, 80, 0.006)
        if (solved) {
          applyAnglesToRobot(robotRef, solved)
          useStore.setState({ jointAngles: { ...solved } })
        }
      }
      return
    }

    if (animState === 'idle') {
      prevStateRef.current = 'idle'
      progressRef.current  = 0
      lastFollowKeyRef.current = ''
      return
    }

    // ── First frame of a new segment: solve IK + pick platform target ─────
    if (prevStateRef.current !== animState) {
      prevStateRef.current = animState
      progressRef.current  = 0
      // Also publish the reset to the store so other useFrame consumers
      // (e.g. CarriedObject, the warehouse scheduler) don't read the stale
      // animProgress=1 left over from the previous segment on the *next*
      // frame.  Without this, they slerp to the end of their interpolation
      // for one frame.
      useStore.setState({ animProgress: 0 })

      const currentAngles   = readAnglesFromRobot(robotRef) || { ...HOME_ANGLES }
      const currentPlatform = { ...store.platformPose }

      if (animState === 'moving_to_start') {
        _solveSegment(useStore, store, 'start', currentAngles, currentPlatform)
      } else if (animState === 'moving_to_end') {
        _solveSegment(useStore, store, 'end',   currentAngles, currentPlatform)
      } else if (animState === 'returning') {
        const homeAngles = clampAllJoints({ ...HOME_ANGLES })
        const homePlatform = { ...store.homePlatform }
        useStore.setState({
          fromAngles: currentAngles, toAngles: homeAngles,
          fromPlatform: currentPlatform, toPlatform: homePlatform,
          platformPath: buildSafePath(currentPlatform.position, homePlatform.position, store.zones, store.gridMovement, store.gridCell, store.gridOrigin),
        })
        store.addLog('info', 'Returning to HOME…')
      } else {
        useStore.setState({
          fromAngles: currentAngles, toAngles: currentAngles,
          fromPlatform: currentPlatform, toPlatform: currentPlatform,
          platformPath: [currentPlatform.position, currentPlatform.position],
        })
      }
      return
    }

    // ── Advance segment ────────────────────────────────────────────────────
    const duration = SPEED[animState] || 1.0
    progressRef.current = Math.min(1, progressRef.current + delta / duration)
    const t = easeInOutCubic(progressRef.current)

    // Collect this frame's changes into ONE store write.  Three separate
    // setState calls per robot per frame meant three zustand notify passes
    // every frame for every arm — batching them into a single update cuts that
    // overhead by 3× and is a big part of keeping a multi-robot run smooth.
    const patch = { animProgress: progressRef.current }

    if (fromAngles && toAngles && robotRef) {
      const interp = lerpAngles(fromAngles, toAngles, t)
      applyAnglesToRobot(robotRef, interp)
      patch.jointAngles = { ...interp }
    }
    if (mobileMode && fromPlatform && toPlatform) {
      const path = platformPath && platformPath.length >= 2
        ? platformPath
        : [fromPlatform.position, toPlatform.position]
      patch.platformPose = poseAlongPath(path, fromPlatform.rotation, toPlatform.rotation, t)
    }

    useStore.setState(patch)

    if (progressRef.current >= 1) {
      _onSegmentComplete(useStore, store, animState)
    }
  })

  return null
}

function _solveSegment(useStore, store, target, currentAngles, currentPlatform) {
  const { robotRef, startObject, endObject, addLog, mobileMode, platformGroupRef, parkingRef } = store
  if (!robotRef) return

  const obj = target === 'start' ? startObject : endObject
  const { faceCenter, toolZ } = computeGrabPose(obj.position, obj.rotation, obj.grabVector)

  const targetPlatform = mobileMode
    ? computePlatformPoseFor(obj.position, currentPlatform.position, parkingRef)
    : currentPlatform

  addLog('info', `IK: solving for ${target.toUpperCase()}`, {
    X: faceCenter.x.toFixed(3),
    Y: faceCenter.y.toFixed(3),
    Z: faceCenter.z.toFixed(3),
  })

  // Snap platform group into its future pose for the duration of the solve
  const group = platformGroupRef
  let savedPos = null, savedRot = null
  if (mobileMode && group) {
    savedPos = group.position.clone()
    savedRot = group.rotation.clone()
    _setGroupPose(group, targetPlatform)
  }

  const solved = solveCCDIK(robotRef, faceCenter, toolZ, 160, 0.005)

  if (mobileMode && group && savedPos && savedRot) {
    group.position.copy(savedPos)
    group.rotation.copy(savedRot)
    group.updateMatrixWorld(true)
  }

  if (solved) {
    const clamped = clampAllJoints(solved)
    applyAnglesToRobot(robotRef, currentAngles)
    addLog('ok', `IK converged for ${target.toUpperCase()}`, _logAngles(clamped))
    useStore.setState({
      fromAngles: currentAngles, toAngles: clamped,
      fromPlatform: currentPlatform, toPlatform: targetPlatform,
      platformPath: buildSafePath(currentPlatform.position, targetPlatform.position, store.zones, store.gridMovement, store.gridCell, store.gridOrigin),
    })
  } else {
    addLog('warn', `IK did not converge for ${target} — target may be out of reach`)
    applyAnglesToRobot(robotRef, currentAngles)
    useStore.setState({
      fromAngles: currentAngles, toAngles: currentAngles,
      fromPlatform: currentPlatform, toPlatform: currentPlatform,
      platformPath: [currentPlatform.position, currentPlatform.position],
    })
  }
}

function _setGroupPose(group, pose) {
  if (!group) return
  group.position.set(pose.position[0], pose.position[1], pose.position[2])
  group.rotation.set(pose.rotation[0], pose.rotation[1], pose.rotation[2])
  group.updateMatrixWorld(true)
}

function _onSegmentComplete(useStore, store, current) {
  const { addLog, setAnimState, robotRef } = store
  const currentAngles = robotRef
    ? Object.fromEntries(JOINT_NAMES.map((n) => [n, robotRef.joints[n]?.angle ?? 0]))
    : {}

  switch (current) {
    case 'moving_to_start':
      addLog('ok', '✓ Reached START — executing grab', _logAngles(currentAngles))
      break
    case 'grabbing':
      addLog('info', '✓ Object grabbed')
      break
    case 'moving_to_end':
      addLog('ok', '✓ Reached END — releasing', _logAngles(currentAngles))
      break
    case 'releasing':
      addLog('info', '✓ Object released')
      break
    case 'returning':
      addLog('ok', '✓ HOME — sequence complete', _logAngles(HOME_ANGLES))
      useStore.setState({ animProgress: 0 })
      break
  }

  const next = SEQUENCE[current] || 'idle'
  setAnimState(next)
}

function _logAngles(angles) {
  const out = {}
  for (const n of JOINT_NAMES) {
    out[n.replace('joint_', 'A')] = angles[n] ?? 0
  }
  return out
}
