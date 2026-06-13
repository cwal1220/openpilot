#!/usr/bin/env python3
import argparse
import bz2
import time

import cereal.messaging as messaging
from cereal import log as capnp_log
from selfdrive.boardd.boardd_api_impl import can_batch_to_can_capnp, can_list_to_can_batch

K230_KIA_K7_HYBRID_CAN = {
  0: {
    0x220, 0x260, 0x366, 0x367, 0x371, 0x372, 0x386, 0x389, 0x38d, 0x394,
    0x420, 0x421, 0x47f, 0x4f1, 0x507, 0x50a, 0x50b, 0x534, 0x541, 0x553,
    0x559, 0x593,
  },
  1: {0x251, 0x2b0},
  2: {0x340},
}


def keep_can(can, src_max, addr_filter):
  if can.src > src_max:
    return False
  if addr_filter is None:
    return True
  return can.address in addr_filter.get(can.src, set())


def load_can_events(path, src_max, addr_filter):
  with open(path, "rb") as f:
    ents = capnp_log.Event.read_multiple_bytes(bz2.decompress(f.read()))

  events = []
  first_ts = None
  frame_count = 0
  for msg in ents:
    if msg.which() != "can":
      continue

    can_list = []
    for can in msg.can:
      if keep_can(can, src_max, addr_filter):
        can_list.append((can.address, can.busTime, bytes(can.dat), can.src))
    if not can_list:
      continue

    if first_ts is None:
      first_ts = msg.logMonoTime
    events.append(((msg.logMonoTime - first_ts) * 1e-9, can_list))
    frame_count += len(can_list)

  return events, frame_count


def batch_can_events(events, publish_hz):
  if publish_hz <= 0:
    return [(rel_t, can_list_to_can_batch(can_list)) for rel_t, can_list in events]

  period = 1.0 / publish_hz
  batched_events = []
  bucket_idx = -1
  bucket_frames = []
  last_rel_t = 0.0

  for rel_t, can_list in events:
    idx = int(rel_t / period)
    if bucket_frames and idx != bucket_idx:
      batched_events.append((min((bucket_idx + 1) * period, last_rel_t), can_list_to_can_batch(bucket_frames)))
      bucket_frames = []

    if not bucket_frames:
      bucket_idx = idx
    bucket_frames.extend(can_list)
    last_rel_t = rel_t

  if bucket_frames:
    batched_events.append((last_rel_t, can_list_to_can_batch(bucket_frames)))
  return batched_events


def main():
  parser = argparse.ArgumentParser(description="Replay recorded rlog CAN messages on the cereal can service.")
  parser.add_argument("log", help="rlog.bz2 containing can events")
  parser.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
  parser.add_argument("--src-max", type=int, default=2, help="maximum CAN src/bus to replay")
  parser.add_argument("--k230-kia-k7-hybrid", action="store_true",
                      help="replay only the CAN addresses used by the current K230 KIA K7 HYBRID port")
  parser.add_argument("--publish-hz", type=float, default=0.0,
                      help="coalesce log CAN events and publish at this rate; 0 keeps original event timing")
  parser.add_argument("--once", action="store_true", help="replay the log once instead of looping")
  args = parser.parse_args()

  if args.speed <= 0:
    raise SystemExit("--speed must be greater than zero")

  addr_filter = K230_KIA_K7_HYBRID_CAN if args.k230_kia_k7_hybrid else None
  events, frame_count = load_can_events(args.log, args.src_max, addr_filter)
  if not events:
    raise SystemExit("no CAN events found")
  events = [(rel_t, can_batch_to_can_capnp(can_batch)) for rel_t, can_batch in batch_can_events(events, args.publish_hz)]

  pm = messaging.PubMaster(["can"])
  print(
    f"loaded {len(events)} CAN events, {frame_count} frames, "
    f"duration={events[-1][0]:.1f}s, speed={args.speed}x, src_max={args.src_max}, "
    f"filter={'k230-kia-k7-hybrid' if addr_filter else 'none'}, "
    f"publish_hz={args.publish_hz:g}",
    flush=True,
  )

  loop = 0
  while True:
    start = time.monotonic()
    for rel_t, can_msg in events:
      delay = start + rel_t / args.speed - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      pm.send("can", can_msg)

    loop += 1
    print(f"replayed loop {loop}", flush=True)
    if args.once:
      break


if __name__ == "__main__":
  main()
