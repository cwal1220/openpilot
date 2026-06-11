#!/usr/bin/env python3
import argparse
import bz2
import time

import cereal.messaging as messaging
from cereal import log as capnp_log
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp


def load_can_events(path, src_max):
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
      if can.src <= src_max:
        can_list.append((can.address, can.busTime, can.dat, can.src))
    if not can_list:
      continue

    if first_ts is None:
      first_ts = msg.logMonoTime
    events.append(((msg.logMonoTime - first_ts) * 1e-9, can_list))
    frame_count += len(can_list)

  return events, frame_count


def main():
  parser = argparse.ArgumentParser(description="Replay recorded rlog CAN messages on the cereal can service.")
  parser.add_argument("log", help="rlog.bz2 containing can events")
  parser.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
  parser.add_argument("--src-max", type=int, default=2, help="maximum CAN src/bus to replay")
  parser.add_argument("--once", action="store_true", help="replay the log once instead of looping")
  args = parser.parse_args()

  if args.speed <= 0:
    raise SystemExit("--speed must be greater than zero")

  events, frame_count = load_can_events(args.log, args.src_max)
  if not events:
    raise SystemExit("no CAN events found")

  pm = messaging.PubMaster(["can"])
  print(
    f"loaded {len(events)} CAN events, {frame_count} frames, "
    f"duration={events[-1][0]:.1f}s, speed={args.speed}x, src_max={args.src_max}",
    flush=True,
  )

  loop = 0
  while True:
    start = time.monotonic()
    for rel_t, can_list in events:
      delay = start + rel_t / args.speed - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      pm.send("can", can_list_to_can_capnp(can_list))

    loop += 1
    print(f"replayed loop {loop}", flush=True)
    if args.once:
      break


if __name__ == "__main__":
  main()
