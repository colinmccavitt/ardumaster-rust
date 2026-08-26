-- FW-007: a mission driver that runs on simulated time.
--
-- The autotest drives SITL over MAVLink from a wall-clock-paced client, so
-- its commands land at simulated times that differ between runs. Two runs of
-- test.Plane.ClimbBeforeTurn stayed bit-identical for 34 seconds and then
-- parted company, traceable to one mode change arriving 40 ms apart.
--
-- This script runs on the vehicle's own scheduler instead. Every decision it
-- makes is keyed on millis(), which in SITL is simulated time, so the same
-- commands land at the same simulated moment in every run by construction.
-- That is the property the golden-trajectory oracle needs and the only thing
-- MAVProxy cannot provide.
--
-- Deliberately simple: this is a determinism fixture, not a flight test. It
-- must be reproducible, not realistic.

local STEP_MS = 100

-- What to do and when, in milliseconds of simulated time since boot. The
-- times are generous because the point is reproducibility, not speed; a step
-- that fires while the previous one is still settling would still be
-- reproducible, but it would be harder to read a divergence out of.
local SCHEDULE = {
   { at =  5000, what = "arm" },
   { at =  8000, what = "mode", mode = 13 },   -- TAKEOFF
   { at = 40000, what = "mode", mode = 12 },   -- LOITER
   { at = 80000, what = "mode", mode = 11 },   -- RTL
   -- Disarm at a fixed simulated moment so the run ends on simulated time.
   -- With LOG_DISARMED off the log closes here, which is what lets the
   -- runner stop without a stopwatch. Without this the vehicle loiters in
   -- RTL forever and the run ends only when a wall-clock budget expires --
   -- which is exactly what slice 1 warned against, and what the first Lua
   -- runs did.
   { at = 120000, what = "disarm" },
}

local next_step = 1

function update()
   local now = millis():toint()

   while next_step <= #SCHEDULE and now >= SCHEDULE[next_step].at do
      local step = SCHEDULE[next_step]

      if step.what == "arm" then
         -- arm_force skips the pre-arm checks. This is a determinism
         -- fixture: the checks depend on sensor settling and EKF health,
         -- which is exactly the wall-clock-shaped variability being removed.
         arming:arm_force()
         gcs:send_text(6, string.format("DRIVER %d arm -> %s",
                                        step.at, tostring(arming:is_armed())))

      elseif step.what == "mode" then
         vehicle:set_mode(step.mode)
         gcs:send_text(6, string.format("DRIVER %d mode %d", step.at, step.mode))

      elseif step.what == "disarm" then
         arming:disarm()
         gcs:send_text(6, string.format("DRIVER %d disarm -> %s",
                                        step.at, tostring(arming:is_armed())))
      end

      next_step = next_step + 1
   end

   return update, STEP_MS
end

gcs:send_text(6, "DRIVER loaded")
return update, STEP_MS
