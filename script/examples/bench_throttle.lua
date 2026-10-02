-- Throttle a small motor on a bench stand, from RC or an ADC pot.
--
-- Written for a 3625 1800kv outrunner on 2-4s with nothing attached to the
-- shaft. It is a bring-up script: the point is that the first time a Lua
-- build drives a motor, the thing driving it is small, rate limited, and
-- stops on its own when the input goes away.
--
-- Read the four notes below before using it on anything with a load.
--
-- 1. SET AN ERPM LIMIT FIRST. 1800kv on 4s is about 28,000 rpm unloaded. A
--    12N14P outrunner has 7 pole pairs, so that is close to 200,000 ERPM --
--    far above what the FOC loop is configured for by default. Set
--    l_max_erpm in VESC Tool to something the board is happy with before
--    running this, or the motor will hit the limiter rather than the
--    throttle. This script does not set it, because a script quietly
--    rewriting a motor limit is worse than one that asks.
--
-- 2. IT DISARMS AT START. Nothing is driven until the throttle has been seen
--    below ARM_BELOW. A board that powers up with the throttle held, or a
--    transmitter already at full, does nothing until the stick is returned.
--
-- 3. A STALE INPUT STOPS THE MOTOR AND DISARMS IT. The RC decoder holds its
--    last value, so a transmitter switched off or out of range reads as
--    whatever it last commanded. Losing the input therefore coasts the motor
--    *and* requires the throttle to return to zero before it will drive
--    again -- otherwise a dropout at full throttle would spin up the instant
--    the link came back.
--
-- 4. IT TAKES THE THROTTLE AWAY FROM THE APPS. app_disable_output keeps the
--    ADC or PPM app from driving the motor at the same time. Without it two
--    things fight over the output and the result is neither.

local cfg = {
  input = "ppm",        -- "ppm" for RC, "adc" for a pot on the EXT pin

  max_current = 4.0,    -- amps, hard cap. A 3625 unloaded needs very little
  ramp_up = 6.0,        -- amps per second on the way up; down is immediate
  deadband = 0.05,      -- throttle below this is zero
  stale_s = 0.25,       -- input older than this counts as gone
  arm_below = 0.05,     -- throttle must be under this to arm
  period_ms = 20,       -- 50 Hz, comfortably inside the motor timeout

  adc_min = 0.8,        -- pot voltage at rest
  adc_max = 2.4,        -- pot voltage at full
}

local M = {
  cfg = cfg,
  armed = false,
  current = 0.0,
  reason = "starting",
}

-- Throttle as 0..1, or nil when the input is not trustworthy.
--
-- nil and 0 are different answers here and the caller treats them
-- differently: 0 is a rider asking for nothing, nil is not knowing what the
-- rider is asking for.
function M.read()
  if cfg.input == "ppm" then
    local age = vesc.get_ppm_age()
    if age == nil or age > cfg.stale_s then
      return nil, "rc input stale"
    end
    local v = vesc.get_ppm()
    -- RC is -1..1; only the forward half drives. Clamped at both ends: the
    -- mapping is about the configured centre, so a transmitter set up
    -- outside the configured pulse range can read past 1.
    if v < 0 then v = 0 end
    if v > 1 then v = 1 end
    return v
  end

  local volts = vesc.get_adc(0)
  if volts == nil then
    -- The board has no such pin. get_adc says so rather than handing back
    -- channel 0's voltage, which is what makes this check possible.
    return nil, "no adc pin"
  end

  local span = cfg.adc_max - cfg.adc_min
  if span <= 0 then
    return nil, "adc range misconfigured"
  end

  local v = (volts - cfg.adc_min) / span
  if v < 0 then v = 0 end
  if v > 1 then v = 1 end

  -- A pot reading well below its rest voltage usually means a broken wire,
  -- and a broken wire must not read as "throttle closed" forever -- that is
  -- indistinguishable from working correctly.
  if volts < (cfg.adc_min - 0.3) then
    return nil, "adc below range"
  end

  return v
end

-- One control step. Returns the current commanded, in amps.
function M.step(dt)
  local throttle, why = M.read()

  if throttle == nil then
    -- Coast, and require re-arming. See note 3.
    M.armed = false
    M.current = 0.0
    M.reason = why or "no input"
    vesc.release_motor()
    return 0.0
  end

  if not M.armed then
    if throttle <= cfg.arm_below then
      M.armed = true
      M.reason = "armed"
    else
      M.reason = "waiting for throttle at zero"
      M.current = 0.0
      vesc.release_motor()
      return 0.0
    end
  end

  if throttle < cfg.deadband then
    throttle = 0.0
  end

  local want = throttle * cfg.max_current

  -- Rate limited upward only. Releasing the throttle should not be gradual.
  local step = cfg.ramp_up * dt
  if want > (M.current + step) then
    M.current = M.current + step
  else
    M.current = want
  end

  if M.current > cfg.max_current then
    M.current = cfg.max_current
  end

  M.reason = "running"

  if M.current <= 0.0 then
    -- Zero current and coasting are not the same thing: holding zero current
    -- still regulates, which on a bench stand is a motor that resists being
    -- turned by hand.
    vesc.release_motor()
  else
    vesc.set_current(M.current)
  end

  return M.current
end

-- Driven by the engine's timer, skipped when a test has set BENCH_TEST so it
-- can step M itself.
--
-- This used to be a `while true` loop ending in vesc.sleep(), which does not
-- exist: the Lua engine has no sleep and no clock of any kind, so on hardware
-- that line was "attempt to call a nil value" the first time round. It went
-- unnoticed because the only test path sets BENCH_TEST and never enters the
-- loop. vesc.on_timer is the periodic entry point that does exist, and it
-- also yields to the engine between passes rather than occupying it.
if not BENCH_TEST then
  local last_report = ""

  vesc.on_timer(cfg.period_ms, function()
    -- Keep the apps off the output for as long as this runs, refreshed each
    -- pass so the motor is released if the script dies.
    vesc.app_disable_output(cfg.period_ms * 5)
    M.step(cfg.period_ms / 1000.0)

    if M.reason ~= last_report then
      print(M.reason .. " (" .. string.format("%.2f", M.current) .. " A)")
      last_report = M.reason
    end
  end)
end

return M
