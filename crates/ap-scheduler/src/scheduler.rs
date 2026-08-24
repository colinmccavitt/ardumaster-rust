//! Port of `AP_Scheduler`, pinned to `Plane-4.7.0`.
//!
//! The tick-ordering contract for the whole vehicle. This is load-bearing
//! behavior, not plumbing: which task runs, in what order, and what happens
//! when the loop runs out of time are all observable in flight.
//!
//! # The contract
//!
//! Two task tables — vehicle-specific and common — are merged by ascending
//! `priority`, **and a tie goes to the vehicle task**. Tasks with priority at
//! or below [`MAX_FAST_TASK_PRIORITIES`] are "fast tasks": they run every loop
//! regardless of budget. Everything else is rate-limited and skipped when it
//! would not fit in the time remaining.
//!
//! For ArduPlane the fast tasks are, in order:
//! `ahrs_update` → `update_control_mode` → `stabilize` → `set_servos`.
//! That ordering is a data dependency, not a preference — AHRS state must be
//! fresh before the control mode reads it, and servos must be written last in
//! the same tick.
//!
//! # Running out of time does not stop the walk
//!
//! When a task overruns, upstream sets the remaining budget to zero but
//! **keeps walking the table**, so later fast tasks still run and the
//! accounting stays correct. Returning early would silently drop `set_servos`
//! in exactly the overloaded conditions where it matters most. Preserved, and
//! pinned by a test.
//!
//! # Shape changes (ADR-0004)
//!
//! - Tasks are `fn(&mut V)` over a vehicle type rather than bound member
//!   functors, and the vehicle is passed to [`Scheduler::run`]. This is the
//!   direct translation of upstream's `SCHED_TASK_CLASS(Plane, &plane, func)`
//!   without the global instance.
//! - `last_run` state is a caller-provided slice, since `no_std` without an
//!   allocator cannot size it internally.
//! - Time comes from a [`Clock`], not `AP_HAL::micros()`.

use ap_hal::internal_error::{InternalError, InternalErrors};
use ap_hal::time::Clock;

/// Tasks with priority at or below this run every loop, budget permitting or
/// not. Upstream `MAX_FAST_TASK_PRIORITIES`.
pub const MAX_FAST_TASK_PRIORITIES: u8 = 3;

/// A task running later than this multiple of its interval counts as not
/// achieved, which upstream uses to decide the loop needs more time.
/// Upstream `max_task_slowdown`.
pub const MAX_TASK_SLOWDOWN: u16 = 4;

/// Rate value meaning "run at the loop rate". Upstream `LOOP_RATE` is 0.
pub const LOOP_RATE: f32 = 0.0;

/// One scheduler table entry. Upstream `AP_Scheduler::Task`.
pub struct Task<V> {
    /// The function to run. Upstream binds an instance and member function;
    /// here the vehicle is passed in explicitly.
    pub function: fn(&mut V),
    /// Task name, used in overrun diagnostics.
    pub name: &'static str,
    /// Requested rate in Hz. [`LOOP_RATE`] (0) means every loop.
    pub rate_hz: f32,
    /// Expected worst-case runtime, in microseconds. A task is skipped when
    /// this exceeds the time remaining.
    pub max_time_micros: u16,
    /// Ordering key, ascending. Ties between the two tables go to the vehicle.
    pub priority: u8,
}

/// What one [`Scheduler::run`] call did, for accounting and tests.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RunStats {
    /// Tasks actually executed.
    pub tasks_run: u16,
    /// Tasks skipped because their budget exceeded the time remaining.
    pub tasks_skipped_for_time: u16,
    /// Tasks that ran at least one whole interval late. Upstream
    /// `perf_info.task_slipped()`.
    pub tasks_slipped: u16,
    /// Tasks later than [`MAX_TASK_SLOWDOWN`] intervals. Upstream
    /// `task_not_achieved`, which drives lowering the loop rate.
    pub tasks_not_achieved: u16,
    /// Tasks that took longer than their declared budget.
    pub overruns: u16,
}

/// The task scheduler. Upstream `AP_Scheduler`.
pub struct Scheduler<'a, V> {
    vehicle_tasks: &'a [Task<V>],
    common_tasks: &'a [Task<V>],
    last_run: &'a mut [u16],
    tick_counter: u16,
    loop_rate_hz: u16,
    /// Internal errors raised while scheduling.
    pub errors: InternalErrors,
}

impl<'a, V> Scheduler<'a, V> {
    /// Build a scheduler over two task tables.
    ///
    /// `last_run` must have at least one entry per task; a shorter slice makes
    /// [`Scheduler::run`] report [`InternalError::InvalidArgOrResult`] rather
    /// than indexing out of bounds, which is what upstream's raw array would
    /// do.
    pub fn new(
        vehicle_tasks: &'a [Task<V>],
        common_tasks: &'a [Task<V>],
        last_run: &'a mut [u16],
        loop_rate_hz: u16,
    ) -> Self {
        Self {
            vehicle_tasks,
            common_tasks,
            last_run,
            tick_counter: 0,
            loop_rate_hz,
            errors: InternalErrors::none(),
        }
    }

    /// Total tasks across both tables.
    pub fn num_tasks(&self) -> usize {
        self.vehicle_tasks.len() + self.common_tasks.len()
    }

    /// Loop period in microseconds. Upstream `get_loop_period_us()`.
    pub fn loop_period_us(&self) -> u32 {
        if self.loop_rate_hz == 0 {
            return 0;
        }
        1_000_000u32 / self.loop_rate_hz as u32
    }

    /// Ticks elapsed. Upstream `_tick_counter`.
    pub fn tick_counter(&self) -> u16 {
        self.tick_counter
    }

    /// Advance the tick counter. Upstream `tick()`.
    pub fn tick(&mut self) {
        self.tick_counter = self.tick_counter.wrapping_add(1);
    }

    /// Interval in ticks for a task's requested rate.
    ///
    /// Upstream: a rate of 0 means the loop rate, and the result is clamped to
    /// at least 1 so a rate above the loop rate does not mean "never".
    fn interval_ticks(&self, rate_hz: f32) -> u16 {
        if rate_hz == 0.0 {
            return 1;
        }
        let ticks = (self.loop_rate_hz as f32 / rate_hz) as i32;
        if ticks < 1 {
            1
        } else {
            ticks as u16
        }
    }

    /// Run one scheduler pass over both tables.
    ///
    /// Upstream `AP_Scheduler::run(time_available)`.
    pub fn run(&mut self, vehicle: &mut V, clock: &dyn Clock, time_available_us: u32) -> RunStats {
        let mut stats = RunStats::default();
        let mut time_available = time_available_us;
        let mut now = clock.micros();

        let mut vehicle_off = 0usize;
        let mut common_off = 0usize;

        for i in 0..self.num_tasks() {
            // Merge the two tables by priority. A tie goes to the vehicle
            // task, which is what lets a vehicle override a common task's
            // position without renumbering the common table.
            let run_vehicle = match (
                vehicle_off < self.vehicle_tasks.len(),
                common_off < self.common_tasks.len(),
            ) {
                (true, true) => {
                    let v = match self.vehicle_tasks.get(vehicle_off) {
                        Some(t) => t,
                        None => break,
                    };
                    let c = match self.common_tasks.get(common_off) {
                        Some(t) => t,
                        None => break,
                    };
                    v.priority <= c.priority
                }
                (true, false) => true,
                (false, true) => false,
                (false, false) => {
                    // upstream: the outer loop should have terminated already
                    self.errors.report(InternalError::FlowOfControl);
                    break;
                }
            };

            let task = if run_vehicle {
                let t = match self.vehicle_tasks.get(vehicle_off) {
                    Some(t) => t,
                    None => break,
                };
                vehicle_off += 1;
                t
            } else {
                let t = match self.common_tasks.get(common_off) {
                    Some(t) => t,
                    None => break,
                };
                common_off += 1;
                t
            };

            let last = match self.last_run.get(i) {
                Some(v) => *v,
                None => {
                    // caller gave too small a state slice
                    self.errors.report(InternalError::InvalidArgOrResult);
                    break;
                }
            };

            let task_time_allowed: u32;

            if task.priority > MAX_FAST_TASK_PRIORITIES {
                let dt = self.tick_counter.wrapping_sub(last);
                let interval = self.interval_ticks(task.rate_hz);

                if dt < interval {
                    // not yet due
                    continue;
                }

                if dt >= interval.saturating_mul(2) {
                    stats.tasks_slipped += 1;
                }
                if dt >= interval.saturating_mul(MAX_TASK_SLOWDOWN) {
                    stats.tasks_not_achieved += 1;
                }

                task_time_allowed = task.max_time_micros as u32;
                if task_time_allowed > time_available {
                    // Not enough time for this one, but keep going: a cheaper
                    // task later in the table may still fit.
                    stats.tasks_skipped_for_time += 1;
                    continue;
                }
            } else {
                // fast task: runs regardless of remaining budget
                task_time_allowed = self.loop_period_us();
            }

            let started = now;
            (task.function)(vehicle);
            stats.tasks_run += 1;

            if let Some(slot) = self.last_run.get_mut(i) {
                *slot = self.tick_counter;
            }

            now = clock.micros();
            let taken = now.since(started);
            if taken > task_time_allowed {
                stats.overruns += 1;
            }

            if taken >= time_available {
                // Out of time, but keep walking the table so later fast tasks
                // still run and the accounting stays correct. Returning here
                // would drop set_servos exactly when the loop is overloaded.
                time_available = 0;
            } else {
                time_available -= taken;
            }
        }

        stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_hal::time::{Micros, Millis};
    use core::cell::Cell;

    /// A clock the test advances by a fixed amount per task, so runtimes are
    /// deterministic. This is only possible because time is injected.
    struct StepClock {
        us: Cell<u32>,
        step: Cell<u32>,
    }

    impl StepClock {
        fn new(step: u32) -> Self {
            Self {
                us: Cell::new(0),
                step: Cell::new(step),
            }
        }
    }

    impl Clock for StepClock {
        fn millis(&self) -> Millis {
            Millis(self.us.get() / 1000)
        }
        fn micros(&self) -> Micros {
            // every observation advances the clock, modelling elapsed runtime
            let v = self.us.get();
            self.us.set(v.wrapping_add(self.step.get()));
            Micros(v)
        }
        fn millis64(&self) -> u64 {
            self.us.get() as u64 / 1000
        }
        fn micros64(&self) -> u64 {
            self.us.get() as u64
        }
    }

    #[derive(Default)]
    struct Vehicle {
        log: [u8; 16],
        n: usize,
    }

    impl Vehicle {
        fn record(&mut self, id: u8) {
            if let Some(slot) = self.log.get_mut(self.n) {
                *slot = id;
                self.n += 1;
            }
        }
        fn order(&self) -> &[u8] {
            self.log.get(..self.n).unwrap_or(&[])
        }
        fn clear(&mut self) {
            self.n = 0;
        }
    }

    fn t_ahrs(v: &mut Vehicle) {
        v.record(1);
    }
    fn t_mode(v: &mut Vehicle) {
        v.record(2);
    }
    fn t_stab(v: &mut Vehicle) {
        v.record(3);
    }
    fn t_servos(v: &mut Vehicle) {
        v.record(4);
    }
    fn t_slow(v: &mut Vehicle) {
        v.record(9);
    }

    /// The four ArduPlane fast tasks, in upstream's order.
    fn fast_tasks() -> [Task<Vehicle>; 4] {
        [
            Task {
                function: t_ahrs,
                name: "ahrs_update",
                rate_hz: LOOP_RATE,
                max_time_micros: 0,
                priority: 0,
            },
            Task {
                function: t_mode,
                name: "update_control_mode",
                rate_hz: LOOP_RATE,
                max_time_micros: 0,
                priority: 1,
            },
            Task {
                function: t_stab,
                name: "stabilize",
                rate_hz: LOOP_RATE,
                max_time_micros: 0,
                priority: 2,
            },
            Task {
                function: t_servos,
                name: "set_servos",
                rate_hz: LOOP_RATE,
                max_time_micros: 0,
                priority: 3,
            },
        ]
    }

    /// The ordering contract: AHRS before control mode before stabilize before
    /// servos, every single loop.
    #[test]
    fn fast_tasks_run_every_loop_in_priority_order() {
        let tasks = fast_tasks();
        let mut last = [0u16; 4];
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);

        for _ in 0..3 {
            v.clear();
            s.tick();
            let stats = s.run(&mut v, &clock, 2500);
            assert_eq!(v.order(), &[1, 2, 3, 4], "fast task order must not vary");
            assert_eq!(stats.tasks_run, 4);
        }
    }

    /// Rate-limited tasks run only on their interval. At 400 Hz loop and 50 Hz
    /// task the interval is 8 ticks.
    #[test]
    fn rate_limited_tasks_respect_their_interval() {
        let tasks = [Task {
            function: t_slow,
            name: "slow",
            rate_hz: 50.0,
            max_time_micros: 100,
            priority: 10,
        }];
        let mut last = [0u16; 1];
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);

        assert_eq!(s.interval_ticks(50.0), 8);

        let mut runs = 0;
        for _ in 0..24 {
            s.tick();
            runs += s.run(&mut v, &clock, 2500).tasks_run;
        }
        assert_eq!(runs, 3, "24 ticks at an 8-tick interval is 3 runs");
    }

    /// A rate above the loop rate clamps to every loop rather than never.
    #[test]
    fn rate_above_loop_rate_clamps_to_one_tick() {
        let tasks: [Task<Vehicle>; 0] = [];
        let mut last = [0u16; 0];
        let mut s = Scheduler::<Vehicle>::new(&tasks, &[], &mut last, 50);
        assert_eq!(s.interval_ticks(400.0), 1);
        assert_eq!(s.interval_ticks(LOOP_RATE), 1);
        s.tick();
    }

    /// Ties between the tables go to the vehicle task.
    #[test]
    fn priority_tie_goes_to_the_vehicle_task() {
        let vehicle_tasks = [Task {
            function: t_ahrs,
            name: "vehicle",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 5,
        }];
        let common_tasks = [Task {
            function: t_slow,
            name: "common",
            rate_hz: LOOP_RATE,
            max_time_micros: 0,
            priority: 5,
        }];
        let mut last = [0u16; 2];
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&vehicle_tasks, &common_tasks, &mut last, 400);
        s.tick();
        s.run(&mut v, &clock, 5000);
        assert_eq!(v.order(), &[1, 9], "vehicle task wins the tie");
    }

    /// A task whose budget exceeds the time remaining is skipped, but the walk
    /// continues so a cheaper task later can still run.
    #[test]
    fn expensive_task_is_skipped_but_walk_continues() {
        let tasks = [
            Task {
                function: t_slow,
                name: "expensive",
                rate_hz: LOOP_RATE,
                max_time_micros: 5000,
                priority: 10,
            },
            Task {
                function: t_servos,
                name: "cheap",
                rate_hz: LOOP_RATE,
                max_time_micros: 50,
                priority: 11,
            },
        ];
        let mut last = [0u16; 2];
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);
        s.tick();

        let stats = s.run(&mut v, &clock, 1000);
        assert_eq!(v.order(), &[4], "only the cheap task fits");
        assert_eq!(stats.tasks_skipped_for_time, 1);
        assert_eq!(stats.tasks_run, 1);
    }

    /// The critical one: when a task overruns the whole budget, later FAST
    /// tasks must still run. Returning early would drop set_servos exactly
    /// when the loop is overloaded.
    #[test]
    fn overrun_does_not_stop_later_fast_tasks() {
        let tasks = fast_tasks();
        let mut last = [0u16; 4];
        // Each micros() observation burns 3000us. That exhausts the 1000us
        // pass budget immediately AND exceeds the 2500us per-fast-task budget
        // at 400Hz, so both the overrun and the keep-walking behaviour are
        // exercised. (2000us would exhaust the pass but stay inside the
        // per-task budget, recording no overrun.)
        let clock = StepClock::new(3000);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);
        s.tick();

        let stats = s.run(&mut v, &clock, 1000);
        assert_eq!(
            v.order(),
            &[1, 2, 3, 4],
            "all fast tasks must run even with no budget left"
        );
        assert_eq!(stats.tasks_run, 4);
        assert!(stats.overruns > 0, "the overrun must still be recorded");
    }

    /// Slip and not-achieved accounting, which upstream uses to decide the loop
    /// rate needs lowering.
    #[test]
    fn slip_and_not_achieved_are_counted() {
        let tasks = [Task {
            function: t_slow,
            name: "slow",
            rate_hz: 50.0, // 8-tick interval at 400Hz
            max_time_micros: 100,
            priority: 10,
        }];
        let mut last = [0u16; 1];
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);

        // jump far past the interval: 32 ticks is 4x the 8-tick interval
        for _ in 0..32 {
            s.tick();
        }
        let stats = s.run(&mut v, &clock, 5000);
        assert_eq!(stats.tasks_run, 1);
        assert_eq!(stats.tasks_slipped, 1, "ran more than 2 intervals late");
        assert_eq!(
            stats.tasks_not_achieved, 1,
            "ran at least MAX_TASK_SLOWDOWN intervals late"
        );
    }

    /// A too-small last_run slice is reported rather than indexing out of
    /// bounds, which upstream's raw array would do.
    #[test]
    fn undersized_state_slice_is_reported() {
        let tasks = fast_tasks();
        let mut last = [0u16; 2]; // 4 tasks, only 2 slots
        let clock = StepClock::new(0);
        let mut v = Vehicle::default();
        let mut s = Scheduler::new(&tasks, &[], &mut last, 400);
        s.tick();
        s.run(&mut v, &clock, 5000);
        assert!(s.errors.contains(InternalError::InvalidArgOrResult));
    }

    #[test]
    fn loop_period_matches_rate() {
        let tasks: [Task<Vehicle>; 0] = [];
        let mut last = [0u16; 0];
        let s = Scheduler::<Vehicle>::new(&tasks, &[], &mut last, 400);
        assert_eq!(s.loop_period_us(), 2500);
    }
}
