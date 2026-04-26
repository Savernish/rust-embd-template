#![no_std]
#![no_main]

use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};
use fugit::ExtU32;
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Pins,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    timer::Alarm0,
    timer::Alarm,
    pac::interrupt,
    pwm::{Slices, Slice, SliceId, FreeRunning},
    Timer,
};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000;

static TICK: AtomicBool = AtomicBool::new(false);

static SHARED_ALARM: Mutex<RefCell<Option<Alarm0>>> = 
    Mutex::new(RefCell::new(None));

pub enum Led {
    Led1,
    Led2,
    Led3,
}

pub enum DutyStates {
    Min,
    Mid,
    Max,
}

impl DutyStates {
    fn duty(&self) -> u16 {
        match self {
            DutyStates::Min => 1953,   // 1 ms — 0°
            DutyStates::Mid => 2929,   // 1.5 ms — 90°
            DutyStates::Max => 3906,   // 2 ms — 180°
        }
    }
}

struct LedManager<'a> {
    pins: [&'a mut dyn OutputPin<Error = core::convert::Infallible>; 3],
}

impl<'a> LedManager<'a> {
    fn on(&mut self, led:Led) {
        let pin = &mut self.pins[led as usize];
        let _ = pin.set_high();
    }
    fn off(&mut self, led:Led) {
        let pin = &mut self.pins[led as usize];
        let _ = pin.set_low();
    }
    fn set_all_low(&mut self) {
        for pin in self.pins.iter_mut() {
            let _ = pin.set_low();
        }
    }
    fn set_all_high(&mut self) {
        for pin in self.pins.iter_mut() {
            let _ = pin.set_high();
        }
    }
    fn chase_step(&mut self, step: u32) {
        self.set_all_low();
        match step % 4 {
            0 => self.on(Led::Led1),
            1 => self.on(Led::Led2),
            2 => self.on(Led::Led3),
            _ => self.set_all_high(),
        }
    }
}

fn pwm_handle_duty_cycle<S: SliceId>(pwm: &mut Slice<S, FreeRunning>, cycle: u16) { //Handles the PWM duty cycle, 
                    //which controls where the servo goes
                    //we will use the existing alarm to update the duty cycle every 500ms
                    //which will move the servo to a new position.
    let clamped = cycle.min(39062);
    pwm.channel_b.set_duty_cycle(clamped).unwrap(); //Set the duty cycle,
                    //unwrapping to panic if it's out of range
                    //(which it shouldn't be due to the clamp above
}

fn pwm_check_duty_cycle<S: SliceId>(pwm: &mut Slice<S, FreeRunning>, step: u32) { //Checks if the duty cycle's state
                    //which is expressed in DutyState enum
                    //it iterates to the other state in every alarm cycle
                    //uses match to match duty cycle with states
    let state = match step % 3 {
        0 => DutyStates::Min,
        1 => DutyStates::Mid,
        _ => DutyStates::Max,
    };

    pwm_handle_duty_cycle(pwm, state.duty());
}

#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        if let Some(alarm) = SHARED_ALARM.borrow(cs).borrow_mut().as_mut() {
            alarm.clear_interrupt();
            alarm.schedule(250.millis()).unwrap();
        }
    });

    TICK.store(true, Ordering::Relaxed);
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm = pwm_slices.pwm7;

    pwm.set_div_int(64);
    pwm.set_top(39062);
    pwm.enable();
    pwm.channel_b.output_to(pins.gpio15); //The PWM pin is connected to the GPIO15, and the 2040 has the slice 7 set to GPIO15.

    let mut led1 = pins.gpio6.into_push_pull_output();
    let mut led2 = pins.gpio18.into_push_pull_output();
    let mut led3 = pins.gpio8.into_push_pull_output();

    // Hardware timer. Ticks at 1 MHz, reads in microseconds.
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut alarm = timer.alarm_0().unwrap();

    alarm.schedule(500.millis()).unwrap();
    alarm.enable_interrupt();

    critical_section::with(|cs| {
            SHARED_ALARM.borrow(cs).replace(Some(alarm));
    });
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut manager = LedManager {
        pins: [&mut led1, &mut led2, &mut led3],
    };

    let mut state: u32 = 0;

    loop {
        cortex_m::asm::wfi();
        if TICK.load(Ordering::Relaxed) {
            TICK.store(false, Ordering::Relaxed);
            pwm_check_duty_cycle(&mut pwm, state);
            manager.chase_step(state);
            state = state.wrapping_add(1);
        }   
    }
}