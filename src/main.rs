#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use embedded_hal::{digital::OutputPin, i2c::I2c, pwm::SetDutyCycle};
use fugit::{ExtU32, /*MicrosDurationU32,*/ RateExtU32};
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls /*Clock*/},
    gpio::{FunctionI2C, Pin, Pins, PullUp},
    i2c::I2C,
    pac,
    pac::interrupt,
    pwm::{FreeRunning, Slice, SliceId, Slices},
    sio::Sio,
    timer::Alarm,
    timer::Alarm0,
    watchdog::Watchdog,
    Timer,
};

use critical_section::Mutex;

type ButtonPin = rp2040_hal::gpio::Pin<
    rp2040_hal::gpio::bank0::Gpio10,
    rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioInput>,
    rp2040_hal::gpio::PullUp,
>;

type I2cBus = I2C<
    pac::I2C0,
    (
        Pin<rp2040_hal::gpio::bank0::Gpio4, FunctionI2C, PullUp>,
        Pin<rp2040_hal::gpio::bank0::Gpio5, FunctionI2C, PullUp>,
    ),
>;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000;

static TICK: AtomicBool = AtomicBool::new(false);

static SHARED_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static SHARED_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));

static SHARED_I2C: Mutex<RefCell<Option<I2cBus>>> = Mutex::new(RefCell::new(None));

static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);

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
            DutyStates::Min => 1953, // 1 ms — 0°
            DutyStates::Mid => 2929, // 1.5 ms — 90°
            DutyStates::Max => 3906, // 2 ms — 180°
        }
    }
}

struct LedManager<'a> {
    pins: [&'a mut dyn OutputPin<Error = core::convert::Infallible>; 3],
}

impl<'a> LedManager<'a> {
    fn on(&mut self, led: Led) {
        let pin = &mut self.pins[led as usize];
        let _ = pin.set_high();
    }
    fn off(&mut self, led: Led) {
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

fn pwm_handle_duty_cycle<S: SliceId>(pwm: &mut Slice<S, FreeRunning>, cycle: u16) {
    //Handles the PWM duty cycle,
    //which controls where the servo goes
    //we will use the existing alarm to update the duty cycle every 500ms
    //which will move the servo to a new position.
    let clamped = cycle.min(39062);
    pwm.channel_b.set_duty_cycle(clamped).unwrap(); //Set the duty cycle,
                                                    //unwrapping to panic if it's out of range
                                                    //(which it shouldn't be due to the clamp above
}

fn pwm_check_duty_cycle<S: SliceId>(pwm: &mut Slice<S, FreeRunning>, step: u32) {
    //Checks if the duty cycle's state
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
            alarm.schedule(400.millis()).unwrap();
        }
    });

    TICK.store(true, Ordering::Relaxed);
}

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        if let Some(button) = SHARED_BUTTON.borrow(cs).borrow_mut().as_mut() {
            // ack the interrupt for this specific pin and edge type
            button.clear_interrupt(rp2040_hal::gpio::Interrupt::EdgeLow);
        }
    });

    BUTTON_PRESSED.store(true, Ordering::Relaxed);
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

    let button = pins.gpio10.into_pull_up_input();
    button.set_interrupt_enabled(rp2040_hal::gpio::Interrupt::EdgeLow, true);

    //I2C Button configrution
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio5.reconfigure();
    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio4.reconfigure();

    let mut i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin, // sda
        scl_pin, // scl
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

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

    critical_section::with(|cs| {
        SHARED_BUTTON.borrow(cs).replace(Some(button));
    });

    critical_section::with(|cs| {
        SHARED_I2C.borrow(cs).replace(Some(i2c));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    let mut manager = LedManager {
        pins: [&mut led1, &mut led2, &mut led3],
    };

    let mut state: u32 = 0;

    critical_section::with(|cs| {
        if let Some(i2c) = SHARED_I2C.borrow(cs).borrow_mut().as_mut() {
            // Wake the chip: write 0x00 to PWR_MGMT_1 (register 0x6B).
            i2c.write(0x68u8, &[0x6B, 0x00]).unwrap();

            // Read WHO_AM_I (register 0x75) — must be 0x68.
            let mut buf = [0u8; 1];
            i2c.write_read(0x68u8, &[0x75], &mut buf).unwrap();

            if buf[0] != 0x68 {
                // chip not responding correctly; halt
                loop {}
            }
        }
    });

    loop {
        cortex_m::asm::wfi();
        if TICK.load(Ordering::Relaxed) {
            TICK.store(false, Ordering::Relaxed);
            pwm_check_duty_cycle(&mut pwm, state);
            state = state.wrapping_add(1);
            if BUTTON_PRESSED.load(Ordering::Relaxed) {
                BUTTON_PRESSED.store(false, Ordering::Relaxed);
                manager.set_all_high(); // visual signal: all LEDs flash
            } else {
                let mut buf = [0u8; 14];
                critical_section::with(|cs| {
                    if let Some(i2c) = SHARED_I2C.borrow(cs).borrow_mut().as_mut() {
                        // Read 14 bytes starting at ACCEL_XOUT_H (0x3B):
                        // accel X/Y/Z (6), temp (2), gyro X/Y/Z (6).
                        i2c.write_read(0x68u8, &[0x3B], &mut buf).unwrap();
                    }
                });
                let accel_x = i16::from_be_bytes([buf[0], buf[1]]);
                //let accel_y = i16::from_be_bytes([buf[2], buf[3]]);
                //let accel_z = i16::from_be_bytes([buf[4], buf[5]]);

                manager.set_all_low();
                if accel_x > 4000 {
                    manager.on(Led::Led1);
                } else if accel_x < -4000 {
                    manager.on(Led::Led3);
                } else {
                    manager.on(Led::Led2);
                }
            }
        }
    }
}
