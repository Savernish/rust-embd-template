#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use fugit::ExtU32;
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Pins,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Timer,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000;

pub enum Led {
    Led1,
    Led2,
    Led3,
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

    let mut led1 = pins.gpio6.into_push_pull_output();
    let mut led2 = pins.gpio18.into_push_pull_output();
    let mut led3 = pins.gpio14.into_push_pull_output();

    // Hardware timer. Ticks at 1 MHz, reads in microseconds.
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut manager = LedManager {
        pins: [&mut led1, &mut led2, &mut led3],
    };

    // Ignore the unused-import warning from earlier versions.
    let _ = clocks.system_clock.freq();

    loop {
        manager.set_all_low();
        manager.on(Led::Led1);
        timer.delay_ms(500u32);
        manager.off(Led::Led1);
        manager.on(Led::Led2);
        timer.delay_ms(500u32);
        manager.off(Led::Led2);
        manager.on(Led::Led3);
        timer.delay_ms(500u32);
        manager.set_all_high();
        timer.delay_ms(300u32);
    }
}