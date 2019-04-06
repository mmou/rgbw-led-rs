#![no_main]
#![no_std]

extern crate embedded_hal as hal;
extern crate panic_semihosting;

mod cdc_acm;

use maple_mini::{
    gpio::{gpiob::*, Alternate, Output, PushPull},
    pins::*,
    prelude::*,
    pwm::{Pins, Pwm, C1, C2, C3, C4},
    stm32::TIM2,
    timer,
};

use core::convert::{TryFrom, TryInto};
use core::u16;
use cortex_m::asm;
use ditherable_leds::{typenum::U1, DitherableLeds, GAMMA_2_2_LUT};
use hal::PwmPin;
use rgb::FromSlice;
use rtfm::app;
use smart_leds_trait::{SmartLedsWrite, RGBW16, RGBW8};
use stm32f103xx_usb::UsbBus;
use usb_device::bus;
use usb_device::prelude::*;

pub struct PwmLed<PR: PwmPin, PG: PwmPin, PB: PwmPin, PW: PwmPin> {
    r: PwmLedChannel<PR>,
    g: PwmLedChannel<PG>,
    b: PwmLedChannel<PB>,
    w: PwmLedChannel<PW>,
}

struct PwmLedChannel<P: PwmPin> {
    pin: P,
}

impl<P> PwmLedChannel<P>
where
    P: PwmPin,
    P::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    fn set_brightness(&mut self, value: u16) {
        let max_duty = self.pin.get_max_duty();
        self.pin.set_duty(
            (max_duty.into() * value as u32 / u16::MAX as u32)
                .try_into()
                .unwrap_or(max_duty),
        );
    }
}

impl<PR, PG, PB, PW> PwmLed<PR, PG, PB, PW>
where
    PR: PwmPin,
    PB: PwmPin,
    PG: PwmPin,
    PW: PwmPin,
    PR::Duty: Into<u32> + TryFrom<u32> + Copy,
    PB::Duty: Into<u32> + TryFrom<u32> + Copy,
    PG::Duty: Into<u32> + TryFrom<u32> + Copy,
    PW::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    pub fn new(r: PR, g: PG, b: PB, w: PW) -> PwmLed<PR, PG, PB, PW> {
        PwmLed {
            r: PwmLedChannel { pin: r },
            g: PwmLedChannel { pin: g },
            b: PwmLedChannel { pin: b },
            w: PwmLedChannel { pin: w },
        }
    }
}

impl<PR, PG, PB, PW> SmartLedsWrite for PwmLed<PR, PG, PB, PW>
where
    PR: PwmPin,
    PB: PwmPin,
    PG: PwmPin,
    PW: PwmPin,
    PR::Duty: Into<u32> + TryFrom<u32> + Copy,
    PB::Duty: Into<u32> + TryFrom<u32> + Copy,
    PG::Duty: Into<u32> + TryFrom<u32> + Copy,
    PW::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    type Pixel = RGBW16;
    type Error = ();
    fn write<T, I>(&mut self, mut iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = I>,
        I: Into<RGBW16>,
    {
        if let Some(rgbw) = iterator.next() {
            let rgbw = rgbw.into();
            self.r.set_brightness(rgbw.r);
            self.g.set_brightness(rgbw.g);
            self.b.set_brightness(rgbw.b);
            self.w.set_brightness(rgbw.a);
        }
        Ok(())
    }
}

#[app(device = maple_mini::stm32)]
const APP: () = {
    static mut LED: LedPin<Output<PushPull>> = ();
    static mut PWMLED: DitherableLeds<
        PwmLed<
            maple_mini::pwm::Pwm<TIM2, C3>,
            maple_mini::pwm::Pwm<TIM2, C4>,
            maple_mini::pwm::Pwm<TIM2, C1>,
            maple_mini::pwm::Pwm<TIM2, C2>,
        >,
        U1,
    > = ();
    static mut USB_DEV: UsbDevice<'static, UsbBus> = ();
    static mut SERIAL: cdc_acm::SerialPort<'static, UsbBus> = ();
    static mut TIME: u32 = 0;

    #[init]
    fn init() {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBus>> = None;

        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        //let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        timer::Timer::syst(core.SYST, 600.hz(), clocks).listen(timer::Event::Update);

        // setup PWM

        let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
        let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

        let mut pwm = device.TIM2.pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            30.khz(),
            clocks,
            &mut rcc.apb1,
        );

        pwm.0.enable(); // blue
        pwm.1.enable(); // white
        pwm.2.enable(); // red
        pwm.3.enable(); // green

        let led = PwmLed::new(pwm.2, pwm.3, pwm.0, pwm.1);
        let mut led: DitherableLeds<_, U1> = DitherableLeds::new(led, Some(&GAMMA_2_2_LUT));

        // setup USB

        *USB_BUS = Some(UsbBus::usb_with_reset(
            device.USB,
            &mut rcc.apb1,
            &clocks,
            &mut gpiob.crh,
            gpiob.pb9,
        ));

        let serial = cdc_acm::SerialPort::new(USB_BUS.as_ref().unwrap());

        let mut usb_dev =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x5824, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(cdc_acm::USB_CLASS_CDC)
                .build();

        usb_dev.force_reset().expect("reset failed");

        // initialize

        USB_DEV = usb_dev;
        SERIAL = serial;
        LED = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        PWMLED = led;
    }

    #[exception(resources = [TIME, PWMLED], binds=SysTick)]
    fn inc_time() {
        *resources.TIME += 1;
        resources.PWMLED.now(*resources.TIME).flush();
    }

    #[interrupt(resources = [USB_DEV, SERIAL, PWMLED, TIME])]
    fn USB_HP_CAN_TX() {
        if let Some(color) = usb_poll(&mut resources.USB_DEV, &mut resources.SERIAL) {
            resources
                .PWMLED
                .now(*resources.TIME)
                .write([color].iter().cloned())
                .ok();
        }
    }

    #[interrupt(resources = [USB_DEV, SERIAL, PWMLED, TIME])]
    fn USB_LP_CAN_RX0() {
        if let Some(color) = usb_poll(&mut resources.USB_DEV, &mut resources.SERIAL) {
            resources
                .PWMLED
                .now(*resources.TIME)
                .write([color].iter().cloned())
                .ok();
        }
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut cdc_acm::SerialPort<'static, B>,
) -> Option<RGBW8> {
    if !usb_dev.poll(&mut [serial]) {
        return None;
    }

    let mut buf = [0u8; 4];

    match serial.read(&mut buf) {
        Ok(count) if count >= 3 => {
            let color: RGBW8 = buf.into();
            return Some(color);
        }
        _ => {}
    }

    return None;
}
