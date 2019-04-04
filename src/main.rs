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
use core::u8;
use cortex_m::asm;
use hal::PwmPin;
use rtfm::app;
use smart_leds_trait::{Color, SmartLedsWrite};
use stm32f103xx_usb::UsbBus;
use usb_device::bus;
use usb_device::prelude::*;

pub struct PwmLed<PR: PwmPin, PG: PwmPin, PB: PwmPin> {
    r: PwmLedChannel<PR>,
    g: PwmLedChannel<PG>,
    b: PwmLedChannel<PB>,
}

struct PwmLedChannel<P: PwmPin> {
    pin: P,
}

impl<P> PwmLedChannel<P>
where
    P: PwmPin,
    P::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    fn set_brightness(&mut self, value: u8) {
        let max_duty = self.pin.get_max_duty();
        self.pin.set_duty(
            (max_duty.into() * value as u32 / u8::MAX as u32)
                .try_into()
                .unwrap_or(max_duty),
        );
    }
}

impl<PR, PG, PB> PwmLed<PR, PG, PB>
where
    PR: PwmPin,
    PB: PwmPin,
    PG: PwmPin,
    PR::Duty: Into<u32> + TryFrom<u32> + Copy,
    PB::Duty: Into<u32> + TryFrom<u32> + Copy,
    PG::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    pub fn new(r: PR, g: PG, b: PB) -> PwmLed<PR, PG, PB> {
        PwmLed {
            r: PwmLedChannel { pin: r },
            g: PwmLedChannel { pin: g },
            b: PwmLedChannel { pin: b },
        }
    }
}

impl<PR, PG, PB> SmartLedsWrite for PwmLed<PR, PG, PB>
where
    PR: PwmPin,
    PB: PwmPin,
    PG: PwmPin,
    PR::Duty: Into<u32> + TryFrom<u32> + Copy,
    PB::Duty: Into<u32> + TryFrom<u32> + Copy,
    PG::Duty: Into<u32> + TryFrom<u32> + Copy,
{
    type Error = ();
    fn write<T>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = Color>,
    {
        for item in iterator {
            self.r.set_brightness(item.r);
            self.g.set_brightness(item.g);
            self.b.set_brightness(item.b);
        }
        Ok(())
    }
}

#[app(device = maple_mini::stm32)]
const APP: () = {
    static mut LED: LedPin<Output<PushPull>> = ();
    /*static mut PWM: <(
        D11<Alternate<PushPull>>,
        D10<Alternate<PushPull>>,
        D9<Alternate<PushPull>>,
        D8<Alternate<PushPull>>,
    ) as Pins<TIM2>>::Channels = ();
    */
    static mut PWMLED: PwmLed<
        maple_mini::pwm::Pwm<TIM2, C3>,
        maple_mini::pwm::Pwm<TIM2, C4>,
        maple_mini::pwm::Pwm<TIM2, C1>,
    > = ();
    static mut USB_DEV: UsbDevice<'static, UsbBus> = ();
    static mut SERIAL: cdc_acm::SerialPort<'static, UsbBus> = ();

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

        timer::Timer::syst(core.SYST, 6.hz(), clocks).listen(timer::Event::Update);

        // setup PWM

        let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
        let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

        let mut pwm = device.TIM2.pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            1.khz(),
            clocks,
            &mut rcc.apb1,
        );

        pwm.0.enable(); // blue
        pwm.1.enable(); // white
        pwm.2.enable(); // red
        pwm.3.enable(); // green

        let led = PwmLed::new(pwm.2, pwm.3, pwm.0);

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

    #[exception(resources = [PWMLED], binds=SysTick)]
    fn set_color() {
        //let colors: [Color; 1] = [Color { r: 0, g: 0, b: 255 }]; //, Color { r: 0, g: 0, b: 255 }];
        //resources.PWMLED.write(colors.iter().cloned());
    }

    #[interrupt(resources = [USB_DEV, SERIAL, PWMLED])]
    fn USB_HP_CAN_TX() {
        usb_poll(
            &mut resources.USB_DEV,
            &mut resources.SERIAL,
            &mut resources.PWMLED,
        );
    }

    #[interrupt(resources = [USB_DEV, SERIAL, PWMLED])]
    fn USB_LP_CAN_RX0() {
        usb_poll(
            &mut resources.USB_DEV,
            &mut resources.SERIAL,
            &mut resources.PWMLED,
        );
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut cdc_acm::SerialPort<'static, B>,
    led: &mut PwmLed<
        maple_mini::pwm::Pwm<TIM2, C3>,
        maple_mini::pwm::Pwm<TIM2, C4>,
        maple_mini::pwm::Pwm<TIM2, C1>,
    >,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 3];

    match serial.read(&mut buf) {
        Ok(count) if count == 3 => {
            //serial.write(&buf).ok();
            led.write([buf.into()].iter().cloned()).ok();
        }
        _ => {}
    }
}
