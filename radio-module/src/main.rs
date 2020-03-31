//#![cfg_attr(not(test), no_std)]
#![no_main]
#![no_std]
#![allow(deprecated)]

//mod longfi_bindings;

extern crate panic_semihosting;

use hal::{
    exti::TriggerEdge,
    exti::Interrupt,
    gpio::*,
    pac,
    prelude::*,
    rcc::Config,
    spi,
    syscfg,
    serial::{Config as serialConfig, Rx, Serial, Tx},
    stm32::usart1,
    stm32::{USART1},
    delay::Delay,
    time::U32Ext,
};
use cortex_m_semihosting::hprintln;
use cortex_m::peripheral::DWT;
use nb::block;
// use rtfm::{Exclusive, Mutex, app};
//use rtfm::cyccnt::{Instant, U32Ext as _};
// use longfi_bindings::AntennaSwitches;
// use longfi_device;
// use longfi_device::LongFi;
// use longfi_device::{ClientEvent, RfConfig, RfEvent};
use stm32l0xx_hal as hal;
// use communicator::{Message, Channel};
use heapless::consts::*;

#[rtfm::app(device = stm32l0xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources{
        INT: pac::EXTI,
        //static mut INT2: pac::EXTI = ();
        //static mut SX1276_DIO0: gpiob::PB4<Input<PullUp>> = ();
        DistanceButton: gpiob::PB5<Input<PullUp>>,
        PositionButton: gpiob::PB6<Input<PullUp>>,
        #[init([0; 512])]
        BUFFER: [u8; 512],
        // static mut LONGFI: LongFi = ();
        LED: gpiob::PB2<Output<PushPull>>,
        #[init(false)]
        STATE: bool,
        #[init(0)]
        COUNTER_1: u32,
        #[init(0)]
        COUNTER_2: u32,
        TX: Tx<USART1>,
        RX: Rx<USART1>,
     //   ITM: ITM,
        //USART1: *const usart1::RegisterBlock,
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Configure the clock.
        let mut rcc = cx.device.RCC.freeze(Config::hsi16());
        let mut syscfg = syscfg::SYSCFG::new(cx.device.SYSCFG, &mut rcc);
        //let clocks = cx.device.RCC.Clocks;
       // let clocks = rcc2.cfgr.freeze();
       // let clocks = rcc.cfgr.freeze();
        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpioa = cx.device.GPIOA.split(&mut rcc);
        let gpiob = cx.device.GPIOB.split(&mut rcc);
        let gpioc = cx.device.GPIOC.split(&mut rcc);

        

        let exti = cx.device.EXTI;
        //let exti2 = device.EXTI;

        // Configure PB4 as input.
       // let sx1276_dio0 = gpiob.pb4.into_pull_up_input();
        let mut distancebutton = gpiob.pb5.into_pull_up_input();
        let mut positionbutton = gpiob.pb6.into_pull_up_input();

        let mut rx = gpioa.pa10.into_pull_up_input();
        let mut tx = gpioa.pa9.into_push_pull_output();
        // Configure the external interrupt on the falling edge for the pin 2.

        let serial = Serial::usart1(
            cx.device.USART1,
            (tx, rx),
        serialConfig::default(), //.baudrate(115_200.bps())     Is now 9600 (default, 8 bit data 1 stop)
            &mut rcc,   
        ).unwrap();

        let (tx, rx) = serial.split();

        exti.listen(
            &mut syscfg,
            distancebutton.port(),
            distancebutton.pin_number(),
            TriggerEdge::Rising,
        );
        exti.listen(
            &mut syscfg,
            positionbutton.port(),
            positionbutton.pin_number(),
            TriggerEdge::Rising,
        );

        let sck = gpiob.pb3;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        //let nss = gpioa.pa15.into_push_pull_output();
        //longfi_bindings::set_spi_nss(nss);

        // Initialise the SPI peripheral.
        let mut _spi = cx.device
            .SPI1
            .spi((sck, miso, mosi), spi::MODE_0, 1_000_000.hz(), &mut rcc);

        let reset = gpioc.pc0.into_push_pull_output();

        // fn init(_: init::Context) {
        //     rtfm::pend(Interrupt::EXTI4_15);
        //     rtfm::pend(Interrupt::EXTI2_3);
        // }
        //longfi_bindings::set_radio_reset(reset);

        // let ant_sw = AntennaSwitches::new(
            // gpioa.pa1.into_push_pull_output(),
            // gpioc.pc2.into_push_pull_output(),
            // gpioc.pc1.into_push_pull_output(),
        // );

        //longfi_bindings::set_antenna_switch(ant_sw);

        // let en_tcxo = gpioa.pa8.into_push_pull_output();
        // longfi_bindings::set_tcxo_pins(en_tcxo);

        // static mut BINDINGS: longfi_device::BoardBindings = longfi_device::BoardBindings {
            // reset: Some(longfi_bindings::radio_reset),
            // spi_in_out: Some(longfi_bindings::spi_in_out),
            // spi_nss: Some(longfi_bindings::spi_nss),
            // delay_ms: Some(longfi_bindings::delay_ms),
            // get_random_bits: Some(longfi_bindings::get_random_bits),
            // set_antenna_pins: Some(longfi_bindings::set_antenna_pins),
            // set_board_tcxo: Some(longfi_bindings::set_tcxo),
        // }; 

        // let rf_config = RfConfig {
            // oui: 0xBEEF_FEED,
            // device_id: 0xABCD,
        // };

        // let mut longfi_radio = unsafe { LongFi::new(&mut BINDINGS, rf_config).unwrap() };

        // longfi_radio.set_buffer(resources.BUFFER);

        // longfi_radio.receive();

        // Configure PB5 as output.
        let mut led = gpiob.pb2.into_push_pull_output();
       // led.set_high().ok();

       //let usart1 = serial::USART1::ptr();
        // Return the initialised resources.
        init::LateResources {
            INT: exti,
            //INT2: exti2,
            PositionButton: positionbutton,
            DistanceButton: distancebutton,
            LED: led,
            TX: tx,
            RX: rx,
        }
    }

     #[idle(resources = [RX, TX])]
    fn idle(cx: idle::Context) -> ! {
        let rx = cx.resources.RX;
        let tx = cx.resources.TX;
        loop {
            hprintln!("test");
            match block!(rx.read()) {
                Ok(byte) => {
                    hprintln!("Ok {:?}", byte);
                    tx.write(byte).unwrap();
                }
                Err(err) => {
                    hprintln!("Error {:?}", err);
                }
            }
        }
    }



    // #[task(priority = 2, resources = [LED])]
    // fn led_button_event(cx: led_button_event::Context, ledon: bool){
        ////let temp: stm32l0xx_hal::gpio::gpiob::PB2<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>> = *cx.resources.LED;
        // led(cx.resources.LED, ledon);
    // }

    //#[task(binds = EXTI4_15, priority = 1, resources = [DistanceButton, PositionButton, INT], spawn = [led_button_event])]
    // fn EXTI4_15(c: EXTI4_15::Context) {
        // 
      //// let mut distancebutton: gpiob::PB5<Input<PullUp>> = c.resources.DistanceButton;
        // let res = c.resources.DistanceButton.is_high();
        // match res {
            // core::result::Result::Ok(v) => {
                // if v==true {
                    // hprintln!("Hello!_testif").unwrap();
                    // c.resources.INT.clear_irq(c.resources.DistanceButton.pin_number());
                    // c.spawn.led_button_event(false).unwrap();
                // } else {
                    // hprintln!("Hello!_testelse").unwrap();
                    // c.resources.INT.clear_irq(c.resources.PositionButton.pin_number());
                    // c.spawn.led_button_event(true).unwrap();
                // }
            // },
            // _ => {
// 
            // }
        // }
   // }

    // #[task(binds = USART1, priority = 1, resources = [USART1, INT], spawn = [position_button_event])]
    // fn usart(cx: usart::Context){
        // cx.resources.INT.clear_irq(c.resources.USART1.ptr();
        // cx.spawn.usart_event().unwrap();
// 
    // }
    
    // #[task(priority = 2, resources = [USART1])]
    // fn usart_button_event(cx: usart_button_event::Context){
        // 
    // }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
        fn LPTIM1();
    }
};

// fn ledOn(led: &mut gpiob::PB2<Output<PushPull>>) {
    // led.set_high().ok();
// }
// 
// fn ledOff(led: &mut gpiob::PB2<Output<PushPull>>) {
    // led.set_low().ok();
// }
// 
// fn led(led: &mut gpiob::PB2<Output<PushPull>>, turnon: bool,) {
//    
    // if turnon==true {
        // led.set_high().ok();
    // } else {
        // led.set_low().ok();
    // }
// 
// }
// Example application: increment counter:
// fn application(
// 
    // led: &mut gpiob::PB2<Output<PushPull>>,
// ) -> heapless::Vec<u8, U90> {
// 
// 
   // toggle the LED
    // if *state {
        // led.set_low().unwrap();
        // *state = false;
    // } else {
        // led.set_high().unwrap();
        // *state = true;
    // }
// 
    // binary
// }