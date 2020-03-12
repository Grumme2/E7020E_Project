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
    delay::Delay,
};
use cortex_m_semihosting::hprintln;
use cortex_m::peripheral::DWT;
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
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Configure the clock.
        let mut rcc = cx.device.RCC.freeze(Config::hsi16());
        let mut syscfg = syscfg::SYSCFG::new(cx.device.SYSCFG, &mut rcc);

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
        // Configure the external interrupt on the falling edge for the pin 2.
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

        // Return the initialised resources.
        init::LateResources {
            INT: exti,
            //INT2: exti2,
            PositionButton: positionbutton,
            DistanceButton: distancebutton,
            LED: led,
        }
    }

    // #[task(capacity = 4, priority = 2, resources = [BUFFER, LED, STATE, COUNTER_1, COUNTER_2])]
    // fn radio_event(event: RfEvent) {
    //     // let longfi_radio = resources.LONGFI;
    //     // let client_event = longfi_radio.handle_event(event);
    //     match client_event {
    //         // ClientEvent::ClientEvent_TxDone => {
    //             // longfi_radio.receive();
    //         // }
    //          ClientEvent::ClientEvent_Rx => {
    //             // let rx_packet = longfi_radio.get_rx();

                
    //                 // let buf = unsafe {
    //                     // core::slice::from_raw_parts(rx_packet.buf, rx_packet.len as usize)
    //                 // };
    //                 //let message = Message::deserialize(buf);

    //                 // if let Some(message) = message {
    //                 //     // Let's assume we only have permission to use ID 2:
    //                 //     if message.id != 2 {
    //                 //         longfi_radio.set_buffer(resources.BUFFER);
    //                 //         longfi_radio.receive();
    //                 //         return;
    //                 //     }

    //                     let binary = application(
    //                         // message,
    //                         // resources.COUNTER_1,
    //                         // resources.COUNTER_2,
    //                         // resources.STATE,
    //                         resources.LED
    //                     );
    //                     // longfi_radio.send(&binary);
    //                 // }
                

    //             // longfi_radio.set_buffer(resources.BUFFER);
    //             // longfi_radio.receive();
    //         }
    //         ClientEvent::ClientEvent_None => {
                
    //             let binary = application(
    //                         // message,
    //                         // resources.COUNTER_1,
    //                         // resources.COUNTER_2,
    //                         // resources.STATE,
    //                 resources.LED
    //             );
    //         }
    //     }
    // }


    #[task(priority = 2, resources = [LED])]
    fn position_button_event(cx: position_button_event::Context, turnon: bool, blink: bool, number: u32){
        led(cx.resources.LED, turnon, blink, number);
    }
    // #[task(priority = 4, resources = [LED])]
    // fn distance_button_event(){
    //     ledOff(&mut resources.LED);
    // }
    // #[interrupt(priority = 1, resources = [SX1276_DIO0, INT], spawn = [radio_event])]
    // fn EXTI4_15() {
        // resources.INT.clear_irq(resources.SX1276_DIO0.pin_number());
        // spawn.radio_event(RfEvent::DIO0).unwrap();
        // }
    #[task(binds = EXTI4_15, priority = 1, resources = [DistanceButton, INT], spawn = [position_button_event])]
    fn EXTI4_15(c: EXTI4_15::Context) {
        hprintln!("Hello!_test12").unwrap();
        c.resources.INT.clear_irq(c.resources.DistanceButton.pin_number());
        c.spawn.position_button_event(true,false,0).unwrap();
    }
    #[task(binds = EXTI2_3, priority = 1, resources = [PositionButton, INT], spawn = [position_button_event])]
    fn pos(c: pos::Context) {
        hprintln!("Hello!_test").unwrap();
        c.resources.INT.clear_irq(c.resources.PositionButton.pin_number());
        c.spawn.position_button_event(false,false,0).unwrap();
    }
// 
    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
        fn LPTIM1();
    }
};

fn ledOn(led: &mut gpiob::PB2<Output<PushPull>>) {
    led.set_high().ok();
}

fn ledOff(led: &mut gpiob::PB2<Output<PushPull>>) {
    led.set_low().ok();
}

fn led(led: &mut gpiob::PB2<Output<PushPull>>, turnon: bool, blink: bool, number: u32) {
    if blink==true {
        for i in 0..number {
            led.set_high().ok();
            //hal::delay::Delay.DelayMs(100);
            led.set_low().ok();
        }
    } else {
        if turnon==true {
            led.set_high().ok();
        } else {
            led.set_low().ok();
        }
    }
}
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