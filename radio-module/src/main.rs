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
    stm32::TIM2,
    delay::Delay,
    time::U32Ext,
    timer,
    timer::Timer,
};
use cortex_m_semihosting::{hprintln};
use cortex_m::peripheral::DWT;
use nb::block;
use rtfm::Mutex;
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
        COUNTER: u8,
        TX: Tx<USART1>,
        RX: Rx<USART1>,
        TIMER_LED_INTERVAL: timer::Timer<pac::TIM2>,
     //   ITM: ITM,
        //USART1: *const usart1::RegisterBlock,
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

        let mut tim2 = timer::Timer::tim2(cx.device.TIM2, 1000.ms(), &mut rcc);
        let mut counter = 0;

        let mut rx = gpioa.pa10.into_pull_up_input();
        let mut tx = gpioa.pa9.into_push_pull_output();
        // Configure the external interrupt on the falling edge for the pin 2.

        let serial = Serial::usart1(
            cx.device.USART1,
            (tx, rx),
            serialConfig::default().baudrate(9_600.bps()), //    Is now 9600 (default, 8 bit data 1 stop)
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


        // Configure PB5 as output.
        let mut led = gpiob.pb2.into_push_pull_output();
        tim2.listen();
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
            TIMER_LED_INTERVAL: tim2,
            //COUNTER: counter,
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

    #[task(priority = 3, resources = [LED, COUNTER, TIMER_LED_INTERVAL])]
    fn led_button_event(cx: led_button_event::Context) {
        //let temp: stm32l0xx_hal::gpio::gpiob::PB2<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>> = *cx.resources.LED;
        //led(cx.resources.LED, ledon);
        let mut led = cx.resources.LED;
        let mut counter = cx.resources.COUNTER;
        let mut timer_led_interval = cx.resources.TIMER_LED_INTERVAL;
        ledOff(led);
        counterReset(counter);
        timer_led_interval.reset();
        timerListen(timer_led_interval);
        //ledBlink(cx.resources.LED, ledon, cx.resources.COUNTER);
    }

    #[task(binds = TIM2, priority = 2, resources = [LED, COUNTER, TIMER_LED_INTERVAL])]
    fn led_timer_event(cx: led_timer_event::Context) {
        let mut led = cx.resources.LED;
        let mut counter = cx.resources.COUNTER;
        let mut timer_led_interval = cx.resources.TIMER_LED_INTERVAL;
        let mut check = false;
        counter.lock(|counter| {
            counterPlusOne(counter);
            if (counterCheck(counter, 6)) {
                check = true;
            }
            //let counter = *cx.resources.COUNTER;
            
        });
        led.lock(|led| {
            ledBlink(led);
            
        });
        timer_led_interval.lock(|timer_led_interval| {
            timer_led_interval.clear_irq();
            timer_led_interval.reset();
            if (check) {
                timerUnlisten(timer_led_interval);
            }
        });

    }
    // #[task(priority = 2, resources = [&LED])]
    // fn position_button_event(cx: position_button_event::Context){
        //let temp: stm32l0xx_hal::gpio::gpiob::PB2<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>> = *cx.resources.LED;
        // ledOn(cx.resources.LED);
    // }
    // #[task(priority = 4, resources = [&LED])]
    // fn distance_button_event(cx: distance_button_event::Context){
       // let temp: stm32l0xx_hal::gpio::gpiob::PB2<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>> = *cx.resources.LED;
        // ledOff(cx.resources.LED);
    // }
    // #[interrupt(priority = 1, resources = [SX1276_DIO0, INT], spawn = [radio_event])]
    // fn EXTI4_15() {
        // resources.INT.clear_irq(resources.SX1276_DIO0.pin_number());
        // spawn.radio_event(RfEvent::DIO0).unwrap();
        // }
    #[task(binds = EXTI4_15, priority = 1, resources = [DistanceButton, PositionButton, INT], spawn = [led_button_event])]
    fn EXTI4_15(c: EXTI4_15::Context) {
        
       // let mut distancebutton: gpiob::PB5<Input<PullUp>> = c.resources.DistanceButton;
        let res = c.resources.DistanceButton.is_high();
        match res {
            core::result::Result::Ok(v) => {
                if v==true {
                    hprintln!("Hello!_testif").unwrap();
                    c.resources.INT.clear_irq(c.resources.DistanceButton.pin_number());
                    c.spawn.led_button_event().unwrap();
                }
            }
            _ => {}
        }
        let res = c.resources.PositionButton.is_high();
        match res {
            core::result::Result::Ok(v) => {
                if v==true {
                    hprintln!("Hello!_testelse").unwrap();
                    c.resources.INT.clear_irq(c.resources.PositionButton.pin_number());
                    c.spawn.led_button_event().unwrap();
                }
            }
            _ => {}
        }
    }

     #[idle(resources = [RX, TX])]
    fn idle(cx: idle::Context) -> ! {
        let rx = cx.resources.RX;
        let tx = cx.resources.TX;
        //sned sentence $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28<CR><LR>
        tx.write(36u8);
        tx.write(80u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(77u8);
        tx.write(84u8);
        tx.write(75u8);
        tx.write(51u8);
        tx.write(49u8);
        tx.write(52u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(49u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(49u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(48u8);
        tx.write(42u8);
        tx.write(0x28);
        tx.write(13u8);
        tx.write(10u8);

        // send sentence $PMTK220,1000*1F<CR><LR>

        tx.write(36u8); 
        tx.write(80u8);
        tx.write(77u8);
        tx.write(84u8);
        tx.write(75u8);
        tx.write(50u8);
        tx.write(50u8);
        tx.write(48u8);
        tx.write(44u8);
        tx.write(49u8);
        tx.write(48u8);
        tx.write(48u8);
        tx.write(48u8);
        tx.write(42u8);
        tx.write(0x1F);
        tx.write(13u8);
        tx.write(10u8);



        loop {
            //hprintln!("test");
            match block!(rx.read()) {
                Ok(byte) => {
                    hprintln!("{:?}", byte);
                    //tx.write(byte).unwrap();
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

fn ledOn(led: &mut gpiob::PB2<Output<PushPull>>) {
    led.set_high().ok();
}

fn ledOff(led: &mut gpiob::PB2<Output<PushPull>>) {
    led.set_low().ok();
}

fn led(led: &mut gpiob::PB2<Output<PushPull>>, turnon: bool,) {
   
    if turnon==true {
        led.set_high().ok();
    } else {
        led.set_low().ok();
    }

}

fn ledBlink(led: &mut gpiob::PB2<Output<PushPull>>){
    let res = led.is_low();
    match res {
        core::result::Result::Ok(v) => {
            if v==true {
                led.set_high().ok();
                
            } else {
                led.set_low().ok();
            }
        }
        _ => {}
    }
}

fn counterPlusOne(counter: &mut u8) {
    *counter += 1;
}

fn counterReset(counter: &mut u8) {
    *counter = 0;
}

fn counterCheck(counter: &mut u8, check: u8) -> bool {
    if (*counter >= check) {
        return true;
    } else {
        return false;
    }
}

fn timerListen(timer: &mut timer::Timer<pac::TIM2>) {
    timer.listen();
}

fn timerUnlisten(timer: &mut timer::Timer<pac::TIM2>) {
    timer.unlisten();
}
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