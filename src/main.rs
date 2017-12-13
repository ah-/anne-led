#![cfg_attr(feature="clippy", feature(plugin))]
#![cfg_attr(feature="clippy", plugin(clippy))]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_semihosting;
extern crate cortex_m_rtfm as rtfm;
extern crate stm32l151;

use core::fmt::Write;

use rtfm::{app, Threshold};
use cortex_m::peripheral::SystClkSource;
use cortex_m_semihosting::hio;

use stm32l151::{GPIOA, GPIOB, GPIOC};


app! {
    device: stm32l151,

    resources: {
        static STDOUT: hio::HStdout;
        static ON: bool = true;
        static RGB: u8 = 1;
        static ROW: u8 = 0;
        static COL: u8 = 0;
        static COUNTER: u8 = 0;
        static RECV_LEN: u16 = 0;
        static RECV_BUFFER: [u8; 0x100] = [0; 0x100];
    },

    tasks: {
        SYS_TICK: {
            path: tick,
            resources: [ON, RGB, ROW, COL, COUNTER],
        },
        USART3: {
            path: usart_recv,
            resources: [DMA1, USART3, STDOUT, RECV_LEN],
        },
        DMA1_CHANNEL3: {
            path: dma_recv,
            resources: [DMA1, STDOUT, RECV_LEN, RECV_BUFFER, ON, RGB, ROW, COL],
        }
    }

}

fn init(p: init::Peripherals, mut r: init::Resources) -> init::LateResourceValues {
    init_clock(&p);
    init_led_gpios(&p);
    init_uart(&p, &mut r);

    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(10_000);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();

    p.GPIOH.bsrr.write(|w| w.bs0().bit(true));

    init::LateResourceValues {
        STDOUT: hio::hstdout().unwrap(),
    }
}

fn idle() -> !{
    loop {
        rtfm::wfi();
    }
}

fn tick(_t: &mut Threshold, r: SYS_TICK::Resources) {
    **r.COUNTER += 1;

    if (**r.COUNTER & 0xf) <= 2 && **r.ON {
        set_row_color(**r.ROW, (**r.RGB & 1) != 0,
                               (**r.RGB & 2) != 0,
                               (**r.RGB & 4) != 0);
        enable_led_column(**r.COL, true);
    } else {
        for row in 0..5 {
            set_row_color(row, false, false, false);
        }
        for col in 0..14 {
            enable_led_column(col, false);
        }
    }

}

fn usart_recv(_t: &mut Threshold, r: USART3::Resources) {
    if r.USART3.sr.read().rxne().bit_is_set() {
        let bits = r.USART3.dr.read().bits() as u16;

        if bits != 9 && bits != 0 {
            **r.RECV_LEN = bits;
            r.DMA1.cndtr3.modify(|_, w| {
                unsafe { w.ndt().bits(bits) }
            });
            r.DMA1.ccr3.modify(|_, w| {
                w.en().set_bit()
            });
        }
        //write!(r.STDOUT, "x").unwrap()
    }
}

fn dma_recv(_t: &mut Threshold, r: DMA1_CHANNEL3::Resources) {
    r.DMA1.ifcr.write(|w| {
        w.chtif3().set_bit()
         .ctcif3().set_bit()
         .cgif3().set_bit()
    });

    r.DMA1.ccr3.modify(|_, w| {
        w.en().clear_bit()
    });


    let recv_len = **r.RECV_LEN;
    /*
    if recv_len != 1 && recv_len != 2 && recv_len != 5 && recv_len != 0xb {
        for x in 0..(**r.RECV_LEN) {
            write!(r.STDOUT, " {:x}", r.RECV_BUFFER[x as usize]).unwrap()
        }
    }
    */

    let data = &(**r.RECV_BUFFER)[0..**r.RECV_LEN as usize];
    if data.len() > 0 {
        match data[0] {
            1 => {
                //**r.ON = false;
                **r.ROW = (**r.ROW + 1) % 5;
            },
            2 => {
                **r.ON = true;
            },
            5 => {
                if (data.len() >= 5) {
                if data[2] != 0 {
                    **r.RGB = 1;
                }
                if data[3] != 0 {
                    **r.ROW = (**r.ROW + 1) % 5;
                    **r.RGB = 2;
                }
                if data[4] != 0 {
                    **r.RGB = 4;
                }
                }
                //write!(r.STDOUT, "{}", **r.RGB).unwrap()
            },
            11 => {
                    **r.COL = (**r.COL + 1) % 14;
            },
            _ => write!(r.STDOUT, "pkg {}", data[0]).unwrap(),
        }
    }
}

fn init_clock(p: &init::Peripherals) {
    p.FLASH.acr.modify(|_, w| { w.acc64().set_bit() });
    p.FLASH.acr.modify(|_, w| { w.prften().set_bit() });
    p.FLASH.acr.modify(|_, w| { w.latency().set_bit() });

    p.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while p.RCC.cr.read().hsirdy().bit_is_clear() {}

    p.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
    p.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());

    p.PWR.cr.modify(|_, w| {
        w.lprun().clear_bit();
        unsafe { w.vos().bits(0b01) }
    });
    while p.PWR.csr.read().vosf().bit_is_set() {}

    p.RCC.cfgr.modify(|_, w| unsafe {
        w.ppre1().bits(0b100) 
         .ppre2().bits(0b100)
         .pllmul().bits(0b0010)
         .plldiv().bits(0b10)
    });

    p.RCC.cr.modify(|_, w| w.msion().set_bit());
    while p.RCC.cr.read().msirdy().bit_is_clear() {}

    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    p.RCC.cfgr.modify(|_, w| unsafe { w.sw().bits(0b11) });
    while p.RCC.cfgr.read().sws().bits() != 0b11 {};
}

fn init_uart(p: &init::Peripherals, r: &mut init::Resources) {
    p.GPIOB.moder.modify(|_, w| unsafe {
        w.moder11().bits(0b10)
    });
    p.GPIOB.pupdr.modify(|_, w| unsafe {
        w.pupdr11().bits(0b01)
    });
    p.GPIOB.afrh.modify(|_, w| unsafe {
        w.afrh11().bits(7)
    });

    p.RCC.apb1enr.modify(|_, w| w.usart3en().set_bit());
    p.RCC.ahbenr.modify(|_, w| w.dma1en().set_bit());

    p.USART3.brr.modify(|_, w| unsafe { w.bits(417) });
    p.USART3.cr3.modify(|_, w| w.dmar().set_bit());
    p.USART3.cr1.modify(|_, w| {
        w.rxneie().set_bit()
         .tcie().set_bit()
         .idleie().set_bit()
         .re().set_bit()
         .te().set_bit()
         .ue().set_bit()
    });

    p.DMA1.cpar3.write(|w| {
        unsafe {
            w.pa().bits(0x4000_4804)
        }
    });
    p.DMA1.cmar3.write(|w| {
        unsafe {
            w.ma().bits(r.RECV_BUFFER.as_mut_ptr() as u32) 
        }
    });
    p.DMA1.ccr3.modify(|_, w| {
        unsafe { w.pl().bits(2); }
        w.minc().set_bit()
         .tcie().set_bit()
         .en().clear_bit()
    });
}

fn init_led_gpios(p: &init::Peripherals) {
    p.RCC.ahbenr.modify(|_, w| {
        w.gpiopaen().set_bit()
         .gpiopben().set_bit()
         .gpiopcen().set_bit()
         .gpiophen().set_bit()
    });

    p.GPIOA.moder.modify(|_, w| unsafe {
        w.moder0().bits(1)
         .moder1().bits(1)
         .moder2().bits(1)
         .moder3().bits(1)
         .moder4().bits(1)
         .moder5().bits(1)
         .moder6().bits(1)
         .moder7().bits(1)
         .moder8().bits(1)
         .moder9().bits(1)
         .moder10().bits(1)
         .moder11().bits(1)
         .moder12().bits(1)
         .moder15().bits(1)
    });

    p.GPIOB.moder.modify(|_, w| unsafe {
        w.moder0().bits(1)
         .moder1().bits(1)
         .moder3().bits(1)
         .moder4().bits(1)
         .moder5().bits(1)
         .moder6().bits(1)
         .moder7().bits(1)
         .moder8().bits(1)
         .moder9().bits(1)
         //.moder10().bits(1)
         .moder12().bits(1)
         .moder13().bits(1)
         .moder14().bits(1)
         .moder15().bits(1)
    });

    p.GPIOC.moder.modify(|_, w| unsafe {
        w.moder14().bits(1)
         .moder15().bits(1)
    });

    p.GPIOH.moder.modify(|_, w| unsafe {
        w.moder0().bits(1)
    });
}

fn enable_led_column(column: u8, on: bool) {
    cortex_m::interrupt::free(
        |cs| {
            let gpioa = GPIOA.borrow(cs);
            let gpiob = GPIOB.borrow(cs);
            let gpioc = GPIOC.borrow(cs);

            match column {
                0 => gpioc.odr.modify(|_, w| w.odr15().bit(on)),
                1 => gpioc.odr.modify(|_, w| w.odr14().bit(on)),
                2 => gpiob.odr.modify(|_, w| w.odr3().bit(on)),
                3 => gpioa.odr.modify(|_, w| w.odr15().bit(on)),
                4 => gpioa.odr.modify(|_, w| w.odr12().bit(on)),
                5 => gpioa.odr.modify(|_, w| w.odr11().bit(on)),
                6 => gpioa.odr.modify(|_, w| w.odr10().bit(on)),
                7 => gpioa.odr.modify(|_, w| w.odr9().bit(on)),
                8 => gpioa.odr.modify(|_, w| w.odr8().bit(on)),
                9 => gpiob.odr.modify(|_, w| w.odr15().bit(on)),
                10 => gpioa.odr.modify(|_, w| w.odr7().bit(on)),
                11 => gpioa.odr.modify(|_, w| w.odr6().bit(on)),
                12 => gpioa.odr.modify(|_, w| w.odr5().bit(on)),
                13 => gpioa.odr.modify(|_, w| w.odr4().bit(on)),
                _ => panic!("Invalid led column")
            }
        }
    )
}

fn set_row_color(row: u8, red: bool, green: bool, blue: bool) {
    cortex_m::interrupt::free(
        |cs| {
            let gpioa = GPIOA.borrow(cs);
            let gpiob = GPIOB.borrow(cs);

            match row {
                0 => {
                    gpiob.odr.modify(|_, w| w.odr0().bit(red));
                    gpioa.odr.modify(|_, w| w.odr0().bit(green));
                    gpiob.odr.modify(|_, w| w.odr14().bit(blue));
                },
                1 => {
                    gpiob.odr.modify(|_, w| w.odr1().bit(red));
                    gpioa.odr.modify(|_, w| w.odr1().bit(green));
                    gpiob.odr.modify(|_, w| w.odr4().bit(blue));
                },
                2 => {
                    gpiob.odr.modify(|_, w| w.odr12().bit(red));
                    gpioa.odr.modify(|_, w| w.odr2().bit(green));
                    gpiob.odr.modify(|_, w| w.odr5().bit(blue));
                },
                3 => {
                    gpiob.odr.modify(|_, w| w.odr13().bit(red));
                    gpioa.odr.modify(|_, w| w.odr3().bit(green));
                    gpiob.odr.modify(|_, w| w.odr6().bit(blue));
                },
                4 => {
                    gpiob.odr.modify(|_, w| w.odr8().bit(red));
                    gpiob.odr.modify(|_, w| w.odr7().bit(green));
                    gpiob.odr.modify(|_, w| w.odr9().bit(blue));
                },
                _ => panic!("Invalid led row")
            }
        }
    )
}
