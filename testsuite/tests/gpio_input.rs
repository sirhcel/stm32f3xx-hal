#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, Output, PXx, PushPull};

struct State {
    input_ground: PXx<Input>,
    input_vdd: PXx<Input>,
}

#[defmt_test::tests]
mod tests {
    use defmt::{assert, unwrap};
    use stm32f3xx_hal::{pac, prelude::*};

    // Test the defaults with no configuration
    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
        // TODO: Get pa9 and pa10 because these also implement spi and uart
        let input_ground = gpiob
            .pb8
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr)
            .downgrade()
            .downgrade();
        let input_vdd = gpiob
            .pb9
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr)
            .downgrade()
            .downgrade();

        super::State {
            input_ground,
            input_vdd,
        }
    }

    #[test]
    fn ground_is_low(state: &mut super::State) {
        assert!(unwrap!(state.input_ground.is_low()));
    }

    #[test]
    fn vdd_is_high(state: &mut super::State) {
        assert!(unwrap!(state.input_vdd.is_high()));
    }
}
