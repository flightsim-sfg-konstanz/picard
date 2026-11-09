use std::ops::BitAnd;

#[derive(Debug)]
pub struct BitField<T> {
    state: T,
    previous_state: T,
}

impl<T> BitField<T>
where
    T: BitAnd<Output = T> + PartialEq + Default + Copy,
{
    pub fn new(state: T) -> Self {
        Self {
            state,
            previous_state: state,
        }
    }

    pub fn update(&mut self, new_state: T) {
        self.previous_state = self.state;
        self.state = new_state;
    }

    pub fn is_set(&self, switch: impl Into<T>) -> bool {
        self.state & switch.into() != Default::default()
    }

    /// Check whether the switch moved with the last update.
    pub fn has_changed(&self, switch: impl Into<T>) -> bool {
        let switch = switch.into();
        self.state & switch != self.previous_state & switch
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changes_are_tracked_per_bit() {
        let mut bits = BitField::new(0b01u8);
        assert!(!bits.has_changed(0b01u8));
        bits.update(0b10);
        assert!(bits.has_changed(0b01u8));
        assert!(bits.has_changed(0b10u8));
        assert!(!bits.has_changed(0b100u8));
        bits.update(0b10);
        assert!(!bits.has_changed(0b11u8));
    }
}
