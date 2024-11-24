use std::ops::{BitAnd, BitXor, Not};

#[derive(Debug)]
pub struct BitField<T> {
    last_state: Option<T>,
    state: T,
}

impl<T> BitField<T>
where
    T: Not<Output = T> + BitAnd<Output = T> + BitXor<Output = T> + PartialEq + Default + Copy,
{
    pub fn new(state: T) -> Self {
        Self {
            last_state: None,
            state,
        }
    }

    pub fn update(&mut self, new_state: T) {
        self.last_state = Some(self.state);
        self.state = new_state;
    }

    pub fn is_set(&self, switch: impl Into<T>) -> bool {
        self.state & switch.into() != Default::default()
    }

    pub fn has_changed(&self, switch: impl Into<T>) -> bool {
        match self.last_state {
            Some(last_state) => (self.state ^ last_state) & switch.into() != Default::default(),
            None => true,
        }
    }

    pub fn when_changed(&self, switch: impl Into<T>, func: impl FnOnce(bool)) {
        let switch_bit: T = switch.into();
        if self.has_changed(switch_bit) {
            func(self.is_set(switch_bit))
        }
    }
}
