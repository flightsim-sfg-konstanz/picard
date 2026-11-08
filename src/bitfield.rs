use std::ops::BitAnd;

#[derive(Debug)]
pub struct BitField<T> {
    state: T,
}

impl<T> BitField<T>
where
    T: BitAnd<Output = T> + PartialEq + Default + Copy,
{
    pub fn new(state: T) -> Self {
        Self { state }
    }

    pub fn update(&mut self, new_state: T) {
        self.state = new_state;
    }

    pub fn is_set(&self, switch: impl Into<T>) -> bool {
        self.state & switch.into() != Default::default()
    }
}
