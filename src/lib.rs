use std::ops;

pub trait VecN {
    type Type: VecN;

    fn dimensions(&self) -> isize;

}

pub trait PointN {
    type Type: PointN;

    fn dimensions(&self) -> isize;
}
