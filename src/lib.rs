#![no_std]

use num_traits::{
    clamp, CheckedAdd, CheckedDiv, CheckedMul, CheckedNeg, CheckedSub, Num, One, Zero,
};

pub struct PIDController<T> {
    k_p: T,
    k_i: T,
    k_d: T,
    windup_limit: T,
    divisor: T,

    accumulated: T,
    prev_error: T,
}

impl<T> PIDController<T>
where
    T: Num + Ord + CheckedAdd + CheckedSub + CheckedMul + CheckedDiv + CheckedNeg + Copy,
{
    pub fn new(k_p: T, k_i: T, k_d: T, windup_limit: T, divisor: T) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            windup_limit,
            divisor,
            accumulated: Zero::zero(),
            prev_error: Zero::zero(),
        }
    }

    pub fn set_terms(&mut self, k_p: T, k_i: T, k_d: T, windup_limit: T, divisor: T) {
        self.k_p = k_p;
        self.k_i = k_i;
        self.k_d = k_d;
        self.windup_limit = windup_limit;
        self.divisor = divisor;
    }

    pub fn run<U>(&mut self, target: T, measured: T) -> Option<U>
    where
        U: TryFrom<T>,
    {
        let error = target.checked_sub(&measured)?;

        // update accumulated error for I term
        self.accumulated = clamp(
            self.accumulated.checked_add(&error)?,
            self.windup_limit.checked_neg()?,
            self.windup_limit,
        );

        let p = error.checked_mul(&self.k_p)?;

        let i = &self.accumulated.checked_mul(&self.k_i)?;

        let d = &error
            .checked_sub(&self.prev_error)?
            .checked_mul(&self.k_d)?;

        let output = p.checked_add(i)?.checked_add(d)?;

        self.prev_error = error;

        U::try_from(output.checked_div(&self.divisor)?).ok()
    }
}

impl<T> Default for PIDController<T>
where
    T: Zero + One,
{
    fn default() -> Self {
        Self {
            k_p: Zero::zero(),
            k_i: Zero::zero(),
            k_d: Zero::zero(),
            windup_limit: Zero::zero(),
            divisor: One::one(),
            accumulated: Zero::zero(),
            prev_error: Zero::zero(),
        }
    }
}
