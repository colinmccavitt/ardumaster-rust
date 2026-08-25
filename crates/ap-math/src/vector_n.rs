//! Fixed-size vectors and matrices, ported from `AP_Math/vectorN.h` and
//! `AP_Math/matrixN.{h,cpp}`.
//!
//! Used by the soaring extended Kalman filter (`AP_Soaring`, which drives
//! ArduPlane's thermal mode), by accelerometer calibration, and by NavEKF3.
//!
//! Upstream parameterises the length as `uint8_t N`; here it is a const
//! generic, so the length is part of the type in the same way and mismatched
//! operands do not compile.

use core::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

use crate::scalar::Real;

/// A vector of `N` elements. Upstream `VectorN<T, N>`.
///
/// # Equality is exact
///
/// Upstream compares with `!=` per element rather than `is_equal`, unlike
/// `Vector2` and `Vector3`. Reproduced: these hold filter state, where an
/// epsilon comparison would be the surprising choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VectorN<T, const N: usize> {
    v: [T; N],
}

impl<T: Real, const N: usize> Default for VectorN<T, N> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<T: Real, const N: usize> VectorN<T, N> {
    /// A zero vector. Upstream's default constructor.
    #[must_use]
    pub fn zero() -> Self {
        Self { v: [T::zero(); N] }
    }

    /// From an array of elements.
    ///
    /// Upstream takes a bare `const T *` and `memcpy`s `N` elements from it,
    /// trusting the caller to have supplied that many. An array carries its
    /// length in the type.
    #[must_use]
    pub const fn new(v: [T; N]) -> Self {
        Self { v }
    }

    /// The elements, as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[T] {
        &self.v
    }

    /// Set every element to zero, upstream `zero()`.
    pub fn set_zero(&mut self) {
        self.v = [T::zero(); N];
    }

    /// Dot product with another vector, upstream `operator*(const VectorN&)`.
    ///
    /// # DIVERGENCE D-013
    ///
    /// Upstream accumulates into a `float` regardless of `T`:
    ///
    /// ```cpp
    /// T operator *(const VectorN<T,N> &v) const {
    ///     float ret = 0;
    ///     for (uint8_t i=0; i<N; i++) {
    ///         ret += _v[i] * v._v[i];
    ///     }
    ///     return ret;
    /// }
    /// ```
    ///
    /// `VectorN<ftype, N>` is `VectorN<double, N>` wherever
    /// `HAL_WITH_EKF_DOUBLE` is set, so on those builds each product is
    /// computed in double, rounded to float to accumulate, and the sum widened
    /// back to double on return — discarding most of the precision the double
    /// build exists to provide. This port accumulates in `T`. See
    /// DIVERGENCES.md.
    #[must_use]
    pub fn dot(&self, other: &Self) -> T {
        let mut acc = T::zero();
        for (a, b) in self.v.iter().zip(other.v.iter()) {
            acc = acc + *a * *b;
        }
        acc
    }

    /// `self = a × b`, upstream `VectorN::mult`.
    pub fn set_mult(&mut self, a: &MatrixN<T, N>, b: &Self) {
        for (out, row) in self.v.iter_mut().zip(a.v.iter()) {
            let mut acc = T::zero();
            for (av, bv) in row.iter().zip(b.v.iter()) {
                acc = acc + *av * *bv;
            }
            *out = acc;
        }
    }
}

impl<T, const N: usize> Index<usize> for VectorN<T, N> {
    type Output = T;
    #[allow(
        clippy::indexing_slicing,
        reason = "panicking on an out-of-range subscript is the documented \
contract of Index; there is nothing else this impl could do"
    )]
    fn index(&self, i: usize) -> &T {
        &self.v[i]
    }
}

impl<T, const N: usize> IndexMut<usize> for VectorN<T, N> {
    #[allow(
        clippy::indexing_slicing,
        reason = "the contract of IndexMut, as above"
    )]
    fn index_mut(&mut self, i: usize) -> &mut T {
        &mut self.v[i]
    }
}

impl<T: Real, const N: usize> Neg for VectorN<T, N> {
    type Output = Self;
    fn neg(mut self) -> Self {
        for e in self.v.iter_mut() {
            *e = -*e;
        }
        self
    }
}

impl<T: Real, const N: usize> Add for VectorN<T, N> {
    type Output = Self;
    fn add(mut self, rhs: Self) -> Self {
        for (a, b) in self.v.iter_mut().zip(rhs.v.iter()) {
            *a = *a + *b;
        }
        self
    }
}

impl<T: Real, const N: usize> Sub for VectorN<T, N> {
    type Output = Self;
    fn sub(mut self, rhs: Self) -> Self {
        for (a, b) in self.v.iter_mut().zip(rhs.v.iter()) {
            *a = *a - *b;
        }
        self
    }
}

impl<T: Real, const N: usize> Mul<T> for VectorN<T, N> {
    type Output = Self;
    fn mul(mut self, rhs: T) -> Self {
        for e in self.v.iter_mut() {
            *e = *e * rhs;
        }
        self
    }
}

impl<T: Real, const N: usize> Div<T> for VectorN<T, N> {
    type Output = Self;
    fn div(mut self, rhs: T) -> Self {
        for e in self.v.iter_mut() {
            *e = *e / rhs;
        }
        self
    }
}

impl<T: Real, const N: usize> AddAssign for VectorN<T, N> {
    fn add_assign(&mut self, rhs: Self) {
        for (a, b) in self.v.iter_mut().zip(rhs.v.iter()) {
            *a = *a + *b;
        }
    }
}

impl<T: Real, const N: usize> SubAssign for VectorN<T, N> {
    fn sub_assign(&mut self, rhs: Self) {
        for (a, b) in self.v.iter_mut().zip(rhs.v.iter()) {
            *a = *a - *b;
        }
    }
}

impl<T: Real, const N: usize> MulAssign<T> for VectorN<T, N> {
    fn mul_assign(&mut self, rhs: T) {
        for e in self.v.iter_mut() {
            *e = *e * rhs;
        }
    }
}

impl<T: Real, const N: usize> DivAssign<T> for VectorN<T, N> {
    fn div_assign(&mut self, rhs: T) {
        for e in self.v.iter_mut() {
            *e = *e / rhs;
        }
    }
}

/// An `N × N` matrix. Upstream `MatrixN<T, N>`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MatrixN<T, const N: usize> {
    v: [[T; N]; N],
}

impl<T: Real, const N: usize> Default for MatrixN<T, N> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<T: Real, const N: usize> MatrixN<T, N> {
    /// A zero matrix. Upstream's default constructor.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            v: [[T::zero(); N]; N],
        }
    }

    /// A diagonal matrix, upstream's `MatrixN(const float d[N])`.
    ///
    /// Upstream's parameter is `const float*` regardless of `T`, so building a
    /// `MatrixN<double, N>` means narrowing the diagonal through `float`
    /// first. This takes `T`, which is what the caller already has; passing
    /// floats to a double matrix still works by converting at the call site,
    /// where it is visible.
    #[must_use]
    pub fn from_diagonal(d: &[T; N]) -> Self {
        let mut m = Self::zero();
        for (i, (row, val)) in m.v.iter_mut().zip(d.iter()).enumerate() {
            if let Some(cell) = row.get_mut(i) {
                *cell = *val;
            }
        }
        m
    }

    /// `self = a ⊗ b`, the outer product. Upstream `MatrixN::mult`.
    pub fn set_mult(&mut self, a: &VectorN<T, N>, b: &VectorN<T, N>) {
        for (row, av) in self.v.iter_mut().zip(a.v.iter()) {
            for (cell, bv) in row.iter_mut().zip(b.v.iter()) {
                *cell = *av * *bv;
            }
        }
    }

    /// Average each off-diagonal pair with its transpose, upstream
    /// `force_symmetry`.
    ///
    /// # DIVERGENCE D-014
    ///
    /// Upstream's inner bound is one short:
    ///
    /// ```cpp
    /// for (uint8_t i = 0; i < N; i++) {
    ///     for (uint8_t j = 0; j < (i - 1); j++) {
    /// ```
    ///
    /// so every pair `(i, i-1)` — the whole sub-diagonal — is skipped and the
    /// matrix is left asymmetric. At the `N = 4` used by the soaring EKF that
    /// is three of the six off-diagonal pairs, so half the matrix is not
    /// symmetrised by a routine whose only job is to symmetrise it.
    ///
    /// (`i - 1` at `i == 0` promotes to `int` and gives `-1`, so the loop is
    /// merely skipped rather than running away — the bound is wrong, not
    /// unsafe.)
    ///
    /// This port uses `j < i`. See DIVERGENCES.md.
    pub fn force_symmetry(&mut self) {
        for i in 0..N {
            for j in 0..i {
                let mean = (self.at(i, j) + self.at(j, i)) / T::from_f64(2.0);
                self.set(i, j, mean);
                self.set(j, i, mean);
            }
        }
    }

    /// Element read. `force_symmetry` and `is_symmetric` need `[i][j]` and
    /// `[j][i]` in the same expression, which no iterator pairing expresses;
    /// these keep the algorithm readable without an unchecked subscript.
    #[inline]
    fn at(&self, i: usize, j: usize) -> T {
        self.v
            .get(i)
            .and_then(|r| r.get(j))
            .copied()
            .unwrap_or_else(T::zero)
    }

    #[inline]
    fn set(&mut self, i: usize, j: usize, v: T) {
        if let Some(cell) = self.v.get_mut(i).and_then(|r| r.get_mut(j)) {
            *cell = v;
        }
    }

    /// Whether the matrix equals its own transpose exactly.
    #[must_use]
    pub fn is_symmetric(&self) -> bool {
        for i in 0..N {
            for j in 0..i {
                if self.at(i, j) != self.at(j, i) {
                    return false;
                }
            }
        }
        true
    }
}

impl<T, const N: usize> Index<usize> for MatrixN<T, N> {
    type Output = [T; N];
    #[allow(clippy::indexing_slicing, reason = "the contract of Index, as above")]
    fn index(&self, i: usize) -> &[T; N] {
        &self.v[i]
    }
}

impl<T, const N: usize> IndexMut<usize> for MatrixN<T, N> {
    #[allow(
        clippy::indexing_slicing,
        reason = "the contract of IndexMut, as above"
    )]
    fn index_mut(&mut self, i: usize) -> &mut [T; N] {
        &mut self.v[i]
    }
}

impl<T: Real, const N: usize> AddAssign for MatrixN<T, N> {
    fn add_assign(&mut self, rhs: Self) {
        for (row, rrow) in self.v.iter_mut().zip(rhs.v.iter()) {
            for (cell, r) in row.iter_mut().zip(rrow.iter()) {
                *cell = *cell + *r;
            }
        }
    }
}

impl<T: Real, const N: usize> SubAssign for MatrixN<T, N> {
    fn sub_assign(&mut self, rhs: Self) {
        for (row, rrow) in self.v.iter_mut().zip(rhs.v.iter()) {
            for (cell, r) in row.iter_mut().zip(rrow.iter()) {
                *cell = *cell - *r;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exactly representable values throughout")]
    #![allow(
        clippy::indexing_slicing,
        reason = "indexes fixed-size arrays declared in the test itself; an index fault here is a test failure, which is the desired outcome"
    )]

    use super::*;

    /// D-014: after `force_symmetry` the matrix must actually be symmetric.
    /// Upstream's loop leaves the sub-diagonal untouched, so this fails against
    /// its bound and passes against the corrected one.
    #[test]
    fn d014_force_symmetry_symmetrises_every_pair() {
        for_each_n::<4>();
        for_each_n::<5>();
        for_each_n::<2>();

        fn for_each_n<const N: usize>() {
            let mut m = MatrixN::<f32, N>::zero();
            // deliberately asymmetric
            for i in 0..N {
                for j in 0..N {
                    m[i][j] = (i * N + j) as f32;
                }
            }
            assert!(!m.is_symmetric() || N < 2, "setup should be asymmetric");
            m.force_symmetry();
            assert!(
                m.is_symmetric(),
                "N={N}: still asymmetric after force_symmetry"
            );
        }
    }

    /// The specific pairs upstream skips. Spelled out so the divergence is
    /// visible as a list rather than only as a property.
    #[test]
    fn d014_sub_diagonal_is_the_part_upstream_skips() {
        let mut m = MatrixN::<f32, 4>::zero();
        for i in 0..4 {
            for j in 0..4 {
                m[i][j] = (i * 4 + j) as f32;
            }
        }
        m.force_symmetry();
        // the three sub-diagonal pairs upstream leaves alone
        for (i, j) in [(1usize, 0usize), (2, 1), (3, 2)] {
            let want = ((i * 4 + j) as f32 + (j * 4 + i) as f32) / 2.0;
            assert_eq!(m[i][j], want, "({i},{j}) not averaged");
            assert_eq!(m[j][i], want, "({j},{i}) not averaged");
        }
    }

    /// D-013: the dot product must keep the precision of `T`. In `f64` the sum
    /// below is exact, but is not representable in `f32` — so an `f32`
    /// accumulator gives a visibly different answer.
    #[test]
    fn d013_dot_product_accumulates_in_t() {
        // 2^30 and 1.0: their sum needs 31 bits of mantissa, which f32 (24)
        // cannot hold but f64 (53) can
        let a = VectorN::<f64, 2>::new([1073741824.0, 1.0]);
        let b = VectorN::<f64, 2>::new([1.0, 1.0]);
        let got = a.dot(&b);
        assert_eq!(
            got, 1073741825.0,
            "an f32 accumulator would give 1073741824"
        );

        // and the f32 instantiation still behaves as f32
        let c = VectorN::<f32, 2>::new([1073741824.0, 1.0]);
        let d = VectorN::<f32, 2>::new([1.0, 1.0]);
        assert_eq!(c.dot(&d), 1073741824.0);
    }

    #[test]
    fn arithmetic_is_elementwise() {
        let a = VectorN::<f32, 3>::new([1.0, 2.0, 3.0]);
        let b = VectorN::<f32, 3>::new([10.0, 20.0, 30.0]);
        assert_eq!((a + b).as_slice(), &[11.0, 22.0, 33.0]);
        assert_eq!((b - a).as_slice(), &[9.0, 18.0, 27.0]);
        assert_eq!((a * 2.0).as_slice(), &[2.0, 4.0, 6.0]);
        assert_eq!((b / 10.0).as_slice(), &[1.0, 2.0, 3.0]);
        assert_eq!((-a).as_slice(), &[-1.0, -2.0, -3.0]);
        assert_eq!(a.dot(&b), 140.0);

        let mut c = a;
        c += b;
        assert_eq!(c.as_slice(), &[11.0, 22.0, 33.0]);
        c -= b;
        assert_eq!(c, a);
        c *= 3.0;
        assert_eq!(c.as_slice(), &[3.0, 6.0, 9.0]);
        c /= 3.0;
        assert_eq!(c, a);
    }

    #[test]
    fn outer_product_and_matrix_vector_product() {
        let a = VectorN::<f32, 3>::new([1.0, 2.0, 3.0]);
        let b = VectorN::<f32, 3>::new([4.0, 5.0, 6.0]);
        let mut m = MatrixN::<f32, 3>::zero();
        m.set_mult(&a, &b);
        assert_eq!(m[0], [4.0, 5.0, 6.0]);
        assert_eq!(m[2], [12.0, 15.0, 18.0]);

        let d = MatrixN::<f32, 3>::from_diagonal(&[2.0, 3.0, 4.0]);
        let mut out = VectorN::<f32, 3>::zero();
        out.set_mult(&d, &a);
        assert_eq!(out.as_slice(), &[2.0, 6.0, 12.0]);
    }

    #[test]
    fn matrix_add_and_sub_assign() {
        let mut m = MatrixN::<f32, 2>::from_diagonal(&[1.0, 2.0]);
        let n = MatrixN::<f32, 2>::from_diagonal(&[10.0, 20.0]);
        m += n;
        assert_eq!(m[0][0], 11.0);
        assert_eq!(m[1][1], 22.0);
        m -= n;
        assert_eq!(m[0][0], 1.0);
        assert_eq!(m[1][1], 2.0);
    }

    /// Upstream compares elementwise with `!=`, not `is_equal`, unlike
    /// Vector2 and Vector3. Pinned so the difference is deliberate.
    #[test]
    fn equality_is_exact_not_epsilon_based() {
        // One ulp apart, at a magnitude where that ulp is smaller than
        // FLT_EPSILON. Both details matter and both caught me out:
        //   * `2.0 + f32::EPSILON` rounds straight back to 2.0, because
        //     epsilon is the gap at 1.0 and the gap at 2.0 is twice that.
        //   * at 2.0 one ulp is 2.4e-7, which EXCEEDS the absolute
        //     FLT_EPSILON `is_equal` compares against, so Vector2 would call
        //     them different too and there would be no contrast to show.
        // At 0.5 the gap is 6.0e-8, distinct in f32 and below FLT_EPSILON.
        let base = 0.5f32;
        let next = f32::from_bits(base.to_bits() + 1);
        assert_ne!(next, base, "must be distinct in f32");
        assert!(
            (next - base).abs() < f32::EPSILON,
            "and closer than the epsilon is_equal uses"
        );

        let a = VectorN::<f32, 2>::new([1.0, base]);
        let b = VectorN::<f32, 2>::new([1.0, next]);
        assert_ne!(a, b, "VectorN equality is exact, unlike Vector2/Vector3");

        // the contrast: Vector2 treats a one-ulp difference as equal
        use crate::vector2::Vector2f;
        assert_eq!(
            Vector2f::new(1.0, base),
            Vector2f::new(1.0, next),
            "Vector2 is epsilon-based, which is the difference being pinned"
        );
    }
}
