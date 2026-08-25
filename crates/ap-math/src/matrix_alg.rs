//! Square matrix inversion and multiplication, ported from
//! `AP_Math/matrix_alg.cpp`.
//!
//! Used by accelerometer and compass calibration to invert the normal-equation
//! matrix `JᵀJ` — dimension 3, 4 and 9 in practice.
//!
//! Matrices are row-major `n × n` in a flat slice, as upstream.
//!
//! # DIVERGENCE D-012: no heap
//!
//! Upstream's general path allocates five `n × n` scratch matrices per call
//! with `NEW_NOTHROW` — `new(std::nothrow)`, which returns null rather than
//! throwing — and **never checks any of them**. `mat_inverseN` allocates `L`,
//! `U` and `P` and immediately calls `mat_LU_decompose`, whose first act is
//! `memset(L, ...)`. On a controller that has run out of memory mid-calibration
//! that is a null dereference, not a failed inversion.
//!
//! ADR-0004 rules out an allocator in the port, so the question does not arise:
//! the caller supplies the scratch and the failure is a compile-time
//! requirement rather than a runtime one. See [`scratch_len`].
//!
//! The 3×3 and 4×4 paths need no scratch at all — they are closed-form — and
//! those are the shapes on the calibration hot path.

use crate::scalar::Real;

pub use crate::matrix_alg_gen::{inverse3x3, inverse4x4};

/// Why an inversion could not be performed.
///
/// Upstream returns a bare `bool` for all of these. Distinguishing them costs
/// nothing and means a caller that passed a too-small buffer is not told its
/// matrix was singular.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatError {
    /// The matrix is singular, or its determinant overflowed. Upstream's
    /// `false`.
    Singular,
    /// A slice did not hold `n * n` elements.
    BadDimensions,
    /// The scratch buffer was shorter than [`scratch_len`].
    ScratchTooSmall {
        /// What was supplied.
        given: usize,
        /// What [`scratch_len`] requires.
        needed: usize,
    },
}

/// Scratch elements [`mat_inverse`] needs for an `n × n` matrix.
///
/// Five `n × n` matrices: the pivot, the two triangular factors, and their two
/// inverses. Upstream heap-allocates the same five.
#[must_use]
pub const fn scratch_len(n: usize) -> usize {
    5 * n * n
}

/// `C = A × B` for row-major `n × n` matrices, upstream `mat_mul`.
///
/// # Errors
///
/// [`MatError::BadDimensions`] if any slice is not `n * n` long.
pub fn mat_mul<T: Real>(a: &[T], b: &[T], c: &mut [T], n: usize) -> Result<(), MatError> {
    if a.len() != n * n || b.len() != n * n || c.len() != n * n {
        return Err(MatError::BadDimensions);
    }
    for i in 0..n {
        for j in 0..n {
            let mut acc = T::zero();
            for k in 0..n {
                acc = acc + at(a, n, i, k) * at(b, n, k, j);
            }
            set(c, n, i, j, acc);
        }
    }
    Ok(())
}

/// Fill `a` with the `n × n` identity, upstream `mat_identity`.
///
/// # Errors
///
/// [`MatError::BadDimensions`] if `a` is not `n * n` long.
pub fn mat_identity<T: Real>(a: &mut [T], n: usize) -> Result<(), MatError> {
    if a.len() != n * n {
        return Err(MatError::BadDimensions);
    }
    for e in a.iter_mut() {
        *e = T::zero();
    }
    for i in 0..n {
        set(a, n, i, i, T::one());
    }
    Ok(())
}

/// Inverse of a square row-major matrix, upstream `mat_inverse`.
///
/// Dispatches to the closed-form 3×3 and 4×4 expansions, and otherwise to LU
/// decomposition with partial pivoting. `scratch` is used only on the LU path
/// and must hold at least [`scratch_len`] elements; the closed-form paths
/// ignore it, so an empty slice is fine when `n` is 3 or 4.
///
/// # Errors
///
/// [`MatError::Singular`] if the matrix cannot be inverted,
/// [`MatError::BadDimensions`] on a length mismatch, or
/// [`MatError::ScratchTooSmall`] when the LU path has nowhere to work.
pub fn mat_inverse<T: Real>(
    x: &[T],
    y: &mut [T],
    n: usize,
    scratch: &mut [T],
) -> Result<(), MatError> {
    if x.len() != n * n || y.len() != n * n {
        return Err(MatError::BadDimensions);
    }
    match n {
        3 => {
            let m: [T; 9] = x.try_into().map_err(|_| MatError::BadDimensions)?;
            let inv = inverse3x3(&m).ok_or(MatError::Singular)?;
            y.copy_from_slice(&inv);
            Ok(())
        }
        4 => {
            let m: [T; 16] = x.try_into().map_err(|_| MatError::BadDimensions)?;
            let inv = inverse4x4(&m).ok_or(MatError::Singular)?;
            y.copy_from_slice(&inv);
            Ok(())
        }
        _ => mat_inverse_n(x, y, n, scratch),
    }
}

/// Row-major element read. A single place to get the indexing right.
#[inline]
fn at<T: Real>(m: &[T], n: usize, i: usize, j: usize) -> T {
    // Every caller stays inside n*n, checked at the entry points. Falling back
    // to zero rather than indexing keeps the workspace lint satisfied without
    // an unchecked index in flight code.
    m.get(i * n + j).copied().unwrap_or_else(T::zero)
}

#[inline]
fn set<T>(m: &mut [T], n: usize, i: usize, j: usize, v: T) {
    if let Some(e) = m.get_mut(i * n + j) {
        *e = v;
    }
}

/// The pivot matrix that puts the largest element of each column on the
/// diagonal, upstream `mat_pivot`.
fn mat_pivot<T: Real>(a: &[T], pivot: &mut [T], n: usize) {
    for i in 0..n {
        for j in 0..n {
            set(pivot, n, i, j, if i == j { T::one() } else { T::zero() });
        }
    }

    for i in 0..n {
        let mut max_j = i;
        for j in i..n {
            if at(a, n, j, i).abs() > at(a, n, max_j, i).abs() {
                max_j = j;
            }
        }
        if max_j != i {
            for k in 0..n {
                let tmp = at(pivot, n, i, k);
                let other = at(pivot, n, max_j, k);
                set(pivot, n, i, k, other);
                set(pivot, n, max_j, k, tmp);
            }
        }
    }
}

/// Inverse of a lower-triangular matrix by forward substitution, upstream
/// `mat_forward_sub`.
fn mat_forward_sub<T: Real>(l: &[T], out: &mut [T], n: usize) {
    for e in out.iter_mut() {
        *e = T::zero();
    }
    for i in 0..n {
        set(out, n, i, i, T::one() / at(l, n, i, i));
        for j in (i + 1)..n {
            for k in i..j {
                let v = at(out, n, j, i) - at(l, n, j, k) * at(out, n, k, i);
                set(out, n, j, i, v);
            }
            let v = at(out, n, j, i) / at(l, n, j, j);
            set(out, n, j, i, v);
        }
    }
}

/// Inverse of an upper-triangular matrix by back substitution, upstream
/// `mat_back_sub`.
fn mat_back_sub<T: Real>(u: &[T], out: &mut [T], n: usize) {
    for e in out.iter_mut() {
        *e = T::zero();
    }
    for i in (0..n).rev() {
        set(out, n, i, i, T::one() / at(u, n, i, i));
        for j in (0..i).rev() {
            let mut k = i;
            while k > j {
                let v = at(out, n, j, i) - at(u, n, j, k) * at(out, n, k, i);
                set(out, n, j, i, v);
                k -= 1;
            }
            let v = at(out, n, j, i) / at(u, n, j, j);
            set(out, n, j, i, v);
        }
    }
}

/// `A × P = L × U`, upstream `mat_LU_decompose`.
///
/// `a_prime` is scratch for the pivoted copy of `A`.
fn mat_lu_decompose<T: Real>(
    a: &[T],
    l: &mut [T],
    u: &mut [T],
    p: &mut [T],
    a_prime: &mut [T],
    n: usize,
) {
    for e in l.iter_mut() {
        *e = T::zero();
    }
    for e in u.iter_mut() {
        *e = T::zero();
    }
    mat_pivot(a, p, n);

    // a_prime = P * A
    for i in 0..n {
        for j in 0..n {
            let mut acc = T::zero();
            for k in 0..n {
                acc = acc + at(p, n, i, k) * at(a, n, k, j);
            }
            set(a_prime, n, i, j, acc);
        }
    }

    for i in 0..n {
        set(l, n, i, i, T::one());
    }
    for i in 0..n {
        for j in 0..n {
            if j <= i {
                let mut v = at(a_prime, n, j, i);
                for k in 0..j {
                    v = v - at(l, n, j, k) * at(u, n, k, i);
                }
                set(u, n, j, i, v);
            }
            if j >= i {
                let mut v = at(a_prime, n, j, i);
                for k in 0..i {
                    v = v - at(l, n, j, k) * at(u, n, k, i);
                }
                set(l, n, j, i, v / at(u, n, i, i));
            }
        }
    }
}

/// Inverse by LU decomposition, upstream `mat_inverseN`.
///
/// `inv = inv(U) × inv(L) × P`.
fn mat_inverse_n<T: Real>(
    a: &[T],
    inv: &mut [T],
    n: usize,
    scratch: &mut [T],
) -> Result<(), MatError> {
    let needed = scratch_len(n);
    if scratch.len() < needed {
        return Err(MatError::ScratchTooSmall {
            given: scratch.len(),
            needed,
        });
    }
    let sq = n * n;
    // five n*n workspaces carved out of the caller's buffer, in place of
    // upstream's five heap allocations
    let (l, rest) = scratch.split_at_mut(sq);
    let (u, rest) = rest.split_at_mut(sq);
    let (p, rest) = rest.split_at_mut(sq);
    let (l_inv, rest) = rest.split_at_mut(sq);
    let (u_inv, _) = rest.split_at_mut(sq);

    // `inv` doubles as the scratch for the pivoted copy of A during
    // decomposition; it is fully overwritten below.
    mat_lu_decompose(a, l, u, p, inv, n);
    mat_forward_sub(l, l_inv, n);
    mat_back_sub(u, u_inv, n);

    // l is free now; reuse it for inv(U) * inv(L)
    for i in 0..n {
        for j in 0..n {
            let mut acc = T::zero();
            for k in 0..n {
                acc = acc + at(u_inv, n, i, k) * at(l_inv, n, k, j);
            }
            set(l, n, i, j, acc);
        }
    }
    // then apply the pivot: inv = (inv(U) * inv(L)) * P
    for i in 0..n {
        for j in 0..n {
            let mut acc = T::zero();
            for k in 0..n {
                acc = acc + at(l, n, i, k) * at(p, n, k, j);
            }
            set(inv, n, i, j, acc);
        }
    }

    // Upstream's sanity check: a decomposition that divided by a zero pivot
    // produces NaN or infinity rather than reporting failure earlier.
    if inv.iter().any(|v| v.is_nan() || v.is_infinite()) {
        return Err(MatError::Singular);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::indexing_slicing,
        reason = "indexes fixed-size arrays declared in the test itself"
    )]
    #![allow(
        clippy::float_cmp,
        reason = "identity checks on exactly representable values"
    )]

    use super::*;

    /// `A × A⁻¹` must be the identity, for every dimension the dispatch has a
    /// separate path for. This is independent of upstream: it checks the maths,
    /// not the transcription, so it would catch a wrong index in the generated
    /// cofactor expansion that a parity test could only catch if upstream were
    /// right.
    #[test]
    fn inverse_times_original_is_identity() {
        for n in [3usize, 4, 5, 6, 9] {
            // diagonally dominant, so it is well conditioned and invertible
            let mut a = [0.0f64; 81];
            for i in 0..n {
                for j in 0..n {
                    a[i * n + j] = if i == j {
                        10.0 + i as f64
                    } else {
                        1.0 / (1.0 + (i as f64 - j as f64).abs())
                    };
                }
            }
            let a = &a[..n * n];

            let mut inv = [0.0f64; 81];
            let mut scratch = [0.0f64; 5 * 81];
            mat_inverse(a, &mut inv[..n * n], n, &mut scratch)
                .unwrap_or_else(|e| panic!("n={n}: {e:?}"));

            let mut prod = [0.0f64; 81];
            mat_mul(a, &inv[..n * n], &mut prod[..n * n], n).expect("mul");
            for i in 0..n {
                for j in 0..n {
                    let want = if i == j { 1.0 } else { 0.0 };
                    let got = prod[i * n + j];
                    assert!(
                        (got - want).abs() < 1e-9,
                        "n={n} ({i},{j}): {got} != {want}"
                    );
                }
            }
        }
    }

    #[test]
    fn singular_matrices_are_reported() {
        // a zero row makes every dimension singular
        for n in [3usize, 4, 5] {
            let mut a = [0.0f32; 25];
            for i in 0..n {
                for j in 0..n {
                    a[i * n + j] = if i == 1 {
                        0.0
                    } else {
                        (i * n + j) as f32 + 1.0
                    };
                }
            }
            let mut inv = [0.0f32; 25];
            let mut scratch = [0.0f32; 5 * 25];
            assert_eq!(
                mat_inverse(&a[..n * n], &mut inv[..n * n], n, &mut scratch),
                Err(MatError::Singular),
                "n={n} with a zero row must be singular"
            );
        }
    }

    /// D-012: upstream cannot report this — it allocates and dereferences the
    /// result unchecked. The port cannot allocate, so a caller that supplies
    /// too little scratch is told so.
    #[test]
    fn d012_insufficient_scratch_is_reported_not_dereferenced() {
        let a = [1.0f32; 25];
        let mut inv = [0.0f32; 25];
        let mut scratch = [0.0f32; 4]; // far too small
        assert_eq!(
            mat_inverse(&a, &mut inv, 5, &mut scratch),
            Err(MatError::ScratchTooSmall {
                given: 4,
                needed: 125
            })
        );
    }

    /// The closed-form paths take no scratch, which is what makes them usable
    /// on the calibration path without a buffer to hand.
    #[test]
    fn three_and_four_need_no_scratch() {
        let a3 = [2.0f32, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 8.0];
        let mut inv3 = [0.0f32; 9];
        mat_inverse(&a3, &mut inv3, 3, &mut []).expect("3x3 needs no scratch");
        assert_eq!(inv3[0], 0.5);
        assert_eq!(inv3[4], 0.25);
        assert_eq!(inv3[8], 0.125);

        let mut a4 = [0.0f32; 16];
        for i in 0..4 {
            a4[i * 4 + i] = 2.0;
        }
        let mut inv4 = [0.0f32; 16];
        mat_inverse(&a4, &mut inv4, 4, &mut []).expect("4x4 needs no scratch");
        for i in 0..4 {
            assert_eq!(inv4[i * 4 + i], 0.5);
        }
    }

    #[test]
    fn dimension_mismatches_are_reported() {
        let a = [1.0f32; 9];
        let mut y = [0.0f32; 4];
        assert_eq!(
            mat_inverse(&a, &mut y, 3, &mut []),
            Err(MatError::BadDimensions)
        );
    }

    #[test]
    fn identity_is_its_own_inverse() {
        for n in [3usize, 4, 5, 6] {
            let mut a = [0.0f64; 36];
            mat_identity(&mut a[..n * n], n).expect("identity");
            let mut inv = [0.0f64; 36];
            let mut scratch = [0.0f64; 5 * 36];
            mat_inverse(&a[..n * n], &mut inv[..n * n], n, &mut scratch).expect("invert");
            for i in 0..n {
                for j in 0..n {
                    let want = if i == j { 1.0 } else { 0.0 };
                    assert!((inv[i * n + j] - want).abs() < 1e-12, "n={n} ({i},{j})");
                }
            }
        }
    }

    #[test]
    fn scratch_len_is_five_matrices() {
        assert_eq!(scratch_len(0), 0);
        assert_eq!(scratch_len(3), 45);
        assert_eq!(scratch_len(9), 405);
    }
}
