# Proofs

Machine-checked claims about code in this tree, using [CBMC][] — a bounded
model checker for C. Run them with `./proofs/run.sh`, or through
`./tests/check.sh`, which has a stage for them and skips when CBMC is absent.

[CBMC]: https://www.cprover.org/cbmc/

## Why these, and not tests

The tests here are good and they sample. When a safety-relevant function was
lifted out of a file to make it testable, the commit claimed "behaviour
unchanged" on the strength of a character-identical diff and a dozen passing
cases. That is an argument, not a proof, and the thing it is arguing about is
easy to state formally: two functions, same inputs, same outputs, always.

So each proof here is attached to a specific claim somebody already made in a
commit message.

| proof | claims |
|---|---|
| `imu_freeze` | moving the IMU dead-bus detector out of `imu_thread.c` changed nothing, for every state and every sample; and four properties of it |

## By induction, not by exploring sequences

The first attempt at `imu_freeze` ran 34 steps with a free choice of sample at
each — 2³⁴ paths, which did not finish — and to keep that tractable it drew
samples from a two-element alphabet, so it would have proven something about
the abstraction rather than about the code.

One step from an arbitrary state is cheaper and claims more: equal states in,
one step each, equal states out, plus a base case that a reset leaves both
equal. That is equivalence for every sequence of every length, with no bound
and no alphabet, and it runs in under two seconds.

## What a passing proof here does and does not mean

Worth being exact, because "proved" invites more than is on offer.

**It does** mean: for all inputs the model admits, the two implementations
agree and the stated properties hold, with no undefined behaviour that CBMC
checks for — bounds, pointer validity, conversions.

**It does not** mean the compiler preserves it, that the hardware behaves, or
that the function is the right function to be computing. And a *passing* check
is weaker evidence than a failing one. The T-LLM Compiler paper
([arXiv:2608.14953][tllm]) measured CBMC used for C equivalence checking at
8.4% false rejection against **33.8% false acceptance** — it misses incorrect
transforms when the search space is small. Their floats were intractable
entirely, and their arrays had to stay under 15×15.

[tllm]: https://arxiv.org/abs/2608.14953

That asymmetry is why the proofs here are mutation-tested like the tests are.
A proof that cannot fail is the same hazard as a test that cannot fail, with
more authority attached. For `imu_freeze`: an off-by-one on the threshold, a
counter that no longer saturates, gyro dropped from the comparison, and a
forgotten store of the gyro half are each caught, by the assertion written for
them. The last of those is caught by the proof and **not** by the unit test,
which is the clearest argument for having both.

There is also a vacuity probe in the procedure: asserting `0` at the end of the
harness has to fail. An unsatisfiable assumption would otherwise make every
assertion pass trivially.

## What is deliberately not proven

`util/mc_limits.c` is the other extraction of the same kind, and it is IEEE
float arithmetic. Full equivalence there is where this technique stops being
cheap — which the paper above found as well. Properties are still in reach and
worth doing: that the outputs stay finite for finite inputs, that the usable
limit never exceeds the configured ceiling, and that the ramps are monotonic
in voltage. The unit test checks those at 24 points; a prover would check them
everywhere.

`imu/ahrs.c` is out of reach. `atan2f`, `sinf` and `cosf` are not things an SMT
solver reasons about usefully, and the properties that matter there — a unit
quaternion, gravity rotating back onto the reading — are already checked over
eight orientations by `tests/ahrs_seed`.

None of this is a safety qualification. ISO 26262 or DO-178C want a tool
qualification argument, traceability and a great deal else. This is a handful
of theorems about two hundred lines of C, which is worth having on its own
terms.
