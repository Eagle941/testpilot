# Engineering Challenges

## Scenario catch-up can delay an MSFS frame

Each injection owns a sequential CSV reader and retains only the two samples
needed for interpolation. When an MSFS update arrives late, the current
scenario time may have passed several source samples. The cursor must then read
forward until its samples bracket the current time or the series reaches EOF.

This preserves correct interpolation and bounded memory, but the number of
synchronous CSV reads performed by one simulator update is not bounded. A large
time jump or a densely sampled scenario could therefore make that update take
long enough to delay an MSFS frame.

These goals cannot all be guaranteed simultaneously without another I/O or
scheduling mechanism:

- interpolate immediately at the current simulator time;
- support arbitrary source sample density and late frames;
- perform only a fixed number of reads per simulator frame;
- avoid loading scenario data in proportion to its duration.

The MVP favors correct, deterministic catch-up with bounded memory. If manual
integration testing shows simulator-frame delays, a later design must define an
explicit policy, such as background or chunked prefetching, or bounded catch-up
that temporarily suspends injection until every cursor reaches the current
scenario time. Such a policy must not present stale or incomplete injection as
a valid replay frame.
