use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::hint::black_box;

#[derive(Clone, Copy, Default)]
pub struct Counts {
    pub allocations: u64,
    pub reallocations: u64,
    pub requested_bytes: u64,
    pub deallocations: u64,
}

#[derive(Clone, Copy, Default)]
struct Tracking {
    enabled: bool,
    counts: Counts,
}

thread_local! {
    // Const initialization and Cell operations do not allocate. Keeping this
    // thread-local excludes background work in an attached profiler or harness.
    static TRACKING: Cell<Tracking> = const { Cell::new(Tracking {
        enabled: false,
        counts: Counts { allocations: 0, reallocations: 0, requested_bytes: 0, deallocations: 0 },
    }) };
}

fn update(f: impl FnOnce(&mut Counts)) {
    // Allocation can occur during TLS teardown, when this key is unavailable.
    let _ = TRACKING.try_with(|tracking| {
        let mut state = tracking.get();
        if state.enabled {
            f(&mut state.counts);
            tracking.set(state);
        }
    });
}

pub struct CountingAllocator;

// SAFETY: all allocation operations are delegated to System with unchanged
// arguments. Instrumentation neither allocates nor unwinds inside these hooks.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: the caller supplies the layout required by GlobalAlloc.
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            update(|counts| {
                counts.allocations = counts.allocations.saturating_add(1);
                counts.requested_bytes =
                    counts.requested_bytes.saturating_add(layout.size() as u64);
            });
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        // SAFETY: the caller supplies a valid allocation layout.
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            update(|counts| {
                counts.allocations = counts.allocations.saturating_add(1);
                counts.requested_bytes =
                    counts.requested_bytes.saturating_add(layout.size() as u64);
            });
        }
        pointer
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        // SAFETY: GlobalAlloc's caller guarantees ownership, layout and size.
        let result = unsafe { System.realloc(pointer, layout, size) };
        if !result.is_null() {
            update(|counts| {
                counts.reallocations = counts.reallocations.saturating_add(1);
                counts.requested_bytes = counts.requested_bytes.saturating_add(size as u64);
            });
        }
        result
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        update(|counts| counts.deallocations = counts.deallocations.saturating_add(1));
        // SAFETY: pointer and layout are passed unchanged to their allocator.
        unsafe { System.dealloc(pointer, layout) };
    }
}

pub struct Measurement;

impl Measurement {
    pub fn begin() -> Self {
        TRACKING.with(|tracking| {
            assert!(!tracking.get().enabled, "nested allocation measurement");
            tracking.set(Tracking {
                enabled: true,
                counts: Counts::default(),
            });
        });
        Self
    }

    pub fn finish(self) -> Counts {
        TRACKING.with(|tracking| {
            let state = tracking.get();
            tracking.set(Tracking {
                enabled: false,
                ..state
            });
            state.counts
        })
    }
}

impl Drop for Measurement {
    fn drop(&mut self) {
        TRACKING.with(|tracking| {
            tracking.set(Tracking {
                enabled: false,
                ..tracking.get()
            })
        });
    }
}

/// Exercises fresh allocation, reallocation and freeing before trusting reports.
pub fn self_check() {
    let measurement = Measurement::begin();
    let mut bytes = Vec::<u8>::with_capacity(16);
    bytes.extend_from_slice(&[0; 16]);
    bytes.reserve_exact(16);
    black_box(bytes.as_ptr());
    drop(bytes);
    let counts = measurement.finish();
    assert_eq!(counts.allocations, 1);
    assert_eq!(counts.reallocations, 1);
    assert_eq!(counts.requested_bytes, 48);
    assert_eq!(counts.deallocations, 1);
    let empty = Measurement::begin().finish();
    assert_eq!(
        empty.requested_bytes, 0,
        "counters must reset between measurements"
    );
}
