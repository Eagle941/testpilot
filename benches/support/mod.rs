use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

pub mod boundary;
pub mod full_frame;

static NEXT_DIRECTORY: AtomicU64 = AtomicU64::new(0);

/// Owns only a directory created by this benchmark. Drop open files before it.
pub struct TempDirectory(pub PathBuf);

impl TempDirectory {
    pub fn new() -> Self {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("host clock after epoch")
            .as_nanos();
        let id = NEXT_DIRECTORY.fetch_add(1, Ordering::Relaxed);
        let path =
            std::env::temp_dir().join(format!("replay-bench-{}-{nanos}-{id}", std::process::id()));
        fs::create_dir(&path).expect("create unique benchmark directory");
        Self(path)
    }
}

impl Drop for TempDirectory {
    fn drop(&mut self) {
        // Failing cleanup must be visible; a benchmark must not leak data per batch.
        if let Err(error) = fs::remove_dir_all(&self.0) {
            if std::thread::panicking() {
                eprintln!("benchmark cleanup failed for {}: {error}", self.0.display());
            } else {
                panic!("benchmark cleanup failed for {}: {error}", self.0.display());
            }
        }
    }
}
