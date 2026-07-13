# Delayed NAS synchronization

The GUI writes complete episodes to the local NVMe outbox first. This worker publishes
one READY outbox at a time to `Muka_NAS` only after capture has been idle long enough.

Normal GUI launches start the watcher automatically. Manual commands:

```bash
bash S_sync_nas.sh --once --dry-run
bash S_sync_nas.sh --once
bash S_sync_nas.sh --watch
```

Default paths:

```text
local cache: /home/pnp/Desktop/franka_record_cache
NAS root:    /home/pnp/Desktop/Muka_NAS
log:         /home/pnp/Desktop/franka_record_cache/sync.log
```

The publisher waits while `.capture-active/*.json` is fresh, while normalized system
load is above `0.5`, or while available memory is below `4096 MiB`. Copy bandwidth is
limited to `20 MiB/s`. Useful overrides:

```text
FRANKA_SYNC_IDLE_GRACE_SECONDS
FRANKA_SYNC_MAX_LOAD_PER_CPU
FRANKA_SYNC_MIN_AVAILABLE_MEMORY_MIB
FRANKA_SYNC_RATE_LIMIT_MIB_S
FRANKA_SYNC_CACHE_ROOT
FRANKA_SYNC_NAS_ROOT
```

Each episode is copied into a hidden `.partial-<index>-<uuid>` directory. File sizes and
SHA-256 digests are checked before and after the atomic final rename. The local outbox is
deleted only after the final NAS directory passes the second verification. Network,
mount, checksum, process, or power failures leave the local outbox available for retry.

Final numeric indices are allocated while holding a NAS directory lock. This prevents
two capture hosts from claiming the same `task/quality/index` during publication.
